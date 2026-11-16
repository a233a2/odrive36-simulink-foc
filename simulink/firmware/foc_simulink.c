#include "foc.h"
#include "config.h"
#include "odrive_foc_core.h"
#include "odrive_foc_contract.h"
#include <math.h>

_Static_assert(ODRIVE_FOC_ABI_REVISION == 1u, "Update the STM32 adapter for this interface revision");
_Static_assert(ODRIVE_FOC_SAMPLE_HZ == PWM_FREQUENCY_HZ, "Model sample time must match the ADC IRQ");
_Static_assert(sizeof(real32_T) == sizeof(float), "Expected single-precision generated C");
_Static_assert(sizeof(ExtU_odrive_foc_core_T) == 15u * sizeof(float), "Input ports changed");
_Static_assert(sizeof(ExtY_odrive_foc_core_T) == 9u * sizeof(float), "Output ports changed");

void foc_current_init(void) {
    odrive_foc_core_initialize();
}

bool foc_current_step(const FocCurrentInput *input, Dq previous_feedback_a,
                      PiController *d_pi, PiController *q_pi, FocCurrentOutput *output) {
    if (!isfinite(input->theta_e_rad) || !isfinite(input->ib_a) || !isfinite(input->ic_a) ||
        !isfinite(input->vbus_v) || input->vbus_v < 1.0f ||
        !isfinite(input->phase_voltage_limit_v) || input->phase_voltage_limit_v <= 0.0f ||
        !isfinite(input->reference_a.d) || !isfinite(input->reference_a.q) ||
        !isfinite(previous_feedback_a.d) || !isfinite(previous_feedback_a.q) ||
        !isfinite(d_pi->kp) || !isfinite(d_pi->ki) || !isfinite(d_pi->integral) ||
        !isfinite(q_pi->kp) || !isfinite(q_pi->ki) || !isfinite(q_pi->integral)) return false;

    /* ADC IRQ is the sole owner of these scratch buffers. Each iteration
     * writes the entire input, including the selected axis's prior state. */
    odrive_foc_core_U = (ExtU_odrive_foc_core_T){
        .i_phase_bc_a = {input->ib_a, input->ic_a},
        .theta_e_rad = input->theta_e_rad,
        .i_ref_dq_a = {input->reference_a.d, input->reference_a.q},
        .vbus_v = input->vbus_v,
        .vmax_phase_v = input->phase_voltage_limit_v,
        .gain_pi = {d_pi->kp, d_pi->ki, q_pi->kp, q_pi->ki},
        .previous_state = {previous_feedback_a.d, previous_feedback_a.q, d_pi->integral, q_pi->integral}
    };
    odrive_foc_core_step();

    const ExtY_odrive_foc_core_T *result = &odrive_foc_core_Y;
    for (unsigned i = 0; i < 4; ++i) {
        if (!isfinite(result->next_state[i])) return false;
    }
    for (unsigned i = 0; i < 3; ++i) {
        if (!(result->duty_abc[i] >= 0.10f && result->duty_abc[i] <= 0.80f)) return false;
    }
    float vd = result->v_dq_v[0], vq = result->v_dq_v[1];
    float bus_limit = input->vbus_v * 0.30f;
    float maximum = input->phase_voltage_limit_v < bus_limit ? input->phase_voltage_limit_v : bus_limit;
    /* Check the model outputs before committing any state. The tolerance is
     * for single-precision rounding in the vector norm and integrators. */
    if (!isfinite(vd) || !isfinite(vq) || vd * vd + vq * vq > maximum * maximum + 1e-5f ||
        fabsf(result->next_state[2]) > maximum + 1e-5f ||
        fabsf(result->next_state[3]) > maximum + 1e-5f) return false;

    d_pi->integral = result->next_state[2];
    q_pi->integral = result->next_state[3];
    *output = (FocCurrentOutput){
        .feedback_a = {result->next_state[0], result->next_state[1]},
        .voltage_v = {vd, vq},
        .duty = {result->duty_abc[0], result->duty_abc[1], result->duty_abc[2]}
    };
    return true;
}
