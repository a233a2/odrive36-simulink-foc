#include "foc.h"
#include "config.h"
#include "odrive_foc_core.h"
#include "odrive_foc_contract.h"
#include <math.h>

_Static_assert(ODRIVE_FOC_ABI_REVISION == 2u, "Update the STM32 adapter for this interface revision");
_Static_assert(ODRIVE_FOC_SAMPLE_HZ == PWM_FREQUENCY_HZ, "Model tick must match the ADC IRQ");
_Static_assert(ODRIVE_FOC_MOTION_HZ == MOTION_FREQUENCY_HZ, "Motion tick must match board configuration");
_Static_assert(sizeof(real32_T) == sizeof(float), "Expected single-precision generated C");
_Static_assert(sizeof(ExtU_odrive_foc_core_T) == 31u * sizeof(float), "Input ports changed");
_Static_assert(sizeof(ExtY_odrive_foc_core_T) == 13u * sizeof(float), "Output ports changed");
_Static_assert(FOC_ALIGN == 0 && FOC_TORQUE == 1 && FOC_SPEED == 2 && FOC_POSITION == 3,
               "Control mode values must match the model");

void foc_current_init(void) {
    odrive_foc_core_initialize();
}

bool foc_motion_step(const FocMotionInput *input, FocMotionState *state, FocCurrentOutput *output) {
    if (!foc_motion_inputs_valid(input,state)) return false;

    const FocCurrentInput *c=&input->current;
    /* The ADC ISR is the sole owner of the generated I/O buffers. All persistent
     * controller state, including the 1 kHz phase, belongs to the motor axis. */
    odrive_foc_core_U = (ExtU_odrive_foc_core_T){
        .i_phase_bc_a={c->ib_a,c->ic_a},
        .theta_e_rad=c->theta_e_rad,
        .i_override_dq_a={c->reference_a.d,c->reference_a.q},
        .vbus_v=c->vbus_v,
        .vmax_phase_v=c->phase_voltage_limit_v,
        .gain_pi={input->current_kp.d,input->current_ki.d,input->current_kp.q,input->current_ki.q},
        .mechanical_feedback={input->position_rad,input->speed_rad_s},
        .motion_command={(float)input->mode,input->target},
        .motion_gains={input->speed_kp,input->speed_ki,input->position_kp},
        .motion_limits={CURRENT_LIMIT_A,VELOCITY_LIMIT_RAD_S,ACCEL_LIMIT_RAD_S2,
                        IQ_SLEW_A_PER_S,input->torque_constant_nm_a},
        .previous_state={state->current_feedback_a.d,state->current_feedback_a.q,
                         state->current_integral_v.d,state->current_integral_v.q,
                         state->speed_integral_a,state->speed_reference_rad_s,
                         state->iq_reference_a,(float)state->motion_tick}
    };
    odrive_foc_core_step();

    const ExtY_odrive_foc_core_T *result=&odrive_foc_core_Y;
    const float *next=result->next_state;
    for (unsigned i=0;i<8;i++) {
        if (!isfinite(next[i])) return false;
    }
    for (unsigned i=0;i<3;i++) {
        if (!(result->duty_abc[i]>=0.10f && result->duty_abc[i]<=0.80f)) return false;
    }
    if (next[7]<0.0f || next[7]>=MOTION_DIVIDER) return false;
    uint32_t phase=(uint32_t)next[7];
    if ((float)phase!=next[7] || fabsf(next[4])>CURRENT_LIMIT_A+1e-6f ||
        fabsf(next[5])>VELOCITY_LIMIT_RAD_S+1e-6f || fabsf(next[6])>CURRENT_LIMIT_A+1e-6f)
        return false;

    float vd=result->v_dq_v[0],vq=result->v_dq_v[1];
    float maximum=limitf(c->phase_voltage_limit_v,0.0f,c->vbus_v*0.30f);
    /* Check output limits before committing the state or duty cycle. */
    if (!isfinite(vd)||!isfinite(vq)||vd*vd+vq*vq>maximum*maximum+1e-5f||
        fabsf(next[2])>maximum+1e-5f||fabsf(next[3])>maximum+1e-5f) return false;

    *state=(FocMotionState){
        .current_feedback_a={next[0],next[1]},.current_integral_v={next[2],next[3]},
        .speed_integral_a=next[4],.speed_reference_rad_s=next[5],.iq_reference_a=next[6],.motion_tick=phase
    };
    *output=(FocCurrentOutput){
        .feedback_a={next[0],next[1]},.voltage_v={vd,vq},
        .duty={result->duty_abc[0],result->duty_abc[1],result->duty_abc[2]}
    };
    return true;
}
