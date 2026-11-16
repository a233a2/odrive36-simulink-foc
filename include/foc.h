#ifndef CM_FOC_H
#define CM_FOC_H
#include <stdbool.h>
#include <stdint.h>

typedef struct { float kp, ki, integral; } PiController;
typedef struct { float a, b, c; } PhaseDuty;
typedef struct { float d, q; } Dq;
typedef struct { float s, c; } AngleTrig;

/* One ADC tick for one axis. Currents are in A, angles in electrical rad,
 * bus and winding voltages in V, and PWM duties are ratios in [0, 1]. */
typedef struct {
    float ib_a, ic_a, theta_e_rad, vbus_v, phase_voltage_limit_v;
    Dq reference_a;
} FocCurrentInput;

typedef struct {
    Dq feedback_a;
    Dq voltage_v;
    PhaseDuty duty;
} FocCurrentOutput;

/* Initialize before enabling ADC interrupts. */
void foc_current_init(void);

/* The caller owns the filter and PI state of each axis. A false result is a
 * numeric fault. The caller must shut down the power stage before returning. */
bool foc_current_step(const FocCurrentInput *input, Dq previous_feedback_a,
                      PiController *d_pi, PiController *q_pi, FocCurrentOutput *output);

float limitf(float x, float low, float high);
float wrap_pi(float x);
float wrap_2pi(float x);
float approach(float present, float target, float maximum_step);
int32_t encoder_delta(uint16_t previous, uint16_t current);
float pi_step(PiController *pi, float error, float dt, float limit);
Dq clarke_park(float ib, float ic, float theta);
AngleTrig foc_angle(float theta);
Dq clarke_park_trig(float ib,float ic,AngleTrig angle);
PhaseDuty inverse_park_svm(float vd, float vq, float theta, float bus);
PhaseDuty inverse_park_svm_trig(float vd,float vq,AngleTrig angle,float bus);
bool limit_vector(float *d, float *q, float maximum);
#endif
