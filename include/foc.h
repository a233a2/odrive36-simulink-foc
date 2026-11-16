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

/* The first mode is reserved for the board's electrical calibration. */
typedef enum { FOC_ALIGN = 0, FOC_TORQUE, FOC_SPEED, FOC_POSITION } FocMode;

typedef struct {
    FocCurrentInput current;
    FocMode mode;
    float target;                         /* N m, mechanical rad/s, or mechanical rad */
    float position_rad, speed_rad_s;
    float torque_constant_nm_a;
    Dq current_kp, current_ki;
    float speed_kp, speed_ki, position_kp;
} FocMotionInput;

/* All state belongs to one axis. The caller resets it when changing modes. */
typedef struct {
    Dq current_feedback_a, current_integral_v;
    float speed_integral_a, speed_reference_rad_s, iq_reference_a;
    uint32_t motion_tick;
} FocMotionState;

/* Initialize before enabling ADC interrupts. */
void foc_current_init(void);

/* The caller owns the filter and PI state of each axis. A false result is a
 * numeric fault. The caller must shut down the power stage before returning. */
bool foc_current_step(const FocCurrentInput *input, Dq previous_feedback_a,
                      PiController *d_pi, PiController *q_pi, FocCurrentOutput *output);
bool foc_motion_inputs_valid(const FocMotionInput *input, const FocMotionState *state);
bool foc_motion_step(const FocMotionInput *input, FocMotionState *state, FocCurrentOutput *output);

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
