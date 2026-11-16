#include "foc.h"
#include "config.h"
#include <math.h>

/* 有限数输入的直接限幅，保留NaN以让上层数值保护捕获，避免中断中反复调用libm。 */
float limitf(float x, float low, float high) { return x<low?low:(x>high?high:x); }
float wrap_2pi(float x) {
    if (x>=0.0f&&x<TWO_PI_F) return x;
    x = fmodf(x, TWO_PI_F);
    return x < 0.0f ? x + TWO_PI_F : x;
}
float wrap_pi(float x) { return wrap_2pi(x + PI_F) - PI_F; }
float approach(float present, float target, float step) {
    return present + limitf(target - present, -step, step);
}
int32_t encoder_delta(uint16_t previous, uint16_t current) {
    int32_t delta = (int32_t)current - previous;
    if (delta > 2048) delta -= 4096;
    if (delta < -2048) delta += 4096;
    return delta;
}

/* 条件积分：输出饱和且误差继续推动饱和时，不累加积分。 */
float pi_step(PiController *pi, float error, float dt, float maximum) {
    float next = limitf(pi->integral + pi->ki * error * dt, -maximum, maximum);
    float output = pi->kp * error + next;
    if (fabsf(output) <= maximum || output * error < 0.0f) pi->integral = next;
    return limitf(pi->kp * error + pi->integral, -maximum, maximum);
}

/* 原理图只测 B/C 相。先由 ia+ib+ic=0 重构 A 相，再作 Clarke/Park。 */
Dq clarke_park(float ib, float ic, float theta) {
    return clarke_park_trig(ib,ic,foc_angle(theta));
}
AngleTrig foc_angle(float theta) { return (AngleTrig){sinf(theta),cosf(theta)}; }
Dq clarke_park_trig(float ib,float ic,AngleTrig angle) {
    float alpha = -ib - ic;
    float beta = (ib - ic) * 0.57735026919f;
    float s = angle.s, c = angle.c;
    return (Dq){alpha * c + beta * s, -alpha * s + beta * c};
}

bool limit_vector(float *d, float *q, float maximum) {
    /* 这里是已限幅到几伏的电压矢量，不需要hypotf的一般极大数防溢出路径。
     * 未饱和时不求平方根，饱和时才用FPU sqrt。 */
    float length_squared = (*d)*(*d)+(*q)*(*q);
    if (length_squared <= maximum*maximum) return false;
    float ratio = maximum / sqrtf(length_squared);
    *d *= ratio;
    *q *= ratio;
    return true;
}

/* 三次谐波/共模注入实现连续 SVPWM，保留低桥电流采样窗口。
 * 前级矢量电压必须限制在 bus*0.30 内；末端占空比约束是最后防线。 */
PhaseDuty inverse_park_svm(float vd, float vq, float theta, float bus) {
    return inverse_park_svm_trig(vd,vq,foc_angle(theta),bus);
}
PhaseDuty inverse_park_svm_trig(float vd,float vq,AngleTrig angle,float bus) {
    float s = angle.s, c = angle.c;
    float alpha = vd * c - vq * s, beta = vd * s + vq * c;
    float a = alpha, b = -0.5f * alpha + 0.86602540378f * beta;
    float cc = -0.5f * alpha - 0.86602540378f * beta;
    float maximum=a>b?a:b,minimum=a<b?a:b;
    maximum=maximum>cc?maximum:cc; minimum=minimum<cc?minimum:cc;
    float common = 0.5f * (maximum+minimum);
    float inv = 1.0f / (bus>1.0f?bus:1.0f);
    return (PhaseDuty){limitf(0.5f + (a-common)*inv, 0.10f, 0.80f),
                       limitf(0.5f + (b-common)*inv, 0.10f, 0.80f),
                       limitf(0.5f + (cc-common)*inv, 0.10f, 0.80f)};
}

/* The native current loop also serves as the reference for independent tests. */
bool foc_current_step(const FocCurrentInput *input, Dq previous_feedback_a,
                      PiController *d_pi, PiController *q_pi, FocCurrentOutput *output) {
    if (!isfinite(input->theta_e_rad)) return false;
    AngleTrig angle = foc_angle(input->theta_e_rad);
    Dq measured = clarke_park_trig(input->ib_a, input->ic_a, angle);
    Dq feedback = {
        previous_feedback_a.d + 0.15f * (measured.d - previous_feedback_a.d),
        previous_feedback_a.q + 0.15f * (measured.q - previous_feedback_a.q)
    };
    float bus_limit = input->vbus_v * 0.30f;
    float maximum = input->phase_voltage_limit_v < bus_limit ? input->phase_voltage_limit_v : bus_limit;
    PiController next_d = *d_pi, next_q = *q_pi;
    float vd = pi_step(&next_d, input->reference_a.d - feedback.d, CONTROL_DT, maximum);
    float vq = pi_step(&next_q, input->reference_a.q - feedback.q, CONTROL_DT, maximum);
    float unlimited_d = vd, unlimited_q = vq;
    if (limit_vector(&vd, &vq, maximum)) {
        next_d.integral = limitf(next_d.integral + vd - unlimited_d, -maximum, maximum);
        next_q.integral = limitf(next_q.integral + vq - unlimited_q, -maximum, maximum);
    }
    if (!isfinite(vd) || !isfinite(vq)) return false;

    d_pi->integral = next_d.integral;
    q_pi->integral = next_q.integral;
    *output = (FocCurrentOutput){feedback, {vd, vq},
        inverse_park_svm_trig(vd, vq, angle, input->vbus_v)};
    return true;
}

bool foc_motion_inputs_valid(const FocMotionInput *input, const FocMotionState *state) {
    const FocCurrentInput *c = &input->current;
    return isfinite(c->ib_a) && isfinite(c->ic_a) && isfinite(c->theta_e_rad) &&
           isfinite(c->vbus_v) && c->vbus_v >= 1.0f &&
           isfinite(c->phase_voltage_limit_v) && c->phase_voltage_limit_v > 0.0f &&
           isfinite(c->reference_a.d) && isfinite(c->reference_a.q) &&
           (unsigned)input->mode <= FOC_POSITION && isfinite(input->target) &&
           isfinite(input->position_rad) && isfinite(input->speed_rad_s) &&
           input->torque_constant_nm_a >= TORQUE_CONSTANT_MIN &&
           input->torque_constant_nm_a <= TORQUE_CONSTANT_MAX &&
           isfinite(input->current_kp.d) && isfinite(input->current_kp.q) &&
           isfinite(input->current_ki.d) && isfinite(input->current_ki.q) &&
           isfinite(input->speed_kp) && isfinite(input->speed_ki) && isfinite(input->position_kp) &&
           isfinite(state->current_feedback_a.d) && isfinite(state->current_feedback_a.q) &&
           isfinite(state->current_integral_v.d) && isfinite(state->current_integral_v.q) &&
           isfinite(state->speed_integral_a) && isfinite(state->speed_reference_rad_s) &&
           isfinite(state->iq_reference_a) && state->motion_tick < MOTION_DIVIDER;
}

#ifndef CM_USE_SIMULINK_FOC
void foc_current_init(void) {}

static void update_motion_reference(const FocMotionInput *input, FocMotionState *next) {
    if (input->mode == FOC_ALIGN) {
        next->speed_integral_a = next->speed_reference_rad_s = next->iq_reference_a = 0.0f;
        next->motion_tick = 0u;
        return;
    }
    if (++next->motion_tick < MOTION_DIVIDER) return;
    next->motion_tick = 0u;
    if (input->mode == FOC_TORQUE) {
        float iq = limitf(input->target / input->torque_constant_nm_a, -CURRENT_LIMIT_A, CURRENT_LIMIT_A);
        next->iq_reference_a = approach(next->iq_reference_a, iq, IQ_SLEW_A_PER_S * MOTION_DT);
        next->speed_integral_a = next->speed_reference_rad_s = 0.0f;
        return;
    }
    float requested = input->target;
    if (input->mode == FOC_POSITION) requested = input->position_kp * (requested - input->position_rad);
    requested = limitf(requested, -VELOCITY_LIMIT_RAD_S, VELOCITY_LIMIT_RAD_S);
    next->speed_reference_rad_s = approach(next->speed_reference_rad_s, requested, ACCEL_LIMIT_RAD_S2 * MOTION_DT);
    PiController speed = {input->speed_kp, input->speed_ki, next->speed_integral_a};
    next->iq_reference_a = pi_step(&speed, next->speed_reference_rad_s - input->speed_rad_s,
                                   MOTION_DT, CURRENT_LIMIT_A);
    next->speed_integral_a = speed.integral;
}

bool foc_motion_step(const FocMotionInput *input, FocMotionState *state, FocCurrentOutput *output) {
    if (!foc_motion_inputs_valid(input, state)) return false;
    FocMotionState next = *state;
    update_motion_reference(input, &next);
    FocCurrentInput current = input->current;
    if (input->mode != FOC_ALIGN) current.reference_a = (Dq){0.0f, next.iq_reference_a};
    PiController d = {input->current_kp.d, input->current_ki.d, next.current_integral_v.d};
    PiController q = {input->current_kp.q, input->current_ki.q, next.current_integral_v.q};
    if (!foc_current_step(&current, next.current_feedback_a, &d, &q, output)) return false;
    next.current_feedback_a = output->feedback_a;
    next.current_integral_v = (Dq){d.integral, q.integral};
    *state = next;
    return true;
}
#endif
