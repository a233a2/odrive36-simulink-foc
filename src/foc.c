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

#ifndef CM_USE_SIMULINK_FOC
void foc_current_init(void) {}

/* Native current loop used by the existing firmware build. */
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
#endif
