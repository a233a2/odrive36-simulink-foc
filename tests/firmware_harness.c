/* 执行真正的ARM编译产物，检查控制数学、状态机、保护；不模拟实际功率硬件。 */
#include "app.h"
#include "board.h"
#include "encoder.h"
#include "config.h"
#include <math.h>
#include <string.h>

volatile EncoderReading g_encoder[2];
volatile uint32_t test_checks,test_failure;
static uint32_t clock_us;
static bool gate,pwm[2],fault_pin;
static float fake_bus=12.0f;
static uint16_t fake_ntc[2]={1000,1000};
static PhaseDuty last_duty[2];

void Reset_Handler(void) { for (;;) {} }
uint32_t micros(void) { return clock_us; }
uint32_t millis(void) { return clock_us/1000; }
void board_shutdown(void) { gate=false; pwm[0]=pwm[1]=false; }
void board_pwm_enable(unsigned a,bool on) { pwm[a]=on; }
void board_pwm_set(unsigned a,PhaseDuty d) { last_duty[a]=d; }
bool board_fault_pin(void) { return fault_pin; }
void board_read_slow(float *bus,uint16_t ntc[2]) { *bus=fake_bus;ntc[0]=fake_ntc[0];ntc[1]=fake_ntc[1]; }
bool drv_prepare(void) { gate=true; return true; }
void encoder_scan(unsigned axis,char *reply,unsigned capacity) { (void)axis;(void)capacity;reply[0]=0; }
void board_request_dfu(void) { for (;;) {} }

#define CHECK(c) do { ++test_checks; if (!(c)) {test_failure=__LINE__;return -1;} } while(0)

#ifdef CM_USE_SIMULINK_FOC
#include "foc_replay.h"

static int test_simulink_replay(void) {
    const unsigned count = sizeof(foc_replay) / sizeof(foc_replay[0]);
    foc_current_init();
    /* Revisit the first tick after all other inputs without reinitializing.
     * This models alternating calls with different per-axis state. */
    for (unsigned i = 0; i <= count; ++i) {
        const float *u = foc_replay[i % count].input;
        const float *y = foc_replay[i % count].output;
        FocCurrentInput sample = {.ib_a = u[0], .ic_a = u[1], .theta_e_rad = u[2],
            .vbus_v = u[5], .phase_voltage_limit_v = u[6], .reference_a = {u[3], u[4]}};
        PiController d = {u[7], u[8], u[13]}, q = {u[9], u[10], u[14]};
        FocCurrentOutput result;
        CHECK(foc_current_step(&sample, (Dq){u[11], u[12]}, &d, &q, &result));
        const float actual[] = {result.feedback_a.d, result.feedback_a.q, d.integral, q.integral,
            result.voltage_v.d, result.voltage_v.q, result.duty.a, result.duty.b, result.duty.c};
        for (unsigned channel = 0; channel < sizeof(actual) / sizeof(actual[0]); ++channel) {
            CHECK(fabsf(actual[channel] - y[channel]) < 5e-6f);
        }
    }
    FocCurrentInput invalid = {.theta_e_rad = NAN, .vbus_v = 12.0f, .phase_voltage_limit_v = 2.0f};
    PiController d = {1.0f, 2.0f, 0.25f}, q = {3.0f, 4.0f, -0.25f};
    FocCurrentOutput result = {.duty = {0.5f, 0.5f, 0.5f}};
    CHECK(!foc_current_step(&invalid, (Dq){0.0f, 0.0f}, &d, &q, &result));
    CHECK(d.integral == 0.25f && q.integral == -0.25f && result.duty.a == 0.5f);
    return 0;
}
#endif

static bool command(const char *s) { char result[768];return app_command(s,result,sizeof(result)); }
static void sensors(float angle) {
    for (unsigned a=0;a<2;a++) {
        g_encoder[a].valid=true;g_encoder[a].status=0x20;g_encoder[a].angle=angle;
        g_encoder[a].velocity=0;g_encoder[a].timestamp_us=clock_us;
    }
}
static void tick(const uint16_t raw[4],bool heartbeat) {
    clock_us+=50;
    if (heartbeat) g_app.last_command_ms=millis();
    for (unsigned a=0;a<2;a++) g_encoder[a].timestamp_us=clock_us;
    app_poll();app_adc_isr(raw);
}
static bool setup(void) {
    app_init();board_shutdown();clock_us=10000;fake_bus=12;fake_ntc[0]=fake_ntc[1]=1000;fault_pin=false;
    sensors(0);app_poll();
    if (!command("prepare")) return false;
    uint16_t raw[4]={2048,2048,2048,2048};
    for (unsigned i=0;i<OFFSET_SAMPLE_COUNT;i++) tick(raw,true);
    return g_app.offsets_ready&&!g_app.fault;
}

int run_tests(void) {
#ifdef CM_USE_SIMULINK_FOC
    if (test_simulink_replay() != 0) return -1;
#endif
    CHECK(encoder_delta(4090,3)==9);CHECK(encoder_delta(3,4090)==-9);
    CHECK(fabsf(wrap_pi(3*TWO_PI_F+0.2f)-0.2f)<1e-5f);
    for (unsigned i=0;i<500;i++) {
        float theta=i*0.031f;
        float d=0.13f,q=-0.09f;
        float alpha=d*cosf(theta)-q*sinf(theta),beta=d*sinf(theta)+q*cosf(theta);
        Dq actual=clarke_park(-0.5f*alpha+0.86602540378f*beta,-0.5f*alpha-0.86602540378f*beta,theta);
        CHECK(fabsf(actual.d-d)<1e-5f&&fabsf(actual.q-q)<1e-5f);
        PhaseDuty duty=inverse_park_svm(1.3f,-0.7f,theta,12);
        CHECK(duty.a>=0.1f&&duty.a<=0.8f&&duty.b>=0.1f&&duty.b<=0.8f&&duty.c>=0.1f&&duty.c<=0.8f);
        float aa=(2*duty.a-duty.b-duty.c)*4.0f,bb=(duty.b-duty.c)*12.0f*0.57735026919f;
        CHECK(fabsf(aa*cosf(theta)+bb*sinf(theta)-1.3f)<2e-5f);
        CHECK(fabsf(-aa*sinf(theta)+bb*cosf(theta)+0.7f)<2e-5f);
    }
    PiController pi={2,10,0};
    for (unsigned i=0;i<10000;i++) (void)pi_step(&pi,1,0.001f,0.2f);
    CHECK(fabsf(pi.integral)<1e-6f);CHECK(pi_step(&pi,-0.1f,0.001f,0.2f)<0);
    float d=3,q=4;CHECK(limit_vector(&d,&q,2));CHECK(fabsf(d-1.2f)<1e-6f&&fabsf(q-1.6f)<1e-6f);
    app_init();CHECK(!command("cal 0"));CHECK(!gate&&!pwm[0]&&!pwm[1]);
    CHECK(setup());CHECK(gate&&!pwm[0]&&!pwm[1]);CHECK(fabsf(g_app.current_offsets[0]-2048)<0.01f);
    CHECK(!command("position 0 0.1"));CHECK(!command("cal 2"));CHECK(!command("current 0 nan"));
    CHECK(!command("current 0 1000"));
    CHECK(!command("cal 0 junk"));CHECK(!command("prepare junk"));CHECK(!command("current 0 0.1junk"));
    CHECK(command("tune 0 0.01 0.02"));CHECK(fabsf(g_app.motor[0].velocity_pi.kp-0.01f)<1e-6f);
    CHECK(!command("tune 0 nan 0.01"));CHECK(!command("tune 0 0.01 -1"));
    CHECK(!command("tune 0 0.1 0.01"));CHECK(!command("tune 0 0.01 0.02 extra"));
    CHECK(!command("tune 0 0.01"));CHECK(!pwm[0]);
    CHECK(command("trace 0"));CHECK(!pwm[0]&&!pwm[1]);
    CHECK(!command("trace 0 128"));CHECK(!command("trace 2"));CHECK(!command("trace 0 0 extra"));
    uint16_t raw[4]={2048,2048,2048,2048};
    /* 正/反编码器方向均须完成一圈电角度的合成校准轨迹。 */
    for (int dir=-1;dir<=1;dir+=2) {
        CHECK(setup());sensors(0.5f);CHECK(command("cal 0"));
        tick(raw,true);
        CHECK(g_app.motor[0].state==CAL_ALIGN);
        CHECK(fabsf(g_app.motor[0].voltage_d)<0.001f); /* 对齐第一步不能直接施加满电流阶跃。 */
        for (unsigned i=0;i<CAL_ALIGN_TICKS+CAL_SWEEP_TICKS+CAL_SETTLE_TICKS+1000u&&g_app.motor[0].state!=IDLE;i++) {
            g_encoder[0].angle=0.5f+dir*g_app.motor[0].calibration_theta/MOTOR_POLE_PAIRS;
            tick(raw,true);
        }
        CHECK(!g_app.fault);CHECK(g_app.motor[0].calibrated);CHECK(g_app.motor[0].direction==dir);CHECK(!pwm[0]);
        CHECK(fabsf(wrap_pi(g_app.motor[0].electrical_offset-dir*3.5f))<0.01f);
        CHECK(command("position 0 0.1"));
        for (unsigned i=0;i<100;i++) tick(raw,true);
        CHECK(g_app.motor[0].iq_command>0);
        float previous_iq=g_app.motor[0].iq_command;
        CHECK(!command("tune 0 0.01 0.02"));CHECK(command("position 0 0.2"));
        CHECK(g_app.motor[0].iq_command==previous_iq&&pwm[0]);
        CHECK(command("idle 0"));CHECK(!pwm[0]);
        g_encoder[0].valid=false;tick(raw,true);CHECK(!g_app.motor[0].calibrated);
    }
    CHECK(setup());CHECK(command("cal 0"));
    for (unsigned i=0;i<CAL_ALIGN_TICKS+CAL_SWEEP_TICKS+1000u&&!g_app.fault;i++) tick(raw,true);
    CHECK(g_app.fault&FAULT_CALIBRATION);CHECK(!gate&&!pwm[0]&&!pwm[1]);

    CHECK(setup());g_app.motor[0].calibrated=true;CHECK(command("current 0 0.1"));
    raw[0]=2088;tick(raw,true);raw[0]=2048;
    CHECK(g_app.fault&FAULT_CURRENT);CHECK(!gate&&!pwm[0]&&!pwm[1]);
    CHECK(setup());g_app.motor[0].calibrated=true;CHECK(command("velocity 0 1"));
    CHECK(!command("trace 0"));
    /* 转子速度突升时，下一次速度环必须立即给制动电流，不能被电流指令斜率卡住。 */
    CHECK(!command("tune 0 0.01 0.02"));
    g_app.motor[0].iq_command=0.08f;g_app.motor[0].velocity_pi.integral=0.02f;
    g_encoder[0].velocity=10.0f;
    for (unsigned i=0;i<20;i++) tick(raw,true);
    CHECK(g_app.motor[0].iq_command<0);CHECK(!g_app.fault);
    g_encoder[0].velocity=0;
    clock_us+=2001000;tick(raw,false);CHECK(g_app.fault&FAULT_TIMEOUT);CHECK(!gate);
    CHECK(command("trace 0 0"));CHECK(!command("trace 0")); /* 故障后可读记录，不能重新武装采集。 */
    CHECK(setup());g_app.motor[0].calibrated=true;CHECK(command("position 0 0.1"));
    g_encoder[0].valid=false;tick(raw,true);CHECK(g_app.fault&FAULT_ENCODER);
    CHECK(setup());g_app.motor[0].calibrated=true;CHECK(command("current 0 0.1"));
    g_encoder[0].timestamp_us=clock_us-ENCODER_STALE_US-1u;
    app_adc_isr(raw);CHECK(g_app.fault&FAULT_ENCODER);CHECK(!gate&&!pwm[0]&&!pwm[1]);
    CHECK(setup());g_app.motor[0].calibrated=true;CHECK(command("current 0 0.1"));
    g_encoder[0].angle=NAN;tick(raw,true);CHECK(g_app.fault&FAULT_NUMERIC);
    CHECK(setup());fake_bus=13;clock_us+=10000;tick(raw,true);CHECK(g_app.fault&FAULT_BUS);
    CHECK(setup());g_app.ntc[0]=3000;tick(raw,true);CHECK(g_app.fault&FAULT_TEMPERATURE);
    CHECK(setup());fault_pin=true;tick(raw,true);CHECK(g_app.fault&FAULT_DRIVER);
    CHECK(setup());
    for (unsigned a=0;a<2;a++) g_app.motor[a].calibrated=true;
    CHECK(command("current 0 0.1"));CHECK(command("velocity 1 -1"));CHECK(pwm[0]&&pwm[1]);
    raw[2]=2088;tick(raw,true);raw[2]=2048;CHECK(g_app.fault&FAULT_CURRENT);CHECK(!gate&&!pwm[0]&&!pwm[1]);
    CHECK(command("clear"));CHECK(!g_app.fault&&!g_app.prepared&&!gate);CHECK(!command("velocity 0 1"));
    /* 环形记录必须限长，停机后分页可读；读取记录本身不能重新使能功率。 */
    CHECK(setup());g_app.motor[0].calibrated=true;CHECK(command("trace 0"));CHECK(command("velocity 0 1"));
    for (unsigned i=0;i<12000;i++) tick(raw,true);
    CHECK(command("idle 0"));
    char trace_reply[768];
    CHECK(app_command("trace 0 0",trace_reply,sizeof(trace_reply)));
    CHECK(strstr(trace_reply,"count=512")&&strstr(trace_reply,"\r\nT "));
    CHECK(app_command("trace 0 127",trace_reply,sizeof(trace_reply)));CHECK(strstr(trace_reply,"\r\nT "));
    CHECK(!pwm[0]&&!pwm[1]);
    return 0;
}
