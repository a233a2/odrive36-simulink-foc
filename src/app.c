#include "app.h"
#include "board.h"
#include "config.h"
#include "encoder.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

volatile App g_app;
static uint32_t offset_samples, last_slow_us;
static float offset_mean[4], offset_m2[4];

/* 调试用1kHz单轴环形记录；不在中断内打印，默认关闭，故障后保留最后512ms。 */
typedef struct { uint32_t us;float position,velocity,iqref,id,iq,vd,vq,theta;uint32_t age_us; } TraceSample;
#define TRACE_SAMPLES 512u
static TraceSample trace_samples[TRACE_SAMPLES];
static unsigned trace_head,trace_count,trace_axis;
static bool trace_armed;

static void record_trace(unsigned axis,const Motor *m,float theta,uint32_t now,uint32_t stamp) {
    if (!trace_armed||axis!=trace_axis||g_app.control_ticks%20u) return;
    trace_samples[trace_head]=(TraceSample){now,m->position,m->velocity,m->iq_command,m->id,m->iq,m->voltage_d,m->voltage_q,theta,now-stamp};
    trace_head=(trace_head+1u)%TRACE_SAMPLES;
    if (trace_count<TRACE_SAMPLES) trace_count++;
}

static bool active(MotorState s) { return s!=IDLE; }
static void reset_pi(Motor *m) {
    m->d_pi.integral=m->q_pi.integral=m->velocity_pi.integral=0;
    m->iq_command=m->velocity_ramp=0;
}
void app_init(void) {
    memset((void *)&g_app,0,sizeof(g_app));
    foc_current_init();
    g_app.signature=0x434d464fu;
    for (unsigned i=0;i<2;i++) {
        Motor *m=(Motor *)&g_app.motor[i];
        m->direction=1;
        /* Kp=L*w、Ki=R*w；300Hz用于缩短制动延迟，R/L仍须实机辨识。 */
        m->d_pi.kp=m->q_pi.kp=MOTOR_PHASE_L*TWO_PI_F*CURRENT_BANDWIDTH_HZ;
        m->d_pi.ki=m->q_pi.ki=MOTOR_PHASE_R*TWO_PI_F*CURRENT_BANDWIDTH_HZ;
        m->velocity_pi.kp=VELOCITY_KP; m->velocity_pi.ki=VELOCITY_KI;
    }
    offset_samples=0;
    trace_armed=false;trace_head=trace_count=0;
    memset(offset_mean,0,sizeof(offset_mean)); memset(offset_m2,0,sizeof(offset_m2));
}
void app_fault(uint32_t fault) {
    /* 可从主循环、ADC ISR或更高优先级的nFAULT ISR调用；故障锁存直至clear。 */
    uint32_t p=__get_PRIMASK(); __disable_irq();
    board_shutdown();
    g_app.fault|=fault; g_app.prepared=false; g_app.offsets_ready=false;
    for (unsigned a=0;a<2;a++) { g_app.motor[a].state=IDLE; g_app.motor[a].calibrated=false; }
    __set_PRIMASK(p);
}

void app_poll(void) {
    uint32_t now=micros();
    if ((uint32_t)(now-last_slow_us)>=10000u) {
        float bus; uint16_t ntc[2];
        board_read_slow(&bus,ntc);
        uint32_t p=__get_PRIMASK(); __disable_irq();
        g_app.bus_voltage=bus; g_app.ntc[0]=ntc[0]; g_app.ntc[1]=ntc[1];
        last_slow_us=now;
        __set_PRIMASK(p);
    }
}

static bool offset_step(const uint16_t raw[4]) {
    ++offset_samples;
    for (unsigned i=0;i<4;i++) {
        float delta=raw[i]-offset_mean[i];
        offset_mean[i]+=delta/offset_samples;
        offset_m2[i]+=delta*(raw[i]-offset_mean[i]);
    }
    if (offset_samples<OFFSET_SAMPLE_COUNT) return false;
    for (unsigned i=0;i<4;i++) {
        float noise=sqrtf(offset_m2[i]/(OFFSET_SAMPLE_COUNT-1));
        g_app.current_offsets[i]=offset_mean[i]; g_app.offset_noise[i]=noise;
        if (offset_mean[i]<1400.0f || offset_mean[i]>2700.0f || noise>4.0f) {
            app_fault(FAULT_OFFSET); return false;
        }
    }
    g_app.offsets_ready=true;
    return true;
}

static bool calibrate_step(unsigned axis,Motor *m,EncoderReading enc) {
    ++m->state_ticks;
    if (m->state==CAL_ALIGN) {
        m->calibration_theta=0;
        if (m->state_ticks>=CAL_ALIGN_TICKS) {
            m->calibration_start=enc.angle; m->state=CAL_SWEEP; m->state_ticks=0;
        }
    } else if (m->state==CAL_SWEEP) {
        m->calibration_theta=TWO_PI_F*m->state_ticks/CAL_SWEEP_TICKS;
        if (m->state_ticks>=CAL_SWEEP_TICKS) {
            float travel=enc.angle-m->calibration_start;
            float expected=TWO_PI_F/MOTOR_POLE_PAIRS;
            if (fabsf(travel)<expected*0.65f || fabsf(travel)>expected*1.35f) {
                app_fault(FAULT_CALIBRATION); return false;
            }
            m->direction=travel>=0?1:-1;
            m->state=CAL_SETTLE; m->state_ticks=0;
        }
    } else if (m->state==CAL_SETTLE) {
        m->calibration_theta=TWO_PI_F;
        if (m->state_ticks>=CAL_SETTLE_TICKS) {
            m->electrical_offset=wrap_2pi(m->direction*MOTOR_POLE_PAIRS*enc.angle);
            m->mechanical_zero=enc.angle;
            m->calibrated=true; m->state=IDLE;
            reset_pi(m); board_pwm_enable(axis,false);
            return false;
        }
    }
    return true;
}

void app_adc_isr(const uint16_t raw[4]) {
    ++g_app.control_ticks;
    for (unsigned i=0;i<4;i++) g_app.raw_current[i]=raw[i];
    if (!g_app.prepared || g_app.fault) return;
    uint32_t now=micros();
    if (board_fault_pin()) { app_fault(FAULT_DRIVER); return; }
    if ((uint32_t)(now-last_slow_us)>50000u) { app_fault(FAULT_ADC); return; }
    if (!isfinite(g_app.bus_voltage)||g_app.bus_voltage<BUS_MIN_V||g_app.bus_voltage>BUS_MAX_V) {
        app_fault(FAULT_BUS); return;
    }
    for (unsigned a=0;a<2;a++) {
        if (g_app.ntc[a]<30u || g_app.ntc[a]>NTC_TRIP_ADC) { app_fault(FAULT_TEMPERATURE); return; }
    }
    if (!g_app.offsets_ready) { (void)offset_step(raw); return; }

    for (unsigned a=0;a<2;a++) {
        /* motor状态只由此ISR或短临界区写入，指针不能跨主循环调用缓存。 */
        Motor *m=(Motor *)&g_app.motor[a];
        EncoderReading enc=g_encoder[a];
        m->position=m->direction*(enc.angle-m->mechanical_zero);
        m->velocity=m->direction*enc.velocity;
        float ib=(raw[2*a]-g_app.current_offsets[2*a])*AMPS_PER_ADC_COUNT;
        float ic=(raw[2*a+1]-g_app.current_offsets[2*a+1])*AMPS_PER_ADC_COUNT;
        if (!active(m->state)) {
            m->id=m->iq=0;
            if (!enc.valid) m->calibrated=false; /* 角度展开在失效后重置，不保留旧多圈校准状态。 */
            continue;
        }
        if (!isfinite(enc.angle)||!isfinite(enc.velocity)||!isfinite(ib)||!isfinite(ic)) { app_fault(FAULT_NUMERIC); return; }
        if (!enc.valid||(uint32_t)(now-enc.timestamp_us)>ENCODER_STALE_US) { app_fault(FAULT_ENCODER); return; }
        if (fabsf(ib)>CURRENT_TRIP_A||fabsf(ic)>CURRENT_TRIP_A||fabsf(ib+ic)>CURRENT_TRIP_A) {
            app_fault(FAULT_CURRENT); return;
        }
        if (fabsf(m->velocity)>15.0f) { app_fault(FAULT_SPEED); return; }
        if ((uint32_t)(millis()-g_app.last_command_ms)>COMMAND_TIMEOUT_MS) { app_fault(FAULT_TIMEOUT); return; }
        bool calibration=(m->state>=CAL_ALIGN && m->state<=CAL_SETTLE);
        float theta;
        if (calibration) {
            if (!calibrate_step(a,m,enc)) continue;
            theta=m->calibration_theta;
        } else {
            /* 利用速度补偿I2C采样到当前PWM的延迟，反馈过期仍立即报错。 */
            float predicted=enc.angle+enc.velocity*((uint32_t)(now-enc.timestamp_us)*1e-6f);
            theta=wrap_2pi(m->direction*MOTOR_POLE_PAIRS*predicted-m->electrical_offset);
        }
        float id_target=calibration?ALIGN_CURRENT_A:0;
        if (calibration&&m->state==CAL_ALIGN) {
            /* 空载2208转子惯量小：用1s斜坡建立对齐磁场，避免给Id阶跃时快速摆动。 */
            id_target*=limitf((float)m->state_ticks/CAL_CURRENT_RAMP_TICKS,0.0f,1.0f);
        }
        if (!calibration && ++m->outer_ticks>=20u) {
            m->outer_ticks=0; /* 速度/位置外环1kHz，电流内环20kHz。 */
            float desired_iq;
            if (m->state==RUN_CURRENT) desired_iq=m->target;
            else {
                float requested=m->target;
                if (m->state==RUN_POSITION) requested=limitf(POSITION_KP*(m->target-m->position),-VELOCITY_LIMIT_RAD_S,VELOCITY_LIMIT_RAD_S);
                m->velocity_ramp=approach(m->velocity_ramp,requested,ACCEL_LIMIT_RAD_S2*0.001f);
                desired_iq=pi_step(&m->velocity_pi,m->velocity_ramp-m->velocity,0.001f,CURRENT_LIMIT_A);
            }
            /* 直接电流模式才限制Iq指令斜率。速度/位置模式已有速度目标加速度限制，
             * 速度PI必须能立即降低/反向转矩；再次限斜率会延迟小惯量转子的制动。 */
            m->iq_command=m->state==RUN_CURRENT?
                approach(m->iq_command,desired_iq,IQ_SLEW_A_PER_S*0.001f):desired_iq;
        }
        float iq_target=calibration?0:limitf(m->iq_command,-CURRENT_LIMIT_A,CURRENT_LIMIT_A);
        /* 传入同一轴的物理量和状态。只有数值检查通过才更新 PWM 预装载寄存器。 */
        FocCurrentInput input={.ib_a=ib,.ic_a=ic,.theta_e_rad=theta,
            .vbus_v=g_app.bus_voltage,.phase_voltage_limit_v=PHASE_VOLTAGE_LIMIT_V,
            .reference_a={id_target,iq_target}};
        FocCurrentOutput output;
        if (!foc_current_step(&input,(Dq){m->id,m->iq},&m->d_pi,&m->q_pi,&output)) {
            app_fault(FAULT_NUMERIC);
            return;
        }
        if (g_app.fault) return; /* nFAULT can preempt the current loop above. */
        m->id=output.feedback_a.d; m->iq=output.feedback_a.q;
        m->voltage_d=output.voltage_v.d; m->voltage_q=output.voltage_v.q;
        board_pwm_set(a,output.duty);
        record_trace(a,m,theta,now,enc.timestamp_us);
    }
}

/* USB命令只在主循环解析。目标值/状态更新使用短临界区；SPI和打印不关中断。 */
bool app_command(const char *line,char *reply,unsigned capacity) {
    char command[24]={0},axis_text[24]={0},value_text[32]={0},extra[32]={0},trailing[8]={0};
    unsigned axis=0; float value=0;
    int fields=sscanf(line,"%23s %23s %31s %31s %7s",command,axis_text,value_text,extra,trailing);
    if (fields<1) return false;
    bool global=!strcmp(command,"help")||!strcmp(command,"status")||!strcmp(command,"encoder")||
                !strcmp(command,"ping")||!strcmp(command,"prepare")||!strcmp(command,"clear")||!strcmp(command,"dfu");
    if (global&&fields!=1) { snprintf(reply,capacity,"ERR unexpected argument\r\n"); return false; }
    if (!strcmp(command,"help")) {
        snprintf(reply,capacity,"help | status | encoder | scan <0|1> | ping | prepare | cal <0|1> | current <axis> <A> | velocity <axis> <rad/s> | position <axis> <rad> | tune <axis> <speed_Kp> <speed_Ki> | trace <axis> [page] (IDLE only) | idle <axis> | stop | clear | dfu\r\n");
        return true;
    }
    if (!strcmp(command,"dfu")) { board_shutdown();board_request_dfu(); }
    if (!strcmp(command,"stop")||!strcmp(command,"clear")) {
        uint32_t p=__get_PRIMASK(); __disable_irq();
        board_shutdown();
        if (!strcmp(command,"clear")) { uint32_t boot=g_app.boot_flags;app_init();g_app.boot_flags=boot; }
        else { g_app.prepared=false; g_app.offsets_ready=false; for (unsigned a=0;a<2;a++) g_app.motor[a].state=IDLE; }
        __set_PRIMASK(p);
        snprintf(reply,capacity,"OK %s; gates OFF\r\n",command); return true;
    }
    if (!strcmp(command,"ping")) { g_app.last_command_ms=millis(); snprintf(reply,capacity,"pong\r\n"); return true; }
    if (!strcmp(command,"encoder")) {
        EncoderReading e0=g_encoder[0],e1=g_encoder[1];
        snprintf(reply,capacity,"M0 ok=%u i2c=%u diag=0x%02x lines=%u raw=%u angle=%.4f status=0x%02x err=%lu; M1 ok=%u i2c=%u diag=0x%02x lines=%u raw=%u angle=%.4f status=0x%02x err=%lu\r\nlast_error M0 diag=%02x time_us=%lu M1 diag=%02x time_us=%lu\r\nCONF M0 %04x->%04x verified=%u M1 %04x->%04x verified=%u\r\nGPIOB MODER=%08lx OTYPER=%04lx ODR=%04lx IDR=%04lx PUPDR=%08lx\r\n",
            e0.valid,e0.transport_ok,e0.diagnostic,e0.lines,e0.raw,(double)e0.angle,e0.status,(unsigned long)e0.errors,
            e1.valid,e1.transport_ok,e1.diagnostic,e1.lines,e1.raw,(double)e1.angle,e1.status,(unsigned long)e1.errors,
            e0.last_diagnostic,(unsigned long)e0.last_error_us,e1.last_diagnostic,(unsigned long)e1.last_error_us,
            e0.initial_config,e0.config,e0.configured,e1.initial_config,e1.config,e1.configured,
            (unsigned long)GPIOB->MODER,(unsigned long)GPIOB->OTYPER,(unsigned long)GPIOB->ODR,(unsigned long)GPIOB->IDR,(unsigned long)GPIOB->PUPDR);
        return true;
    }
    if (!strcmp(command,"status")) {
        /* 仅拷贝遥测字段，避免长时间关中断。各轴字段允许相差一个控制周期。 */
        snprintf(reply,capacity,"fw=" CM_FIRMWARE_VERSION " fault=0x%lx ready=%u offset=%u bus=%.3f ntc=%u,%u ISRmax=%lu cycles up_ms=%lu boot=%08lx\r\nM0 state=%u cal=%u pos=%.4f vel=%.3f id=%.3f iq=%.3f vd=%.3f vq=%.3f tick=%lu dir=%d target=%.3f iqref=%.3f kp=%.4f ki=%.4f\r\nM1 state=%u cal=%u pos=%.4f vel=%.3f id=%.3f iq=%.3f vd=%.3f vq=%.3f tick=%lu dir=%d target=%.3f iqref=%.3f kp=%.4f ki=%.4f\r\nADC=%u,%u,%u,%u offset=%.2f,%.2f,%.2f,%.2f noise=%.2f,%.2f,%.2f,%.2f counts\r\n",
            (unsigned long)g_app.fault,g_app.prepared,g_app.offsets_ready,(double)g_app.bus_voltage,g_app.ntc[0],g_app.ntc[1],(unsigned long)g_app.isr_cycles_max,(unsigned long)millis(),(unsigned long)g_app.boot_flags,
            g_app.motor[0].state,g_app.motor[0].calibrated,(double)g_app.motor[0].position,(double)g_app.motor[0].velocity,(double)g_app.motor[0].id,(double)g_app.motor[0].iq,
            (double)g_app.motor[0].voltage_d,(double)g_app.motor[0].voltage_q,(unsigned long)g_app.motor[0].state_ticks,g_app.motor[0].direction,
            (double)g_app.motor[0].target,(double)g_app.motor[0].iq_command,(double)g_app.motor[0].velocity_pi.kp,(double)g_app.motor[0].velocity_pi.ki,
            g_app.motor[1].state,g_app.motor[1].calibrated,(double)g_app.motor[1].position,(double)g_app.motor[1].velocity,(double)g_app.motor[1].id,(double)g_app.motor[1].iq,
            (double)g_app.motor[1].voltage_d,(double)g_app.motor[1].voltage_q,(unsigned long)g_app.motor[1].state_ticks,g_app.motor[1].direction,
            (double)g_app.motor[1].target,(double)g_app.motor[1].iq_command,(double)g_app.motor[1].velocity_pi.kp,(double)g_app.motor[1].velocity_pi.ki,
            g_app.raw_current[0],g_app.raw_current[1],g_app.raw_current[2],g_app.raw_current[3],
            (double)g_app.current_offsets[0],(double)g_app.current_offsets[1],(double)g_app.current_offsets[2],(double)g_app.current_offsets[3],
            (double)g_app.offset_noise[0],(double)g_app.offset_noise[1],(double)g_app.offset_noise[2],(double)g_app.offset_noise[3]);
        return true;
    }
    if (!strcmp(command,"trace")) {
        char *end_axis,*end_page;
        unsigned long a=strtoul(axis_text,&end_axis,10),page=strtoul(value_text,&end_page,10);
        if ((fields!=2&&fields!=3)||end_axis==axis_text||*end_axis||a>1||
            (fields==3&&(end_page==value_text||*end_page||page>=TRACE_SAMPLES/4))) {
            snprintf(reply,capacity,"ERR trace axis [page 0..127]\r\n");return false;
        }
        /* 采集只在控制ISR写入；两轴空闲才允许取出，不需要大段关中断拷贝。 */
        if (active(g_app.motor[0].state)||active(g_app.motor[1].state)) {
            snprintf(reply,capacity,"ERR idle both axes before trace command\r\n");return false;
        }
        if (fields==2) {
            if (g_app.fault) { snprintf(reply,capacity,"ERR clear fault before arming trace\r\n");return false; }
            trace_axis=(unsigned)a;trace_head=trace_count=0;trace_armed=true;
            snprintf(reply,capacity,"OK trace armed axis=%lu 1kHz last512 samples\r\n",a);return true;
        }
        if (a!=trace_axis) { snprintf(reply,capacity,"ERR trace belongs to axis=%u\r\n",trace_axis);return false; }
        unsigned used=(unsigned)snprintf(reply,capacity,"TRACE axis=%lu page=%lu count=%u columns=us,pos,vel,iqref,id,iq,vd,vq,theta,age_us\r\n",a,page,trace_count);
        for (unsigned i=(unsigned)page*4u;i<trace_count&&i<(unsigned)page*4u+4u;i++) {
            TraceSample s=trace_samples[(trace_head+TRACE_SAMPLES-trace_count+i)%TRACE_SAMPLES];
            if (used+140u>capacity) break;
            used+=(unsigned)snprintf(reply+used,capacity-used,"T %lu %.5f %.4f %.5f %.5f %.5f %.4f %.4f %.4f %lu\r\n",
                (unsigned long)s.us,(double)s.position,(double)s.velocity,(double)s.iqref,(double)s.id,(double)s.iq,(double)s.vd,(double)s.vq,(double)s.theta,(unsigned long)s.age_us);
        }
        return true;
    }
    if (g_app.fault) { snprintf(reply,capacity,"ERR fault=0x%lx; inspect then clear\r\n",(unsigned long)g_app.fault); return false; }
    if (!strcmp(command,"prepare")&&fields==1) {
        if (g_app.prepared) { snprintf(reply,capacity,"ERR already prepared; stop before reinitializing\r\n"); return false; }
        if (g_app.bus_voltage<BUS_MIN_V||g_app.bus_voltage>BUS_MAX_V) {
            snprintf(reply,capacity,"ERR bus %.3f V outside %.1f..%.1f V\r\n",(double)g_app.bus_voltage,(double)BUS_MIN_V,(double)BUS_MAX_V); return false;
        }
        for (unsigned a=0;a<2;a++) {
            if (g_app.ntc[a]<30u||g_app.ntc[a]>NTC_TRIP_ADC) { snprintf(reply,capacity,"ERR NTC disconnected/hot\r\n"); return false; }
        }
        if (!drv_prepare()) { app_fault(FAULT_DRIVER); snprintf(reply,capacity,"ERR driver SPI/config/fault\r\n"); return false; }
        uint32_t p=__get_PRIMASK(); __disable_irq();
        offset_samples=0; memset(offset_mean,0,sizeof(offset_mean)); memset(offset_m2,0,sizeof(offset_m2));
        g_app.offsets_ready=false; g_app.last_command_ms=millis(); g_app.prepared=true;
        __set_PRIMASK(p);
        snprintf(reply,capacity,"OK amplifiers enabled; PWM OFF; collecting offsets ~103ms\r\n"); return true;
    }
    if (fields<2) { snprintf(reply,capacity,"ERR unknown command/axis; help\r\n"); return false; }
    char *end;
    unsigned long parsed_axis=strtoul(axis_text,&end,10);
    if (*end||parsed_axis>1u) { snprintf(reply,capacity,"ERR axis must be 0 or 1\r\n"); return false; }
    axis=(unsigned)parsed_axis;
    if (!strcmp(command,"tune")) {
        /* 只改空闲轴速度PI，不使能功率；范围限于本次小电机调试，重启/clear恢复默认。 */
        char *end_kp,*end_ki;
        float kp=strtof(value_text,&end_kp),ki=strtof(extra,&end_ki);
        if (fields!=4||end_kp==value_text||end_ki==extra||*end_kp||*end_ki||
            !isfinite(kp)||!isfinite(ki)||kp<0||kp>0.05f||ki<0||ki>0.20f) {
            snprintf(reply,capacity,"ERR tune axis Kp[0..0.05] Ki[0..0.20]\r\n");return false;
        }
        uint32_t p=__get_PRIMASK();__disable_irq();
        Motor *m=(Motor *)&g_app.motor[axis];
        if (active(m->state)) { __set_PRIMASK(p);snprintf(reply,capacity,"ERR idle axis before tuning\r\n");return false; }
        m->velocity_pi.kp=kp;m->velocity_pi.ki=ki;m->velocity_pi.integral=0;
        __set_PRIMASK(p);
        snprintf(reply,capacity,"OK tune axis=%u kp=%.4f ki=%.4f\r\n",axis,(double)kp,(double)ki);return true;
    }
    if (!strcmp(command,"scan")&&fields==2) {
        if (g_app.prepared) { snprintf(reply,capacity,"ERR stop before I2C scan\r\n");return false; }
        encoder_scan(axis,reply,capacity);return true;
    }
    if (fields>=3) {
        value=strtof(value_text,&end);
        if (*end||!isfinite(value)) { snprintf(reply,capacity,"ERR invalid number\r\n"); return false; }
    }
    bool cal=!strcmp(command,"cal"),idle=!strcmp(command,"idle");
    MotorState desired=IDLE;
    if (!strcmp(command,"current")) desired=RUN_CURRENT;
    if (!strcmp(command,"velocity")) desired=RUN_VELOCITY;
    if (!strcmp(command,"position")) desired=RUN_POSITION;
    if ((!cal&&!idle&&desired==IDLE)||(cal&&fields!=2)||(idle&&fields!=2)||
        (desired!=IDLE&&(fields!=3||!isfinite(value)))) { snprintf(reply,capacity,"ERR syntax; help\r\n"); return false; }
    if (!g_app.prepared||!g_app.offsets_ready) { snprintf(reply,capacity,"ERR prepare and wait for offset=1\r\n"); return false; }
    if (!g_encoder[axis].valid||(uint32_t)(micros()-g_encoder[axis].timestamp_us)>ENCODER_STALE_US) {
        snprintf(reply,capacity,"ERR encoder invalid/stale\r\n"); return false;
    }
    if (cal&&(active(g_app.motor[0].state)||active(g_app.motor[1].state))) {
        snprintf(reply,capacity,"ERR idle both motors before calibration\r\n"); return false;
    }
    if (desired!=IDLE && !g_app.motor[axis].calibrated) { snprintf(reply,capacity,"ERR cal axis first\r\n"); return false; }
    float maximum=desired==RUN_CURRENT?CURRENT_LIMIT_A:desired==RUN_VELOCITY?VELOCITY_LIMIT_RAD_S:100.0f;
    if (desired!=IDLE&&fabsf(value)>maximum) { snprintf(reply,capacity,"ERR target exceeds %.3f\r\n",(double)maximum); return false; }
    uint32_t p=__get_PRIMASK(); __disable_irq();
    /* 之前的检查之后仍可能有中断故障，最后一次检查与使能必须原子完成。 */
    if (g_app.fault||!g_app.prepared) { __set_PRIMASK(p); snprintf(reply,capacity,"ERR fault during command\r\n"); return false; }
    Motor *m=(Motor *)&g_app.motor[axis];
    if (desired!=IDLE && m->state==desired) {
        /* 同一运行模式只更新目标，保留PI和斜坡，连续指令不会反复关PWM或重置积分。 */
        m->target=value;g_app.last_command_ms=millis();__set_PRIMASK(p);
        snprintf(reply,capacity,"OK target axis=%u value=%.4f\r\n",axis,(double)value);return true;
    }
    board_pwm_enable(axis,false); reset_pi(m); m->outer_ticks=0; m->state_ticks=0;
    m->target=value; m->state=cal?CAL_ALIGN:desired;
    if (cal) m->calibrated=false;
    g_app.last_command_ms=millis();
    board_pwm_set(axis,(PhaseDuty){0.5f,0.5f,0.5f});
    if (!idle) board_pwm_enable(axis,true);
    __set_PRIMASK(p);
    snprintf(reply,capacity,"OK %s axis=%u target=%.4f; ping required every <2s\r\n",command,axis,(double)value);
    return true;
}
