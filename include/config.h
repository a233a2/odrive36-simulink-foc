#ifndef CM_CONFIG_H
#define CM_CONFIG_H
#ifdef CM_USE_SIMULINK_FOC
#define CM_FIRMWARE_VERSION "0.2.1-simulink"
#else
#define CM_FIRMWARE_VERSION "0.1.3-sample13"
#endif

/* 用户给定参数。电阻/电感尚未辨识；所有限值是首次空载调试值，不是额定值。 */
#define MOTOR_POLE_PAIRS       7.0f
#define MOTOR_PHASE_R          8.0f
#define MOTOR_PHASE_L          0.00425f
#define CORE_CLOCK_HZ          168000000u
#define PWM_FREQUENCY_HZ       20000u
#define PWM_PERIOD_TICKS       4200u
#define CONTROL_DT             (1.0f / PWM_FREQUENCY_HZ)
#define ADC_REFERENCE_V        3.3f
#define SHUNT_RESISTANCE_OHM    0.0005f
#define DRV_CURRENT_GAIN        80.0f
#define AMPS_PER_ADC_COUNT      (ADC_REFERENCE_V / (4095.0f * SHUNT_RESISTANCE_OHM * DRV_CURRENT_GAIN))
#define CURRENT_LIMIT_A        0.20f
#define CURRENT_TRIP_A         0.60f
#define ALIGN_CURRENT_A        0.15f
#define CAL_ALIGN_TICKS        (PWM_FREQUENCY_HZ * 3u / 2u)
#define CAL_CURRENT_RAMP_TICKS  PWM_FREQUENCY_HZ
#define CAL_SWEEP_TICKS        (PWM_FREQUENCY_HZ * 2u)
#define CAL_SETTLE_TICKS       (PWM_FREQUENCY_HZ / 2u)
#define PHASE_VOLTAGE_LIMIT_V   2.0f
#define VELOCITY_LIMIT_RAD_S    3.0f
#define ACCEL_LIMIT_RAD_S2      3.0f
#define CURRENT_BANDWIDTH_HZ   300.0f
#define VELOCITY_KP            0.015f
#define VELOCITY_KI            0.005f
#define POSITION_KP            2.0f
#define IQ_SLEW_A_PER_S        0.5f
#define VELOCITY_FILTER_TAU_S  0.0005f
#define ENCODER_POLL_PERIOD_US 500u
#define BUS_MIN_V              8.0f
#define BUS_MAX_V              12.2f
#define ENCODER_STALE_US       4000u
#define I2C_TRANSACTION_TIMEOUT_US 1000u /* 包含被ADC中断抢占的时间，仍受4ms反馈陈旧保护。 */
#define COMMAND_TIMEOUT_MS     2000u
#define USB_BOOT_DFU_TIMEOUT_MS 8000u /* 联调版：启动后未枚举USB则退回ROM DFU；独立运行时可改0禁用。 */
#define OFFSET_SAMPLE_COUNT    2048u
#define NTC_TRIP_ADC           2900u
#define PI_F                   3.14159265358979323846f
#define TWO_PI_F               (2.0f * PI_F)

#endif
