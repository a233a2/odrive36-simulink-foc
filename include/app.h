#ifndef CM_APP_H
#define CM_APP_H
#include <stdbool.h>
#include <stdint.h>
#include "foc.h"

typedef enum { IDLE=0, CAL_ALIGN, CAL_SWEEP, CAL_SETTLE, RUN_CURRENT, RUN_VELOCITY, RUN_POSITION } MotorState;
enum {
    FAULT_DRIVER=1u, FAULT_BUS=2u, FAULT_ENCODER=4u, FAULT_CURRENT=8u,
    FAULT_TIMEOUT=16u, FAULT_OVERRUN=32u, FAULT_OFFSET=64u, FAULT_CALIBRATION=128u,
    FAULT_TEMPERATURE=256u, FAULT_NUMERIC=512u, FAULT_ADC=1024u, FAULT_SPEED=2048u
};
typedef struct {
    MotorState state;
    bool calibrated;
    int direction;
    float electrical_offset, mechanical_zero;
    float position, velocity, id, iq, voltage_d, voltage_q;
    float target, velocity_ramp, iq_command;
    float calibration_start, calibration_theta;
    uint32_t state_ticks, outer_ticks;
    PiController d_pi, q_pi, velocity_pi;
} Motor;
typedef struct {
    uint32_t signature; /* 0x434D464F (CMFO)，便于调试器定位。 */
    uint32_t boot_flags; /* 启动时RCC_CSR复位原因，读取后清除硬件锁存位。 */
    uint32_t fault, isr_cycles_max, control_ticks, last_command_ms;
    bool prepared, offsets_ready;
    float bus_voltage;
    uint16_t ntc[2];
    float current_offsets[4], offset_noise[4];
    uint16_t raw_current[4];
    Motor motor[2];
} App;
extern volatile App g_app;
void app_init(void);
void app_poll(void);
void app_adc_isr(const uint16_t raw[4]);
void app_fault(uint32_t fault);
bool app_command(const char *line, char *reply, unsigned capacity);
void console_init(void);
void console_poll(void);
#endif
