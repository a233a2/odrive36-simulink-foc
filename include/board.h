#ifndef CM_BOARD_H
#define CM_BOARD_H
#include "stm32f4xx.h"
#include "foc.h"
#include <stdbool.h>
#include <stdint.h>

/* 用户PDF第1页U2A逐线复核：编码器A用作SCL、B用作SDA。 */
#define ENCODER0_SCL_PIN 4u /* PB4 / MCU56 / M0_ENC_A */
#define ENCODER0_SDA_PIN 5u /* PB5 / MCU57 / M0_ENC_B */
#define ENCODER1_SCL_PIN 6u /* PB6 / MCU58 / M1_ENC_A */
#define ENCODER1_SDA_PIN 7u /* PB7 / MCU59 / M1_ENC_B */

void board_init(void);
uint32_t micros(void);
uint32_t millis(void);
void delay_us(uint32_t us);
bool time_reached(uint32_t now, uint32_t deadline);
void gpio_mode(GPIO_TypeDef *gpio, unsigned pin, unsigned mode, unsigned af, bool od);
void board_gate(bool enabled);
void board_pwm_enable(unsigned axis, bool enabled);
void board_pwm_set(unsigned axis, PhaseDuty duty);
void board_shutdown(void);
bool board_fault_pin(void);
void board_read_slow(float *bus, uint16_t ntc[2]);
void board_watchdog_start(void);
void board_watchdog_feed(void);
void board_usb_init(void);
void board_panic(void) __attribute__((noreturn));
void board_request_dfu(void) __attribute__((noreturn));
bool drv_prepare(void);
bool drv_status(uint16_t status[2][2]);
#endif
