#ifndef CM_ENCODER_H
#define CM_ENCODER_H
#include <stdbool.h>
#include <stdint.h>
typedef struct {
    uint16_t raw;
    uint8_t status;
    bool valid;
    bool transport_ok;
    uint8_t diagnostic; /* bit0状态读失败，bit1角度读失败，bit2缺磁铁，bit3磁场范围，bit4跳变，bit5配置未验证。 */
    uint8_t lines;      /* bit0=SCL，bit1=SDA，事务结束后的引脚电平。 */
    float angle;        /* 未乘方向的连续机械角，rad。 */
    float velocity;     /* 未乘方向的机械速度，rad/s。 */
    uint32_t timestamp_us;
    uint32_t errors;
    uint8_t last_diagnostic;
    uint32_t last_error_us;
    uint16_t initial_config,config;
    bool configured;
} EncoderReading;
extern volatile EncoderReading g_encoder[2];
void encoder_init(void);
void encoder_poll(unsigned axis);
void encoder_scan(unsigned axis,char *reply,unsigned capacity);
#endif
