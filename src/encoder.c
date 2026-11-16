#include "encoder.h"
#include "board.h"
#include "config.h"
#include <math.h>
#include <stdio.h>

volatile EncoderReading g_encoder[2];
static uint32_t last_status_us[2];
static uint8_t magnet_status[2];
static bool initialized[2];

/* M0: PB4=SCL，PB5=SDA。外部3.3k上拉；所有释放高电平都使用开漏。 */
static uint32_t soft_scl=1u<<ENCODER0_SCL_PIN,soft_sda=1u<<ENCODER0_SDA_PIN;
#define SCL0 soft_scl
#define SDA0 soft_sda
static void release0(uint32_t pins) { GPIOB->BSRR=pins; }
static void bit_delay(void) {
    /* DWT提供真正不少于1us的间隔；TIM2整数微秒计数无法保证1us短延时的下限。
     * ADC中断只会延长I2C时钟，不屏蔽电流环中断。 */
    uint32_t begin=DWT->CYCCNT;
    while ((uint32_t)(DWT->CYCCNT-begin)<CORE_CLOCK_HZ/1000000u) {}
}
static void low0(uint32_t pins) {
    GPIOB->BSRR=pins<<16;
    if (pins&SCL0) {
        uint32_t begin=DWT->CYCCNT;
        while ((uint32_t)(DWT->CYCCNT-begin)<CORE_CLOCK_HZ/4000000u) {} /* 数据保持至少0.25us。 */
    }
}
static bool scl0_high(void) {
    release0(SCL0);
    uint32_t t=micros();
    while (!(GPIOB->IDR&SCL0)) if ((uint32_t)(micros()-t)>50u) return false;
    bit_delay();
    return true;
}
static void stop0(void) {
    low0(SDA0); bit_delay();
    (void)scl0_high(); release0(SDA0); bit_delay();
}
static bool start0(void) {
    release0(SDA0);
    if (!scl0_high() || !(GPIOB->IDR&SDA0)) return false;
    low0(SDA0); bit_delay(); low0(SCL0);
    return true;
}
static bool write0(uint8_t byte) {
    for (unsigned bit=0;bit<8;bit++) {
        if (byte&0x80) release0(SDA0); else low0(SDA0);
        bit_delay();
        if (!scl0_high()) return false;
        low0(SCL0); byte<<=1;
    }
    release0(SDA0); bit_delay();
    if (!scl0_high()) return false;
    bool ack=!(GPIOB->IDR&SDA0); low0(SCL0);
    return ack;
}
static bool read0(uint8_t *byte,bool ack) {
    *byte=0; release0(SDA0);
    for (unsigned bit=0;bit<8;bit++) {
        bit_delay();
        if (!scl0_high()) return false;
        *byte=(uint8_t)((*byte<<1)|((GPIOB->IDR&SDA0)?1u:0u));
        low0(SCL0);
    }
    if (ack) low0(SDA0); else release0(SDA0);
    bit_delay();
    if (!scl0_high()) return false;
    low0(SCL0); release0(SDA0);
    return true;
}
static bool read_register0(uint8_t reg,uint8_t *bytes,unsigned count) {
    bool ok=start0()&&write0(0x6c)&&write0(reg)&&start0()&&write0(0x6d);
    for (unsigned i=0;ok&&i<count;i++) ok=read0(&bytes[i],i+1<count);
    stop0();
    return ok;
}

/* M1: PB6/PB7硬件I2C1，400kHz。STM32F4的2字节接收必须遵守POS/ACK顺序。 */
static void i2c1_config(void) {
    gpio_mode(GPIOB,6,2,4,true); gpio_mode(GPIOB,7,2,4,true);
    GPIOB->PUPDR|=(1u<<12)|(1u<<14); /* 弱上拉辅助诊断，不能代替合适的外部上拉。 */
    I2C1->CR1=I2C_CR1_SWRST; I2C1->CR1=0;
    I2C1->CR2=42u;
    I2C1->CCR=I2C_CCR_FS|35u; /* 快速模式DUTY=0，42MHz/(3*35)=400kHz。 */
    I2C1->TRISE=13u; /* 快速模式300ns上升时间预算：42MHz*300ns+1。 */
    I2C1->CR1=I2C_CR1_PE|I2C_CR1_ACK;
}
static bool wait1(uint32_t flag,uint32_t begin) {
    while (!(I2C1->SR1&flag)) {
        if (I2C1->SR1&(I2C_SR1_AF|I2C_SR1_BERR|I2C_SR1_ARLO|I2C_SR1_OVR)) return false;
        if ((uint32_t)(micros()-begin)>I2C_TRANSACTION_TIMEOUT_US) return false;
    }
    return true;
}
static void clear_addr1(void) { volatile uint32_t x=I2C1->SR1; x=I2C1->SR2; (void)x; }
static bool read_register1(uint8_t reg,uint8_t *bytes,unsigned count) {
    uint32_t begin=micros();
    while (I2C1->SR2&I2C_SR2_BUSY) if ((uint32_t)(micros()-begin)>I2C_TRANSACTION_TIMEOUT_US) return false;
    I2C1->CR1=(I2C1->CR1&~I2C_CR1_POS)|I2C_CR1_ACK|I2C_CR1_START;
    if (!wait1(I2C_SR1_SB,begin)) goto fail;
    I2C1->DR=0x6c;
    if (!wait1(I2C_SR1_ADDR,begin)) goto fail;
    clear_addr1();
    if (!wait1(I2C_SR1_TXE,begin)) goto fail;
    I2C1->DR=reg;
    if (!wait1(I2C_SR1_BTF,begin)) goto fail;
    I2C1->CR1|=I2C_CR1_START;
    if (!wait1(I2C_SR1_SB,begin)) goto fail;
    I2C1->DR=0x6d;
    if (!wait1(I2C_SR1_ADDR,begin)) goto fail;
    if (count==2) {
        uint32_t primask=__get_PRIMASK(); __disable_irq();
        I2C1->CR1|=I2C_CR1_POS; I2C1->CR1&=~I2C_CR1_ACK;
        clear_addr1(); __set_PRIMASK(primask);
        if (!wait1(I2C_SR1_BTF,begin)) goto fail;
        primask=__get_PRIMASK(); __disable_irq();
        I2C1->CR1|=I2C_CR1_STOP;
        bytes[0]=(uint8_t)I2C1->DR; bytes[1]=(uint8_t)I2C1->DR;
        __set_PRIMASK(primask);
    } else {
        uint32_t primask=__get_PRIMASK(); __disable_irq();
        I2C1->CR1&=~I2C_CR1_ACK; clear_addr1(); I2C1->CR1|=I2C_CR1_STOP;
        __set_PRIMASK(primask);
        if (!wait1(I2C_SR1_RXNE,begin)) goto fail;
        bytes[0]=(uint8_t)I2C1->DR;
    }
    I2C1->CR1=(I2C1->CR1&~I2C_CR1_POS)|I2C_CR1_ACK;
    return true;
fail:
    I2C1->CR1|=I2C_CR1_STOP;
    return false;
}

/* SDA卡死时，先释放开漏引脚并给9个时钟，然后STOP。电机由陈旧/无效反馈保护停机。 */
static void recover(unsigned axis) {
    unsigned scl=axis?ENCODER1_SCL_PIN:ENCODER0_SCL_PIN,sda=axis?ENCODER1_SDA_PIN:ENCODER0_SDA_PIN;
    if (axis) I2C1->CR1=0;
    GPIOB->BSRR=(1u<<scl)|(1u<<sda);
    gpio_mode(GPIOB,scl,1,0,true); gpio_mode(GPIOB,sda,1,0,true);
    GPIOB->PUPDR|=(1u<<(2*scl))|(1u<<(2*sda));
    for (unsigned i=0;i<9;i++) {
        GPIOB->BSRR=1u<<(scl+16); delay_us(3);
        GPIOB->BSRR=1u<<scl; delay_us(3);
    }
    GPIOB->BSRR=1u<<(sda+16); delay_us(3);
    GPIOB->BSRR=1u<<scl; delay_us(3); GPIOB->BSRR=1u<<sda;
    if (axis) i2c1_config();
}
static bool fast_config(unsigned axis) {
    /* 仅启动时、功率关闭时调用。两组临时用软件I2C读改写CONF并回读。
     * CONF[9:8]=11(2x)，PM[1:0]=00(正常模式)，WD[13]=0(禁止空闲自动降采样)。
     * 保留其余位，不访问0xff BURN，不改ZPOS/MPOS/MANG，不执行任何OTP烧写。 */
    unsigned scl=axis?ENCODER1_SCL_PIN:ENCODER0_SCL_PIN,sda=axis?ENCODER1_SDA_PIN:ENCODER0_SDA_PIN;
    if (axis) I2C1->CR1=0;
    GPIOB->BSRR=(1u<<scl)|(1u<<sda);
    gpio_mode(GPIOB,scl,1,0,true);gpio_mode(GPIOB,sda,1,0,true);
    GPIOB->PUPDR|=(1u<<(2*scl))|(1u<<(2*sda));
    soft_scl=1u<<scl;soft_sda=1u<<sda;
    uint8_t bytes[2]={0,0};
    bool ok=read_register0(0x07,bytes,2);
    uint16_t initial=((uint16_t)bytes[0]<<8)|bytes[1];
    uint16_t requested=(initial&~0x2003u)|0x0300u;
    if (ok) {
        ok=start0()&&write0(0x6c)&&write0(0x07)&&write0((uint8_t)(requested>>8))&&write0((uint8_t)requested);
        stop0();
    }
    if (ok) ok=read_register0(0x07,bytes,2);
    uint16_t actual=((uint16_t)bytes[0]<<8)|bytes[1];
    g_encoder[axis].initial_config=initial;g_encoder[axis].config=actual;
    g_encoder[axis].configured=ok&&actual==requested;
    soft_scl=1u<<ENCODER0_SCL_PIN;soft_sda=1u<<ENCODER0_SDA_PIN;
    if (axis) i2c1_config();
    return g_encoder[axis].configured;
}
void encoder_init(void) {
    recover(0); recover(1);
    delay_us(20000); /* 等编码器供电启动稳定，再配置临时寄存器。 */
    (void)fast_config(0);(void)fast_config(1);
}

void encoder_scan(unsigned axis,char *reply,unsigned capacity) {
    /* 只在两路功率均关闭时调用。两组均临时用软件I2C扫描，不依赖硬件接收逻辑。 */
    unsigned scl=axis?ENCODER1_SCL_PIN:ENCODER0_SCL_PIN,sda=axis?ENCODER1_SDA_PIN:ENCODER0_SDA_PIN;
    if (axis) I2C1->CR1=0;
    GPIOB->BSRR=(1u<<scl)|(1u<<sda);
    gpio_mode(GPIOB,scl,1,0,true);gpio_mode(GPIOB,sda,1,0,true);
    GPIOB->PUPDR|=(1u<<(2*scl))|(1u<<(2*sda));
    soft_scl=1u<<scl;soft_sda=1u<<sda;
    unsigned used=(unsigned)snprintf(reply,capacity,"M%u scan:",axis),found=0;
    for (unsigned address=8;address<120;address++) {
        bool ack=start0()&&write0((uint8_t)(address<<1));
        stop0();
        if (ack&&used+8<capacity) { used+=(unsigned)snprintf(reply+used,capacity-used," 0x%02x",address);found++; }
    }
    if (used<capacity) snprintf(reply+used,capacity-used,"; found=%u lines SCL=%u SDA=%u\r\n",found,
        (GPIOB->IDR&(1u<<scl))?1u:0u,(GPIOB->IDR&(1u<<sda))?1u:0u);
    soft_scl=1u<<ENCODER0_SCL_PIN;soft_sda=1u<<ENCODER0_SDA_PIN;
    if (axis) i2c1_config();
}

void encoder_poll(unsigned axis) {
    bool (*read_reg)(uint8_t,uint8_t *,unsigned)=axis?read_register1:read_register0;
    uint8_t raw[2],status=magnet_status[axis];
    EncoderReading next=g_encoder[axis];
    uint32_t now=micros();
    bool status_ok=true;
    if (!last_status_us[axis] || (uint32_t)(now-last_status_us[axis])>=10000u) {
        status_ok=read_reg(0x0b,&status,1);
        if (status_ok) { magnet_status[axis]=status; last_status_us[axis]=now; }
    }
    bool raw_ok=read_reg(0x0c,raw,2); /* 即使磁场无效也读取，便于区分总线和磁铁故障。 */
    bool retried=!status_ok||!raw_ok;
    if (retried) {
        /* 单次有界重读：失败时尚未发布新样本，ADC ISR仍独立检查最后有效样本的4ms期限。
         * 重读也失败才发布valid=false；不重试磁场无效/角度跳变，也不重置角度展开。
         * 即使重读成功也计入错误总数，保留第一次失败原因供布线/时序诊断。 */
        next.errors++;next.last_diagnostic=(status_ok?0u:1u)|(raw_ok?0u:2u);next.last_error_us=micros();
        recover(axis);
        if (!status_ok) {
            status_ok=read_reg(0x0b,&status,1);
            if (status_ok) { magnet_status[axis]=status;last_status_us[axis]=micros(); }
        }
        raw_ok=read_reg(0x0c,raw,2);
    }
    bool ok=status_ok&&raw_ok&&((status&0x38u)==0x20u)&&g_encoder[axis].configured;
    next.transport_ok=status_ok&&raw_ok;
    next.diagnostic=(status_ok?0u:1u)|(raw_ok?0u:2u)|((status&0x20u)?0u:4u)|((status&0x18u)?8u:0u)|(next.configured?0u:32u);
    next.lines=(uint8_t)(((GPIOB->IDR>>(axis?ENCODER1_SCL_PIN:ENCODER0_SCL_PIN))&1u)|(((GPIOB->IDR>>(axis?ENCODER1_SDA_PIN:ENCODER0_SDA_PIN))&1u)<<1));
    if (ok) {
        uint32_t stamp=micros();
        uint16_t count=(uint16_t)((((unsigned)raw[0]<<8)|raw[1])&0xfffu);
        if (!initialized[axis]) {
            next.angle=count*(TWO_PI_F/4096.0f); next.velocity=0;
            initialized[axis]=true;
        } else {
            int32_t delta=encoder_delta(next.raw,count);
            float dt=(stamp-next.timestamp_us)*1e-6f;
            /* 硬跳变不能悄悄参与FOC。此项目低速运行，>64count/sample明显异常。 */
            if (delta>64 || delta< -64 || dt>0.020f || dt<=0.0f) { ok=false;next.diagnostic|=16u; }
            else {
                float speed=delta*(TWO_PI_F/4096.0f)/dt;
                float alpha=dt/(VELOCITY_FILTER_TAU_S+dt); /* 0.5ms低通，配合更快轮询减少制动延迟。 */
                next.angle+=delta*(TWO_PI_F/4096.0f);
                next.velocity+=alpha*(speed-next.velocity);
            }
        }
        if (ok) { next.raw=count; next.timestamp_us=stamp; }
    }
    next.valid=ok; next.status=status;
    if (raw_ok) next.raw=(uint16_t)((((unsigned)raw[0]<<8)|raw[1])&0xfffu);
    if (!ok) { if(!retried) next.errors++;next.last_diagnostic=next.diagnostic;next.last_error_us=micros();initialized[axis]=false; }
    uint32_t primask=__get_PRIMASK(); __disable_irq();
    g_encoder[axis]=next;
    __set_PRIMASK(primask);
    if (!next.transport_ok) recover(axis); /* 磁场/角度异常不需要反复复位一个已经正常的I2C总线。 */
}
