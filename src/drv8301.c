#include "board.h"

/* 独立实现TI DRV8301 SPI协议。16-bit，mode1(CPOL=0/CPHA=1)，约656kHz。
 * EN_GATE低时芯片SPI不可用，prepare必须先在MOE=0下拉高EN_GATE。 */
static bool transfer(unsigned axis, uint16_t tx, uint16_t *rx) {
    uint32_t mask = 1u<<(13u+axis), begin=micros();
    GPIOC->BSRR = mask<<16;
    SPI3->DR = tx;
    while (!(SPI3->SR & SPI_SR_RXNE)) {
        if ((uint32_t)(micros()-begin)>100u) { GPIOC->BSRR=mask; return false; }
    }
    *rx=(uint16_t)SPI3->DR;
    while (SPI3->SR & SPI_SR_BSY) {
        if ((uint32_t)(micros()-begin)>100u) { GPIOC->BSRR=mask; return false; }
    }
    GPIOC->BSRR=mask;
    delay_us(2); /* CS必须在两个帧之间回到高电平。 */
    return true;
}
static bool write_reg(unsigned a,unsigned reg,uint16_t value) {
    uint16_t discard;
    return transfer(a,(uint16_t)((reg<<11)|(value&0x7ff)),&discard);
}
static bool read_reg(unsigned a,unsigned reg,uint16_t *value) {
    uint16_t discard;
    /* 读命令后的下一个帧返回所请求寄存器。 */
    if (!transfer(a,(uint16_t)(0x8000u|(reg<<11)),&discard)) return false;
    if (!transfer(a,(uint16_t)(0x8000u|(reg<<11)),value)) return false;
    if (((*value>>11)&15u)!=reg) return false;
    *value &= 0x7ffu;
    return true;
}
bool drv_prepare(void) {
    GPIOC->BSRR=(1u<<13)|(1u<<14);
    gpio_mode(GPIOC,13,1,0,false); gpio_mode(GPIOC,14,1,0,false);
    gpio_mode(GPIOC,10,2,6,false); gpio_mode(GPIOC,11,2,6,false); gpio_mode(GPIOC,12,2,6,false);
    SPI3->CR1=SPI_CR1_MSTR|SPI_CR1_SSM|SPI_CR1_SSI|SPI_CR1_DFF|SPI_CR1_CPHA|(5u<<3);
    SPI3->CR1 |= SPI_CR1_SPE;
    board_shutdown(); delay_us(100);
    board_gate(true); delay_us(20000);
    /* GATE_CURRENT=0.25A，6PWM，OCP锁存关断，VDS阈值code6=0.123V。
     * VDS保护仅为功率级后备保护，不是0.2A精细限流。 */
    const uint16_t control1=(6u<<6)|(1u<<4)|2u;
    const uint16_t control2=3u<<2; /* 80V/V，DC_CAL两通道均关闭。 */
    for (unsigned a=0;a<2;a++) {
        uint16_t c1=0,c2=0;
        if (!write_reg(a,2,control1)||!write_reg(a,3,control2)||
            !read_reg(a,2,&c1)||!read_reg(a,3,&c2)||c1!=control1||c2!=control2) {
            board_shutdown(); return false;
        }
    }
    uint16_t status[2][2];
    if (!drv_status(status)||board_fault_pin()) { board_shutdown(); return false; }
    return true;
}
bool drv_status(uint16_t s[2][2]) {
    for (unsigned a=0;a<2;a++) {
        if (!read_reg(a,0,&s[a][0])||!read_reg(a,1,&s[a][1])) return false;
        if (s[a][0] || (s[a][1]&0x80u)) return false;
    }
    return true;
}
