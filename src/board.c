#include "board.h"
#include "app.h"
#include "config.h"
#include "tusb.h"

uint32_t SystemCoreClock = CORE_CLOCK_HZ;
static volatile uint32_t system_millis;
static volatile struct { uint32_t magic,inverse; } boot_request __attribute__((section(".noinit")));
#define DFU_MAGIC 0x434d4446u

/* 官方启动汇编在 .data/.bss 初始化前调用，不能依赖全局变量。 */
void SystemInit(void) {
    SCB->CPACR |= (3u << 20) | (3u << 22);
    SCB->VTOR = FLASH_BASE;
    __DSB(); __ISB();
    if ((RCC->CSR&RCC_CSR_SFTRSTF)&&boot_request.magic==DFU_MAGIC&&boot_request.inverse==~DFU_MAGIC) {
        /* 软件复位后立即进入ROM，无活动外设/中断，不从USB中断栈直接跳转。 */
        boot_request.magic=0;boot_request.inverse=0;
        RCC->APB2ENR|=RCC_APB2ENR_SYSCFGEN;
        SYSCFG->MEMRMP=1u;
        uint32_t stack=*(const uint32_t *)0x1fff0000u;
        uint32_t entry=*(const uint32_t *)0x1fff0004u;
        SCB->VTOR=0x1fff0000u;
        __DSB();__ISB();
        __asm volatile("msr msp, %0\n bx %1"::"r"(stack),"r"(entry):"memory");
        __builtin_unreachable();
    }
    boot_request.magic=0;boot_request.inverse=0;
    /* 兼容ROM DFU的GO命令返回应用：恢复HSI，再关闭可能仍开的PLL/HSE。
     * 不依赖ROM退出时的时钟状态，后续board_init重新建立168/48MHz。 */
    RCC->CR|=RCC_CR_HSION;
    while (!(RCC->CR&RCC_CR_HSIRDY)) {}
    RCC->CFGR=0;
    while (RCC->CFGR&RCC_CFGR_SWS) {}
    RCC->CR&=~(RCC_CR_PLLON|RCC_CR_HSEON|RCC_CR_CSSON);
    while (RCC->CR&RCC_CR_PLLRDY) {}
    RCC->CIR=0;
}
void SystemCoreClockUpdate(void) { SystemCoreClock = CORE_CLOCK_HZ; }

void gpio_mode(GPIO_TypeDef *g, unsigned p, unsigned mode, unsigned af, bool od) {
    g->MODER = (g->MODER & ~(3u << (2*p))) | mode << (2*p);
    g->OTYPER = (g->OTYPER & ~(1u << p)) | (od ? 1u << p : 0u);
    g->OSPEEDR = (g->OSPEEDR & ~(3u << (2*p))) | 2u << (2*p);
    g->PUPDR &= ~(3u << (2*p));
    g->AFR[p/8] = (g->AFR[p/8] & ~(15u << (4*(p%8)))) | af << (4*(p%8));
}

static void clock_init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    PWR->CR |= PWR_CR_VOS;
    RCC->CR |= RCC_CR_HSEON;
    uint32_t count = 2000000u;
    while (!(RCC->CR & RCC_CR_HSERDY) && --count) {}
    if (!count) board_panic();
    FLASH->ACR = FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN | FLASH_ACR_LATENCY_5WS;
    /* HSE=8MHz / M8 * N336 / P2 =168MHz，Q7 输出 USB 所需48MHz。 */
    RCC->PLLCFGR = 8u | (336u << 6) | RCC_PLLCFGR_PLLSRC_HSE | (7u << 24);
    RCC->CFGR = RCC_CFGR_PPRE1_DIV4 | RCC_CFGR_PPRE2_DIV2;
    RCC->CR |= RCC_CR_PLLON;
    count = 2000000u;
    while (!(RCC->CR & RCC_CR_PLLRDY) && --count) {}
    if (!count) board_panic();
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    count = 2000000u;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL && --count) {}
    if (!count) board_panic();
}

uint32_t micros(void) { return TIM2->CNT; }
uint32_t millis(void) { return system_millis; }
void SysTick_Handler(void) { ++system_millis; }
bool time_reached(uint32_t now, uint32_t deadline) { return (int32_t)(now-deadline) >= 0; }
void delay_us(uint32_t us) {
    uint32_t begin = micros();
    while ((uint32_t)(micros()-begin) < us) {}
}

void board_gate(bool on) { GPIOB->BSRR = on ? 1u<<12 : 1u<<28; }
void board_pwm_enable(unsigned a, bool on) {
    TIM_TypeDef *t = a ? TIM8 : TIM1;
    if (on) t->BDTR |= TIM_BDTR_MOE;
    else t->BDTR &= ~TIM_BDTR_MOE;
}
void board_shutdown(void) {
    board_gate(false);
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    TIM8->BDTR &= ~TIM_BDTR_MOE;
}
void board_pwm_set(unsigned a, PhaseDuty d) {
    TIM_TypeDef *t = a ? TIM8 : TIM1;
    t->CCR1 = (uint32_t)(limitf(d.a,0.10f,0.80f)*PWM_PERIOD_TICKS);
    t->CCR2 = (uint32_t)(limitf(d.b,0.10f,0.80f)*PWM_PERIOD_TICKS);
    t->CCR3 = (uint32_t)(limitf(d.c,0.10f,0.80f)*PWM_PERIOD_TICKS);
}
bool board_fault_pin(void) { return !(GPIOD->IDR & (1u<<2)); }

static void pwm_init(TIM_TypeDef *t) {
    t->PSC = 0;
    t->ARR = PWM_PERIOD_TICKS;
    t->CCR1 = t->CCR2 = t->CCR3 = PWM_PERIOD_TICKS/2;
    t->CCR4 = PWM_PERIOD_TICKS-168u;
    t->CR1 = TIM_CR1_ARPE | TIM_CR1_CMS_0; /* 中心对齐，一个完整周期50us。 */
    t->CCMR1 = (6u<<4) | TIM_CCMR1_OC1PE | (6u<<12) | TIM_CCMR1_OC2PE;
    /* OC4=PWM2，仅作为 ADC 触发基准；不连接到外部引脚。 */
    t->CCMR2 = (6u<<4) | TIM_CCMR2_OC3PE | (7u<<12) | TIM_CCMR2_OC4PE;
    t->CCER = TIM_CCER_CC1E|TIM_CCER_CC1NE|TIM_CCER_CC2E|TIM_CCER_CC2NE|TIM_CCER_CC3E|TIM_CCER_CC3NE;
    /* 84 ticks /168MHz =500ns死区，MOE=0时上下桥均输出低；不自动恢复MOE。 */
    t->BDTR = 84u | TIM_BDTR_OSSI | TIM_BDTR_OSSR;
    t->EGR = TIM_EGR_UG;
    t->SR = 0;
}

static void adc_init(void) {
    ADC->CCR = ADC_CCR_ADCPRE_0; /* APB2 /4=21MHz，避免超过ADC上限。 */
    ADC1->CR1 = ADC_CR1_SCAN | ADC_CR1_JEOCIE;
    /* PDF第1页U2A：PC0=M0_SO1，PC1=M0_SO2，PC3=M1_SO1，PC2=M1_SO2。
     * 按控制器的M0 B/C、M1 B/C顺序排列，不能按PC端口号猜相线。 */
    ADC1->JSQR = (3u<<20) | (10u<<0) | (11u<<5) | (13u<<10) | (12u<<15);
    ADC1->SMPR1 = (1u<<0) | (1u<<3) | (1u<<6) | (1u<<9); /* 15 cycles */
    ADC1->CR2 = ADC_CR2_ADON;
    ADC2->CR2 = ADC_CR2_ADON;
    ADC2->SMPR1 = 7u<<15; /* NTC PC5 / channel15，480 cycles。 */
    ADC2->SMPR2 = (7u<<18) | (7u<<12); /* Vbus ch6 / NTC PA4 ch4。 */
    NVIC_SetPriority(ADC_IRQn, 1);
    NVIC_EnableIRQ(ADC_IRQn);
}

void board_init(void) {
    g_app.boot_flags=RCC->CSR;
    RCC->CSR|=RCC_CSR_RMVF;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN|RCC_AHB1ENR_GPIOBEN|RCC_AHB1ENR_GPIOCEN|RCC_AHB1ENR_GPIODEN;
    __DSB();
    GPIOB->BSRR = (1u<<28)|(1u<<26)|(1u<<27); /* EN_GATE、制动上下管先拉低。 */
    gpio_mode(GPIOB,12,1,0,false);
    gpio_mode(GPIOB,10,1,0,false);
    gpio_mode(GPIOB,11,1,0,false);
    clock_init();
    SysTick_Config(CORE_CLOCK_HZ/1000u);
    NVIC_SetPriority(SysTick_IRQn,7);
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_SPI3EN | RCC_APB1ENR_I2C1EN;
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN|RCC_APB2ENR_TIM8EN|RCC_APB2ENR_ADC1EN|RCC_APB2ENR_ADC2EN|RCC_APB2ENR_SYSCFGEN;
    __DSB();
    TIM2->PSC = 83u; TIM2->ARR = 0xffffffffu; TIM2->EGR = TIM_EGR_UG; TIM2->CR1 = TIM_CR1_CEN;
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0; DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    DBGMCU->CR &= ~DBGMCU_CR_TRACE_IOEN; /* 不启用SWO；PB4的NJTRST复用由GPIO配置接管，PA13/14保留SWD。 */
    for (unsigned p=8;p<=10;p++) gpio_mode(GPIOA,p,2,1,false);
    for (unsigned p=13;p<=15;p++) gpio_mode(GPIOB,p,2,1,false);
    for (unsigned p=6;p<=8;p++) gpio_mode(GPIOC,p,2,3,false);
    gpio_mode(GPIOA,7,2,3,false);
    gpio_mode(GPIOB,0,2,3,false); gpio_mode(GPIOB,1,2,3,false);
    for (unsigned p=0;p<=3;p++) gpio_mode(GPIOC,p,3,0,false);
    gpio_mode(GPIOC,5,3,0,false); gpio_mode(GPIOA,4,3,0,false); gpio_mode(GPIOA,6,3,0,false);
    gpio_mode(GPIOD,2,0,0,false);
    GPIOD->PUPDR |= 1u<<4;
    SYSCFG->EXTICR[0] = (SYSCFG->EXTICR[0] & ~(15u<<8)) | 3u<<8;
    EXTI->FTSR |= 1u<<2; EXTI->IMR |= 1u<<2; EXTI->PR = 1u<<2;
    NVIC_SetPriority(EXTI2_IRQn,0); NVIC_EnableIRQ(EXTI2_IRQn);
    pwm_init(TIM1); pwm_init(TIM8);
    adc_init();
    /* TIM1 enable事件经ITR0同时启动TIM8，随后解除启动从模式。
     * TIM1 TRGO改为OC4REF，每个PWM周期仅上数侧的上升沿触发一次ADC。 */
    TIM1->CR2 = 1u<<4;
    TIM8->SMCR = 6u; /* TS=ITR0, SMS=trigger */
    TIM1->CR1 |= TIM_CR1_CEN;
    uint32_t sync_start=micros();
    while (!(TIM8->CR1 & TIM_CR1_CEN)) {
        if ((uint32_t)(micros()-sync_start)>10u) board_panic();
    }
    TIM8->SMCR = 0;
    TIM1->CR2 = 7u<<4;
    delay_us(10);
    ADC1->SR = 0;
    ADC1->CR2 |= ADC_CR2_JEXTSEL_0 | ADC_CR2_JEXTEN_0; /* JEXTSEL=1: TIM1_TRGO。 */
}

static uint16_t slow_adc(unsigned channel) {
    ADC2->SQR1 = 0; ADC2->SQR3 = channel;
    ADC2->SR = 0; ADC2->CR2 |= ADC_CR2_SWSTART;
    uint32_t begin = micros();
    while (!(ADC2->SR & ADC_SR_EOC)) {
        if ((uint32_t)(micros()-begin)>100u) { app_fault(FAULT_ADC); return 0; }
    }
    return (uint16_t)ADC2->DR;
}
void board_read_slow(float *bus, uint16_t ntc[2]) {
    *bus = slow_adc(6) * (ADC_REFERENCE_V*19.0f/4095.0f);
    ntc[0] = slow_adc(15); ntc[1] = slow_adc(4);
}

void ADC_IRQHandler(void) {
    if (ADC1->SR & ADC_SR_JEOC) {
        uint32_t start = DWT->CYCCNT;
        uint16_t raw[4] = {(uint16_t)ADC1->JDR1,(uint16_t)ADC1->JDR2,(uint16_t)ADC1->JDR3,(uint16_t)ADC1->JDR4};
        ADC1->SR = ~ADC_SR_JEOC;
        app_adc_isr(raw);
        uint32_t cycles = DWT->CYCCNT-start;
        if (cycles > g_app.isr_cycles_max) g_app.isr_cycles_max = cycles;
        if (cycles > CORE_CLOCK_HZ/40000u) app_fault(FAULT_OVERRUN); /* 超25us立即停两轴。 */
    }
}
void EXTI2_IRQHandler(void) {
    EXTI->PR = 1u<<2;
    if (g_app.prepared) app_fault(FAULT_DRIVER);
}
void board_watchdog_start(void) {
    IWDG->KR=0x5555; IWDG->PR=4; IWDG->RLR=100; IWDG->KR=0xaaaa; IWDG->KR=0xcccc;
}
void board_watchdog_feed(void) { IWDG->KR=0xaaaa; }
void board_usb_init(void) {
    gpio_mode(GPIOA,11,2,10,false); gpio_mode(GPIOA,12,2,10,false);
    RCC->AHB2ENR |= RCC_AHB2ENR_OTGFSEN;
    __DSB();
    /* 冷启动与ROM GO使用同一外设初态，等待PHY供电稳定，不依赖ROM留下的USB状态。 */
    RCC->AHB2RSTR|=RCC_AHB2RSTR_OTGFSRST;__DSB();
    RCC->AHB2RSTR&=~RCC_AHB2RSTR_OTGFSRST;__DSB();
    delay_us(1000);
    /* PA9已用于M0 PWM，板上无VBUS sense连接；自供电设备关闭硬件VBUS检测。 */
    USB_OTG_FS->GCCFG |= USB_OTG_GCCFG_NOVBUSSENS;
    USB_OTG_FS->GCCFG &= ~(USB_OTG_GCCFG_VBUSASEN|USB_OTG_GCCFG_VBUSBSEN);
    NVIC_SetPriority(OTG_FS_IRQn,6);
    tud_init(0);
    tud_disconnect();
    USB_OTG_FS->GCCFG |= USB_OTG_GCCFG_NOVBUSSENS;
    USB_OTG_FS->GCCFG &= ~(USB_OTG_GCCFG_VBUSASEN|USB_OTG_GCCFG_VBUSBSEN);
    delay_us(20000); /* 给主机明确的断开间隔；此时两轴功率始终关闭。 */
    tud_connect();
}
void OTG_FS_IRQHandler(void) { tud_int_handler(0); }
void board_panic(void) {
    __disable_irq();
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    __DSB();
    GPIOB->BSRR=1u<<28;
    gpio_mode(GPIOB,12,1,0,false);
    board_shutdown();
    for (;;) {}
}
void board_request_dfu(void) {
    __disable_irq();board_shutdown();
    boot_request.magic=DFU_MAGIC;boot_request.inverse=~DFU_MAGIC;
    __DSB();NVIC_SystemReset();
}
void HardFault_Handler(void) { board_panic(); }
void MemManage_Handler(void) { board_panic(); }
void BusFault_Handler(void) { board_panic(); }
void UsageFault_Handler(void) { board_panic(); }
void NMI_Handler(void) { board_panic(); }
