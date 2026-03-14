/**
 * @file hw_hk32m070_qfn48.c
 * @brief Hardware Implementation for HK32M070 QFN48
 */

#include "hw.h"
#include "hk32m070.h"

/*============================================================================*/
/* Private Variables                                                          */
/*============================================================================*/

static volatile uint16_t adc_current_raw[3] = {0};
static volatile uint16_t adc_voltage_raw = 0;
static volatile uint16_t adc_temp_raw = 0;
static volatile uint8_t hall_state = 0;
static volatile uint32_t system_ticks = 0;

static int16_t current_offset[3] = {HW_CURRENT_I_OFFSET_U, 
                                     HW_CURRENT_I_OFFSET_V, 
                                     HW_CURRENT_I_OFFSET_W};

/*============================================================================*/
/* Hardware Initialization                                                    */
/*============================================================================*/

void hw_init(void) {
    __disable_irq();
    
    // 时钟初始化
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY));
    RCC->CFGR |= RCC_CFGR_PLLSRC_HSE | RCC_CFGR_PLLMUL8;
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));
    FLASH->ACR = FLASH_ACR_LATENCY_2;
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
    
    SysTick_Config(MCU_CORE_CLOCK_HZ / 1000);
    
    // GPIO时钟使能
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN;
    
    // LED引脚
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    
    // 门驱动使能
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    
    // 霍尔输入
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    // 编码器输入 (QFN48专用)
#ifdef HW_HAS_ENCODER
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
#endif
    
    // PWM定时器初始化
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    TIM1->ARR = (MCU_PERIPH_CLOCK_HZ / HW_PWM_FREQ_HZ) - 1;
    TIM1->PSC = 0;
    TIM1->BDTR = (20 << TIM_BDTR_DTG_Pos) | TIM_BDTR_MOE;
    TIM1->CCMR1 = (6 << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE |
                  (6 << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
    TIM1->CCMR2 = (6 << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE;
    TIM1->CCER = TIM_CCER_CC1E | TIM_CCER_CC1NE |
                 TIM_CCER_CC2E | TIM_CCER_CC2NE |
                 TIM_CCER_CC3E | TIM_CCER_CC3NE;
    TIM1->CR1 |= TIM_CR1_CEN;
    
    // ADC初始化
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    ADC1->CR |= ADC_CR_ADCAL;
    while (ADC1->CR & ADC_CR_ADCAL);
    ADC1->SMPR = ADC_SMPR_SMP;
    ADC1->CHSELR = ADC_CHSELR_CHSEL0 | ADC_CHSELR_CHSEL1 | 
                   ADC_CHSELR_CHSEL2 | ADC_CHSELR_CHSEL4 | ADC_CHSELR_CHSEL5;
    ADC1->CR |= ADC_CR_ADEN;
    while (!(ADC1->ISR & ADC_ISR_ADRDY));
    ADC1->CR |= ADC_CR_ADSTART;
    
#ifdef HW_HAS_CAN
    // CAN初始化
    RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;
    // ... CAN配置
#endif
    
    __enable_irq();
}

void hw_get_currents(q15_t *phase) {
    for (int i = 0; i < 3; i++) {
        int16_t raw = (int16_t)adc_current_raw[i] - current_offset[i];
        phase[i] = (q15_t)(raw * 16);
    }
}

q15_t hw_get_bus_voltage(void) {
    return (q15_t)((adc_voltage_raw * 526) >> 7);
}

q15_t hw_get_temperature(void) {
    return (q15_t)(2500 - (adc_temp_raw * 100) / 100);
}

uint8_t hw_get_hall_state(void) {
    uint8_t state = 0;
    if (GPIOB->IDR & GPIO_PIN_6) state |= 0x01;
    if (GPIOB->IDR & GPIO_PIN_7) state |= 0x02;
    if (GPIOB->IDR & GPIO_PIN_8) state |= 0x04;
    return state;
}

void hw_set_pwm(uint16_t *duty) {
    uint32_t period = TIM1->ARR + 1;
    TIM1->CCR1 = (duty[0] * period) >> 16;
    TIM1->CCR2 = (duty[1] * period) >> 16;
    TIM1->CCR3 = (duty[2] * period) >> 16;
}

void hw_pwm_enable(bool enable) {
    if (enable) {
        TIM1->BDTR |= TIM_BDTR_MOE;
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
    } else {
        TIM1->BDTR &= ~TIM_BDTR_MOE;
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
    }
}

uint16_t hw_get_timer_value(void) {
    return (uint16_t)TIM2->CNT;
}

void hw_set_led(uint8_t state) {
    if (state == 0) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
    } else if (state == 1) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_3);
    }
}

bool hw_button_pressed(void) {
    return (GPIOC->IDR & GPIO_PIN_4) == 0;
}

void hw_delay_us(uint32_t us) {
    uint32_t start = TIM2->CNT;
    while ((TIM2->CNT - start) < us);
}

void hw_delay_ms(uint32_t ms) {
    while (ms--) hw_delay_us(1000);
}

uint32_t hw_get_micros(void) {
    return TIM2->CNT;
}

uint32_t hw_get_millis(void) {
    return system_ticks;
}

void SysTick_Handler(void) {
    system_ticks++;
}
