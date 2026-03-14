/**
 * @file hw_hk32m070_qfn32.c
 * @brief Hardware Implementation for HK32M070 QFN32
 */

#include "hw.h"
#include "hk32m070.h"
#include <string.h>

/*============================================================================*/
/* Private Variables                                                          */
/*============================================================================*/

// ADC采样缓冲
static volatile uint16_t adc_current_raw[3] = {0};
static volatile uint16_t adc_voltage_raw = 0;
static volatile uint16_t adc_temp_raw = 0;

// 霍尔状态
static volatile uint8_t hall_state = 0;
static volatile uint32_t hall_last_time = 0;
static volatile uint32_t hall_period = 0;

// 时间基准
static volatile uint32_t system_ticks = 0;

// 电流校准值
static int16_t current_offset[3] = {HW_CURRENT_I_OFFSET_U, 
                                     HW_CURRENT_I_OFFSET_V, 
                                     HW_CURRENT_I_OFFSET_W};

/*============================================================================*/
/* Clock Initialization                                                       */
/*============================================================================*/

void hw_clock_init(void) {
    // 启用外部高速晶振 (8MHz)
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY));
    
    // 配置PLL: 8MHz * 8 = 64MHz
    RCC->CFGR &= ~RCC_CFGR_PLLSRC;
    RCC->CFGR |= RCC_CFGR_PLLSRC_HSE;    // HSE作为PLL源
    RCC->CFGR &= ~RCC_CFGR_PLLMUL;
    RCC->CFGR |= RCC_CFGR_PLLMUL8;       // 8倍频
    
    // 启用PLL
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));
    
    // 设置Flash等待周期
    FLASH->ACR = FLASH_ACR_LATENCY_2;
    
    // 切换系统时钟到PLL
    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
    
    // 设置AHB/APB分频
    RCC->CFGR &= ~RCC_CFGR_HPRE;          // AHB = 64MHz
    RCC->CFGR &= ~RCC_CFGR_PPRE1;
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV1;     // APB1 = 64MHz
    RCC->CFGR &= ~RCC_CFGR_PPRE2;
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;     // APB2 = 64MHz
    
    // 启用SysTick (1ms中断)
    SysTick_Config(MCU_CORE_CLOCK_HZ / 1000);
}

/*============================================================================*/
/* GPIO Initialization                                                        */
/*============================================================================*/

void hw_gpio_init(void) {
    // 启用GPIO时钟
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    
    // LED引脚 - 推挽输出
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    // 门驱动使能引脚
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    // 故障输入引脚 - 下拉输入
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    // 按钮输入
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    // 初始状态
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET); // 禁用门驱动
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);   // LED灭
}

/*============================================================================*/
/* PWM Timer Initialization                                                   */
/*============================================================================*/

void hw_pwm_timer_init(void) {
    // 启用TIM1时钟
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    
    // PWM引脚配置 - 复用推挽输出
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    
    // U相 PWM引脚
    GPIO_InitStruct.Pin = GPIO_PIN_8;  // UH
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = GPIO_PIN_7;  // UL (需要映射)
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    // V相 PWM引脚
    GPIO_InitStruct.Pin = GPIO_PIN_9;  // VH
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = GPIO_PIN_0;  // VL
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    // W相 PWM引脚
    GPIO_InitStruct.Pin = GPIO_PIN_10; // WH
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = GPIO_PIN_1;  // WL
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    // TIM1配置: 互补PWM, 20kHz
    uint32_t timer_freq = MCU_PERIPH_CLOCK_HZ;
    uint32_t pwm_freq = HW_PWM_FREQ_HZ;
    uint32_t period = timer_freq / pwm_freq;  // 3200
    
    TIM1->ARR = period - 1;                    // 自动重装载值
    TIM1->PSC = 0;                             // 无预分频
    
    // 死区时间配置 (约500ns @ 64MHz)
    // DTG = 32 (0x20) => 32 * 15.625ns = 500ns
    TIM1->BDTR = (32 << TIM_BDTR_DTG_Pos) | 
                 TIM_BDTR_MOE |                 // 主输出使能
                 TIM_BDTR_AOE;                  // 自动输出使能
    
    // PWM模式1配置 (向上计数时，CNT < CCR时输出有效)
    // 通道1 (U相)
    TIM1->CCMR1 |= (6 << TIM_CCMR1_OC1M_Pos) |  // PWM模式1
                   TIM_CCMR1_OC1PE;              // 预装载使能
    TIM1->CCMR1 |= (6 << TIM_CCMR1_OC2M_Pos) |  // PWM模式1
                   TIM_CCMR1_OC2PE;
    
    // 通道2 (V相)
    TIM1->CCMR2 |= (6 << TIM_CCMR2_OC3M_Pos) |  // PWM模式1
                   TIM_CCMR2_OC3PE;
    TIM1->CCMR2 |= (6 << TIM_CCMR2_OC4M_Pos) |
                   TIM_CCMR2_OC4PE;
    
    // 使能所有通道输出
    TIM1->CCER = TIM_CCER_CC1E | TIM_CCER_CC1NE |  // U相
                 TIM_CCER_CC2E | TIM_CCER_CC2NE |  // V相
                 TIM_CCER_CC3E | TIM_CCER_CC3NE;   // W相
    
    // 配置ADC触发事件 (在PWM中心点触发)
    TIM1->CCR4 = period / 2;  // CCR4用于ADC触发
    TIM1->CR2 |= TIM_CR2_MMS_2; // TRGO on Update
    
    // 初始占空比
    TIM1->CCR1 = period / 2;
    TIM1->CCR2 = period / 2;
    TIM1->CCR3 = period / 2;
    
    // 使能计数器
    TIM1->CR1 |= TIM_CR1_CEN;
}

/*============================================================================*/
/* ADC Initialization                                                         */
/*============================================================================*/

void hw_adc_init(void) {
    // 启用ADC时钟
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    
    // ADC引脚配置 - 模拟输入
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    
    // 电流采样引脚
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    // 电压和温度采样引脚
    GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    // ADC校准
    ADC1->CR &= ~ADC_CR_ADEN;
    ADC1->CR |= ADC_CR_ADCAL;
    while (ADC1->CR & ADC_CR_ADCAL);
    
    // ADC配置
    ADC1->CFGR1 = ADC_CFGR1_CONT |        // 连续转换模式
                  ADC_CFGR1_DMAEN;         // DMA使能
    
    // 采样时间配置 (需要较长采样时间以保证精度)
    ADC1->SMPR = ADC_SMPR_SMP_0 | ADC_SMPR_SMP_1 | ADC_SMPR_SMP_2; // 239.5 cycles
    
    // 通道配置序列 (使用扫描模式)
    // CH0(Iu) -> CH1(Iv) -> CH2(Iw) -> CH4(Vbus) -> CH5(Temp)
    ADC1->CHSELR = ADC_CHSELR_CHSEL0 |
                   ADC_CHSELR_CHSEL1 |
                   ADC_CHSELR_CHSEL2 |
                   ADC_CHSELR_CHSEL4 |
                   ADC_CHSELR_CHSEL5;
    
    // 使能ADC
    ADC1->CR |= ADC_CR_ADEN;
    while (!(ADC1->ISR & ADC_ISR_ADRDY));
    
    // 启动转换
    ADC1->CR |= ADC_CR_ADSTART;
    
    // DMA配置 (用于自动传输ADC结果)
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;
    DMA1_Channel1->CPAR = (uint32_t)&ADC1->DR;
    DMA1_Channel1->CMAR = (uint32_t)adc_current_raw;
    DMA1_Channel1->CNDTR = 5;  // 5个通道
    DMA1_Channel1->CCR = DMA_CCR_MINC |     // 内存地址递增
                         DMA_CCR_CIRC |     // 循环模式
                         DMA_CCR_PL_1 |     // 高优先级
                         DMA_CCR_EN;        // 使能
}

/*============================================================================*/
/* Hall Sensor Initialization                                                 */
/*============================================================================*/

void hw_hall_init(void) {
    // 霍尔引脚配置 - 输入带上拉
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    // 配置外部中断 (用于霍尔边沿检测)
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;
    
    // 连接GPIO到EXTI线
    SYSCFG->EXTICR[1] = SYSCFG_EXTICR2_EXTI6_PB |
                        SYSCFG_EXTICR2_EXTI7_PB |
                        SYSCFG_EXTICR2_EXTI8_PB;
    
    // 配置EXTI线
    EXTI->IMR |= EXTI_IMR_IM6 | EXTI_IMR_IM7 | EXTI_IMR_IM8;
    EXTI->EMR |= EXTI_EMR_EM6 | EXTI_EMR_EM7 | EXTI_EMR_EM8;
    EXTI->RTSR |= EXTI_RTSR_RT6 | EXTI_RTSR_RT7 | EXTI_RTSR_RT8;  // 上升沿
    EXTI->FTSR |= EXTI_FTSR_FT6 | EXTI_FTSR_FT7 | EXTI_FTSR_FT8;  // 下降沿
    
    // 使能NVIC中断
    NVIC_EnableIRQ(EXTI4_15_IRQn);
    NVIC_SetPriority(EXTI4_15_IRQn, 2);
    
    // 速度测量定时器初始化
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->ARR = 0xFFFFFFFF;
    TIM2->PSC = MCU_PERIPH_CLOCK_HZ / 1000000 - 1;  // 1MHz计数
    TIM2->CR1 |= TIM_CR1_CEN;
    
    // 读取初始霍尔状态
    hall_state = hw_get_hall_state();
}

/*============================================================================*/
/* Hardware Initialization                                                    */
/*============================================================================*/

void hw_init(void) {
    // 禁用全局中断
    __disable_irq();
    
    // 初始化时钟系统
    hw_clock_init();
    
    // 初始化GPIO
    hw_gpio_init();
    
    // 初始化ADC
    hw_adc_init();
    
    // 初始化PWM定时器
    hw_pwm_timer_init();
    
    // 初始化霍尔传感器
    hw_hall_init();
    
    // 启用全局中断
    __enable_irq();
}

/*============================================================================*/
/* Current Sensing Functions                                                  */
/*============================================================================*/

void hw_get_currents(q15_t *phase) {
    // 获取原始ADC值 (12-bit, 0-4095)
    // 中心点为2048 (偏置)
    
    for (int i = 0; i < 3; i++) {
        int16_t raw = (int16_t)adc_current_raw[i] - current_offset[i];
        // 转换为Q15格式 (-32768 to 32767)
        // 满量程对应最大电流
        // Q15 = raw * 32768 / 2048 = raw * 16 (简化)
        phase[i] = (q15_t)(raw * 16);
    }
}

q15_t hw_get_bus_voltage(void) {
    // ADC值转换为电压
    // V_bus = ADC_raw * VREF / 4095 / 分压比
    // 使用Q8.8格式: 电压 * 256
    
    uint16_t raw = adc_voltage_raw;
    // 计算实际电压 (简化计算，避免浮点)
    // 假设分压比为 3k/(47k+3k) = 1/16.67
    // Q8.8格式: V = raw * 256 * 16.67 / 4095 * 3.3
    // 简化为: V = raw * 5.26 (约等于)
    q15_t voltage_q8 = (q15_t)((raw * 526) >> 7);  // Q8.8格式
    
    return voltage_q8;
}

q15_t hw_get_temperature(void) {
    // NTC温度计算 (简化版，使用查表更精确)
    uint16_t raw = adc_temp_raw;
    
    // 简化计算: 使用Steinhart-Hart方程
    // 这里使用近似线性化
    // 温度范围 -20°C ~ 120°C 映射到 Q8.8格式
    
    q15_t temp_q8 = 0;
    if (raw > 0 && raw < 4095) {
        // 简化的温度计算
        temp_q8 = (q15_t)(2500 - (raw * 100) / 100);  // Q8.8格式
    }
    
    return temp_q8;
}

/*============================================================================*/
/* Hall Sensor Functions                                                      */
/*============================================================================*/

uint8_t hw_get_hall_state(void) {
    uint8_t state = 0;
    
    if (GPIOB->IDR & GPIO_PIN_6) state |= 0x01;  // U
    if (GPIOB->IDR & GPIO_PIN_7) state |= 0x02;  // V
    if (GPIOB->IDR & GPIO_PIN_8) state |= 0x04;  // W
    
    return state;
}

uint32_t hw_get_hall_period_us(void) {
    return hall_period;
}

/*============================================================================*/
/* PWM Output Functions                                                       */
/*============================================================================*/

void hw_set_pwm(uint16_t *duty) {
    uint32_t period = TIM1->ARR + 1;
    
    // 限制占空比范围
    for (int i = 0; i < 3; i++) {
        if (duty[i] > HW_PWM_MAX_DUTY) duty[i] = HW_PWM_MAX_DUTY;
        if (duty[i] < HW_PWM_MIN_DUTY) duty[i] = HW_PWM_MIN_DUTY;
    }
    
    // 设置CCR值 (占空比映射)
    TIM1->CCR1 = (duty[0] * period) >> 16;  // U相
    TIM1->CCR2 = (duty[1] * period) >> 16;  // V相
    TIM1->CCR3 = (duty[2] * period) >> 16;  // W相
}

void hw_pwm_enable(bool enable) {
    if (enable) {
        // 使能PWM输出
        TIM1->BDTR |= TIM_BDTR_MOE;
        // 使能门驱动
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
    } else {
        // 禁用PWM输出
        TIM1->BDTR &= ~TIM_BDTR_MOE;
        // 禁用门驱动
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
        // LED指示
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
    }
}

/*============================================================================*/
/* Timer Functions                                                            */
/*============================================================================*/

uint16_t hw_get_timer_value(void) {
    return (uint16_t)TIM2->CNT;
}

uint32_t hw_get_micros(void) {
    return TIM2->CNT;
}

uint32_t hw_get_millis(void) {
    return system_ticks;
}

/*============================================================================*/
/* LED and Button Functions                                                   */
/*============================================================================*/

void hw_set_led(uint8_t state) {
    if (state == 0) {
        // Off
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
    } else if (state == 1) {
        // On
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
    } else {
        // Toggle
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9);
    }
}

bool hw_button_pressed(void) {
    return (GPIOB->IDR & GPIO_PIN_10) == 0;  // 低电平有效
}

/*============================================================================*/
/* Delay Functions                                                            */
/*============================================================================*/

void hw_delay_us(uint32_t us) {
    uint32_t start = TIM2->CNT;
    while ((TIM2->CNT - start) < us);
}

void hw_delay_ms(uint32_t ms) {
    while (ms--) {
        hw_delay_us(1000);
    }
}

/*============================================================================*/
/* Interrupt Handlers                                                         */
/*============================================================================*/

void SysTick_Handler(void) {
    system_ticks++;
}

void EXTI4_15_IRQHandler(void) {
    uint32_t current_time = TIM2->CNT;
    
    // 检测霍尔边沿
    if (EXTI->PR & (EXTI_PR_PR6 | EXTI_PR_PR7 | EXTI_PR_PR8)) {
        // 清除中断标志
        EXTI->PR = EXTI_PR_PR6 | EXTI_PR_PR7 | EXTI_PR_PR8;
        
        // 更新霍尔状态
        uint8_t new_state = hw_get_hall_state();
        
        // 计算霍尔周期
        hall_period = current_time - hall_last_time;
        hall_last_time = current_time;
        
        hall_state = new_state;
    }
}

/*============================================================================*/
/* Fault Detection                                                            */
/*============================================================================*/

bool hw_check_fault(void) {
    // 检查故障引脚状态
    if (HW_FAULT_ACTIVE_LOW) {
        return (GPIOB->IDR & GPIO_PIN_4) == 0;
    } else {
        return (GPIOB->IDR & GPIO_PIN_4) != 0;
    }
}
