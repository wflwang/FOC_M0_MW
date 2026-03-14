/**
 * @file hw_hk32m070_qfn32.h
 * @brief Hardware Configuration for HK32M070 QFN32 Package
 *        纠编机控制器专用配置
 * 
 * @details HK32M070 QFN32 Pin Configuration:
 *   - 64MHz M0 Core
 *   - 32KB Flash, 8KB SRAM
 *   - 3x Timer1 (PWM + ADC trigger)
 *   - 12-bit ADC (2 units)
 *   - 1x UART, 1x SPI, 1x I2C
 */

#ifndef HW_HK32M070_QFN32_H
#define HW_HK32M070_QFN32_H

#ifdef __cplusplus
extern "C" {
#endif

#include "types.h"
#include "hw_names.h"

/*============================================================================*/
/* Hardware Features                                                          */
/*============================================================================*/

#define HW_HAS_HALL               1      // 有霍尔传感器
#define HW_HAS_TEMP_SENSOR        1      // 有温度传感器
#define HW_HAS_CAN               0      // 无CAN
#define HW_HAS_UART              1      // 有UART
#define HW_HAS_ADC_OPAMP         1      // 内置运放

/*============================================================================*/
/* MCU Core Configuration                                                     */
/*============================================================================*/

#define MCU_NAME                  "HK32M070"
#define MCU_CORE_CLOCK_HZ         64000000UL
#define MCU_PERIPH_CLOCK_HZ       64000000UL

#define FLASH_SIZE_KB             32
#define SRAM_SIZE_KB              8

#define MCU_HAS_FPU               0      // 无硬件浮点
#define MCU_HAS_MPU               0      // 无MPU

/*============================================================================*/
/* PWM Configuration (TIM1 - Advanced Timer)                                  */
/*============================================================================*/

#define HW_PWM_TIMER              TIM1
#define HW_PWM_FREQ_HZ            20000UL   // 20kHz PWM频率

// PWM引脚定义
#define HW_PWM_UH_PIN             PA8
#define HW_PWM_UL_PIN             PA7
#define HW_PWM_VH_PIN             PA9
#define HW_PWM_VL_PIN             PB0
#define HW_PWM_WH_PIN             PA10
#define HW_PWM_WL_PIN             PB1

// PWM死区时间 (ns)
#define HW_PWM_DEADTIME_NS        500

// 最大占空比限制 (防止交叉导通)
#define HW_PWM_MAX_DUTY           9500     // 95%
#define HW_PWM_MIN_DUTY           200      // 2%

/*============================================================================*/
/* Current Sensing Configuration                                              */
/*============================================================================*/

#define HW_CURRENT_SHUNT_RES      0.001f   // 1mΩ采样电阻
#define HW_CURRENT_AMP_GAIN       10.0f    // 运放增益10x

// ADC通道定义
#define HW_CURRENT_U_ADC          ADC1_CH0   // PA0
#define HW_CURRENT_V_ADC          ADC1_CH1   // PA1
#define HW_CURRENT_W_ADC          ADC1_CH2   // PA2 (可选，二电阻时不用)

// 电流检测模式
#define HW_CURRENT_SAMPLES_NUM    2         // 每周期采样次数

// 电流范围
#define HW_CURRENT_MAX_AMPS       50.0f     // 最大电流50A
#define HW_CURRENT_OFFSET         2048      // ADC中点偏移 (12-bit ADC)

// ADC校准参数
#define HW_CURRENT_I_OFFSET_U     2048
#define HW_CURRENT_I_OFFSET_V     2048
#define HW_CURRENT_I_OFFSET_W     2048

#define HW_CURRENT_I_GAIN_U       1.0f
#define HW_CURRENT_I_GAIN_V       1.0f
#define HW_CURRENT_I_GAIN_W       1.0f

/*============================================================================*/
/* Voltage Sensing Configuration                                              */
/*============================================================================*/

#define HW_VOLTAGE_DIVIDER_UPPER  47000.0f  // 上分压电阻 47kΩ
#define HW_VOLTAGE_DIVIDER_LOWER  3000.0f   // 下分压电阻 3kΩ

#define HW_VOLTAGE_ADC            ADC1_CH4   // PA4
#define HW_VOLTAGE_GAIN           (HW_VOLTAGE_DIVIDER_LOWER / (HW_VOLTAGE_DIVIDER_UPPER + HW_VOLTAGE_DIVIDER_LOWER))

#define HW_VOLTAGE_MAX            60.0f     // 最大电压60V
#define HW_VOLTAGE_MIN            12.0f     // 最小电压12V

// 母线电压保护阈值
#define HW_VOLTAGE_OVER_LIMIT     55.0f     // 过压保护 55V
#define HW_VOLTAGE_UNDER_LIMIT    15.0f     // 欠压保护 15V

/*============================================================================*/
/* Hall Sensor Configuration                                                  */
/*============================================================================*/

#define HW_HALL_U_PIN             PB6       // 霍尔U
#define HW_HALL_V_PIN             PB7       // 霍尔V
#define HW_HALL_W_PIN             PB8       // 霍尔W

// 霍尔中断配置
#define HW_HALL_EXTI_LINE_U       EXTI_Line6
#define HW_HALL_EXTI_LINE_V       EXTI_Line7
#define HW_HALL_EXTI_LINE_W       EXTI_Line8

/*============================================================================*/
/* Temperature Sensor Configuration                                           */
/*============================================================================*/

#define HW_TEMP_ADC               ADC1_CH5   // PA5
#define HW_TEMP_BETA              3380.0f    // NTC Beta值
#define HW_TEMP_R25               10000.0f   // 25°C时电阻 10kΩ
#define HW_TEMP_R_PULLUP          10000.0f   // 上拉电阻 10kΩ

#define HW_TEMP_OVER_LIMIT        85.0f      // 过温保护 85°C
#define HW_TEMP_WARNING_LIMIT     70.0f      // 过温警告 70°C

/*============================================================================*/
/* LED and Button Configuration                                               */
/*============================================================================*/

#define HW_LED_PIN                PB9
#define HW_LED_ON_STATE           0          // 低电平点亮

#define HW_BUTTON_PIN             PB10
#define HW_BUTTON_ACTIVE_LOW      1          // 低电平有效

/*============================================================================*/
/* UART Configuration                                                         */
/*============================================================================*/

#define HW_UART_NUM               UART1
#define HW_UART_TX_PIN            PA9
#define HW_UART_RX_PIN            PA10
#define HW_UART_BAUDRATE          115200

/*============================================================================*/
/* SPI Configuration (for external encoder, optional)                         */
/*============================================================================*/

#define HW_SPI_NUM                SPI1
#define HW_SPI_SCK_PIN            PA5
#define HW_SPI_MISO_PIN           PA6
#define HW_SPI_MOSI_PIN           PA7
#define HW_SPI_CS_PIN             PA4

/*============================================================================*/
/* Gate Driver Enable Pin                                                    */
/*============================================================================*/

#define HW_GATE_DRIVER_EN_PIN     PB3
#define HW_GATE_DRIVER_EN_ACTIVE  1          // 高电平使能

/*============================================================================*/
/* Fault Input Pin                                                            */
/*============================================================================*/

#define HW_FAULT_PIN              PB4
#define HW_FAULT_ACTIVE_LOW       1          // 低电平有效

/*============================================================================*/
/* Timer Configuration for Speed Calculation                                  */
/*============================================================================*/

#define HW_SPEED_TIMER            TIM2
#define HW_SPEED_TIMER_FREQ_HZ    64000000UL

/*============================================================================*/
/* Pin Definition Macros                                                      */
/*============================================================================*/

// GPIO端口定义
#define PA(n)  ((GPIOA_BASE << 8) | (n))
#define PB(n)  ((GPIOB_BASE << 8) | (n))
#define PC(n)  ((GPIOC_BASE << 8) | (n))

/*============================================================================*/
/* Hardware Initialization Function                                           */
/*============================================================================*/

void hw_hk32m070_qfn32_init(void);

/*============================================================================*/
/* Hardware Specific Functions                                                */
/*============================================================================*/

/**
 * @brief Configure PWM timer
 */
void hw_pwm_timer_init(void);

/**
 * @brief Configure ADC for current/voltage sensing
 */
void hw_adc_init(void);

/**
 * @brief Configure GPIO pins
 */
void hw_gpio_init(void);

/**
 * @brief Configure hall sensor inputs
 */
void hw_hall_init(void);

/**
 * @brief Configure system clocks
 */
void hw_clock_init(void);

#ifdef __cplusplus
}
#endif

#endif /* HW_HK32M070_QFN32_H */
