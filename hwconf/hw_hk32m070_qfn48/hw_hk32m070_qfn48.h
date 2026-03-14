/**
 * @file hw_hk32m070_qfn48.h
 * @brief Hardware Configuration for HK32M070 QFN48 Package
 *        扩展版控制器配置 (更多GPIO, 支持外置编码器)
 */

#ifndef HW_HK32M070_QFN48_H
#define HW_HK32M070_QFN48_H

#ifdef __cplusplus
extern "C" {
#endif

#include "types.h"
#include "hw_names.h"

/*============================================================================*/
/* Hardware Features                                                          */
/*============================================================================*/

#define HW_HAS_HALL               1      // 有霍尔传感器
#define HW_HAS_ENCODER            1      // 有增量编码器接口
#define HW_HAS_TEMP_SENSOR        1      // 有温度传感器
#define HW_HAS_CAN               1      // 有CAN总线
#define HW_HAS_UART              1      // 有UART
#define HW_HAS_ADC_OPAMP          1      // 内置运放

/*============================================================================*/
/* MCU Core Configuration                                                     */
/*============================================================================*/

#define MCU_NAME                  "HK32M070"
#define MCU_CORE_CLOCK_HZ         64000000UL
#define MCU_PERIPH_CLOCK_HZ       64000000UL

#define FLASH_SIZE_KB             32
#define SRAM_SIZE_KB              8

#define MCU_HAS_FPU               0
#define MCU_HAS_MPU               0

/*============================================================================*/
/* PWM Configuration (TIM1 - Advanced Timer)                                  */
/*============================================================================*/

#define HW_PWM_TIMER              TIM1
#define HW_PWM_FREQ_HZ            20000UL   // 20kHz PWM频率

// PWM引脚定义 (QFN48有更多引脚选择)
#define HW_PWM_UH_PIN             PA8
#define HW_PWM_UL_PIN             PA7
#define HW_PWM_VH_PIN             PA9
#define HW_PWM_VL_PIN             PB0
#define HW_PWM_WH_PIN             PA10
#define HW_PWM_WL_PIN             PB1

#define HW_PWM_DEADTIME_NS        300
#define HW_PWM_MAX_DUTY           9600
#define HW_PWM_MIN_DUTY           200

/*============================================================================*/
/* Current Sensing Configuration                                              */
/*============================================================================*/

#define HW_CURRENT_SHUNT_RES      0.0005f   // 0.5mΩ采样电阻
#define HW_CURRENT_AMP_GAIN       20.0f     // 运放增益20x

#define HW_CURRENT_U_ADC          ADC1_CH0
#define HW_CURRENT_V_ADC          ADC1_CH1
#define HW_CURRENT_W_ADC          ADC1_CH2

#define HW_CURRENT_SAMPLES_NUM    2
#define HW_CURRENT_MAX_AMPS       80.0f     // 最大电流80A

#define HW_CURRENT_I_OFFSET_U     2048
#define HW_CURRENT_I_OFFSET_V     2048
#define HW_CURRENT_I_OFFSET_W     2048

/*============================================================================*/
/* Voltage Sensing Configuration                                              */
/*============================================================================*/

#define HW_VOLTAGE_DIVIDER_UPPER  68000.0f  // 68kΩ
#define HW_VOLTAGE_DIVIDER_LOWER  2200.0f   // 2.2kΩ

#define HW_VOLTAGE_ADC            ADC1_CH4
#define HW_VOLTAGE_GAIN           (HW_VOLTAGE_DIVIDER_LOWER / (HW_VOLTAGE_DIVIDER_UPPER + HW_VOLTAGE_DIVIDER_LOWER))

#define HW_VOLTAGE_MAX            72.0f     // 最大电压72V
#define HW_VOLTAGE_MIN            12.0f

#define HW_VOLTAGE_OVER_LIMIT     65.0f
#define HW_VOLTAGE_UNDER_LIMIT    15.0f

/*============================================================================*/
/* Hall Sensor Configuration                                                  */
/*============================================================================*/

#define HW_HALL_U_PIN             PB6
#define HW_HALL_V_PIN             PB7
#define HW_HALL_W_PIN             PB8

#define HW_HALL_EXTI_LINE_U       EXTI_Line6
#define HW_HALL_EXTI_LINE_V       EXTI_Line7
#define HW_HALL_EXTI_LINE_W       EXTI_Line8

/*============================================================================*/
/* Encoder Configuration (QFN48专用)                                          */
/*============================================================================*/

#define HW_ENCODER_A_PIN          PC0
#define HW_ENCODER_B_PIN          PC1
#define HW_ENCODER_Z_PIN          PC2

#define HW_ENCODER_TIMER          TIM3
#define HW_ENCODER_PPR            1024      // 每转脉冲数

/*============================================================================*/
/* CAN Configuration (QFN48专用)                                              */
/*============================================================================*/

#define HW_CAN_MODULE             CAN1
#define HW_CAN_TX_PIN             PB9
#define HW_CAN_RX_PIN             PB10
#define HW_CAN_BAUDRATE           500000    // 500kbps

/*============================================================================*/
/* Temperature Sensor Configuration                                           */
/*============================================================================*/

#define HW_TEMP_ADC               ADC1_CH5
#define HW_TEMP_BETA              3380.0f
#define HW_TEMP_R25               10000.0f
#define HW_TEMP_R_PULLUP          10000.0f

#define HW_TEMP_OVER_LIMIT        85.0f
#define HW_TEMP_WARNING_LIMIT     70.0f

/*============================================================================*/
/* LED and Button Configuration                                               */
/*============================================================================*/

#define HW_LED_PIN                PC3
#define HW_LED_ON_STATE           0

#define HW_BUTTON_PIN             PC4
#define HW_BUTTON_ACTIVE_LOW      1

/*============================================================================*/
/* UART Configuration                                                         */
/*============================================================================*/

#define HW_UART_NUM               UART1
#define HW_UART_TX_PIN            PA9
#define HW_UART_RX_PIN            PA10
#define HW_UART_BAUDRATE          115200

/*============================================================================*/
/* Gate Driver Enable Pin                                                    */
/*============================================================================*/

#define HW_GATE_DRIVER_EN_PIN     PC5
#define HW_GATE_DRIVER_EN_ACTIVE  1

/*============================================================================*/
/* Fault Input Pin                                                            */
/*============================================================================*/

#define HW_FAULT_PIN              PC6
#define HW_FAULT_ACTIVE_LOW       1

/*============================================================================*/
/* Hardware Initialization                                                    */
/*============================================================================*/

void hw_hk32m070_qfn48_init(void);

#ifdef __cplusplus
}
#endif

#endif /* HW_HK32M070_QFN48_H */
