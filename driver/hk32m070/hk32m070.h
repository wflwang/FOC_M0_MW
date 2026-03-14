/**
 * @file hk32m070.h
 * @brief HK32M070 MCU Header File
 * @note  HK32M070 is a Cortex-M0 microcontroller from HK (HK MicroChip)
 *        64MHz max, 32KB Flash, 8KB SRAM
 */

#ifndef HK32M070_H
#define HK32M070_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/*============================================================================*/
/* CPU Configuration                                                          */
/*============================================================================*/

#define __CORTEX_M          0           /* Cortex-M0 core */
#define __FPU_PRESENT       0           /* No FPU */
#define __MPU_PRESENT       0           /* No MPU */
#define __NVIC_PRIO_BITS    2           /* 2 bits for priority levels */
#define __Vendor_SysTickConfig 0

/*============================================================================*/
/* Memory Map                                                                 */
/*============================================================================*/

#define FLASH_BASE          0x08000000UL
#define SRAM_BASE           0x20000000UL
#define PERIPH_BASE         0x40000000UL

#define APB1PERIPH_BASE     PERIPH_BASE
#define APB2PERIPH_BASE     (PERIPH_BASE + 0x00010000UL)
#define AHBPERIPH_BASE      (PERIPH_BASE + 0x00020000UL)

/*============================================================================*/
/* Peripheral Base Addresses                                                  */
/*============================================================================*/

/* GPIO */
#define GPIOA_BASE          (AHBPERIPH_BASE + 0x0000UL)
#define GPIOB_BASE          (AHBPERIPH_BASE + 0x0400UL)
#define GPIOC_BASE          (AHBPERIPH_BASE + 0x0800UL)
#define GPIOD_BASE          (AHBPERIPH_BASE + 0x0C00UL)

/* TIM */
#define TIM1_BASE           (APB2PERIPH_BASE + 0x2C00UL)
#define TIM2_BASE           (APB1PERIPH_BASE + 0x0000UL)
#define TIM3_BASE           (APB1PERIPH_BASE + 0x0400UL)
#define TIM6_BASE           (APB1PERIPH_BASE + 0x1000UL)
#define TIM14_BASE          (APB1PERIPH_BASE + 0x2000UL)
#define TIM15_BASE          (APB2PERIPH_BASE + 0x4000UL)
#define TIM16_BASE          (APB2PERIPH_BASE + 0x4400UL)
#define TIM17_BASE          (APB2PERIPH_BASE + 0x4800UL)

/* ADC */
#define ADC1_BASE           (APB2PERIPH_BASE + 0x2400UL)

/* DMA */
#define DMA1_BASE           (AHBPERIPH_BASE + 0x2000UL)

/* UART */
#define UART1_BASE          (APB1PERIPH_BASE + 0x3800UL)
#define UART2_BASE          (APB1PERIPH_BASE + 0x4400UL)

/* SPI */
#define SPI1_BASE           (APB2PERIPH_BASE + 0x3000UL)
#define SPI2_BASE           (APB1PERIPH_BASE + 0x3800UL)

/* I2C */
#define I2C1_BASE           (APB1PERIPH_BASE + 0x5400UL)

/* CAN */
#define CAN1_BASE           (APB1PERIPH_BASE + 0x6400UL)

/* System Control */
#define RCC_BASE            (AHBPERIPH_BASE + 0x1000UL)
#define FLASH_BASE_REGS     (AHBPERIPH_BASE + 0x2000UL)
#define SYSCFG_BASE         (APB2PERIPH_BASE + 0x0000UL)

/*============================================================================*/
/* Peripheral Type Definitions                                                */
/*============================================================================*/

typedef struct {
    volatile uint32_t CRL;
    volatile uint32_t CRH;
    volatile uint32_t IDR;
    volatile uint32_t ODR;
    volatile uint32_t BSRR;
    volatile uint32_t BRR;
    volatile uint32_t LCKR;
} GPIO_TypeDef;

typedef struct {
    volatile uint32_t CR;
    volatile uint32_t CFGR;
    volatile uint32_t CIR;
    volatile uint32_t APB2RSTR;
    volatile uint32_t APB1RSTR;
    volatile uint32_t AHBENR;
    volatile uint32_t APB2ENR;
    volatile uint32_t APB1ENR;
    volatile uint32_t BDCR;
    volatile uint32_t CSR;
    volatile uint32_t AHBRSTR;
    volatile uint32_t CFGR2;
} RCC_TypeDef;

typedef struct {
    volatile uint32_t CR;
    volatile uint32_t CFGR;
    volatile uint32_t CIR;
    volatile uint32_t ACPR;
    volatile uint32_t RESERVED0;
    volatile uint32_t AHBENR;
    volatile uint32_t RESERVED1;
    volatile uint32_t RESERVED2;
} SYSCFG_TypeDef;

typedef struct {
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t SMCR;
    volatile uint32_t DIER;
    volatile uint32_t SR;
    volatile uint32_t EGR;
    volatile uint32_t CCMR1;
    volatile uint32_t CCMR2;
    volatile uint32_t CCER;
    volatile uint32_t CNT;
    volatile uint32_t PSC;
    volatile uint32_t ARR;
    volatile uint32_t RCR;
    volatile uint32_t CCR1;
    volatile uint32_t CCR2;
    volatile uint32_t CCR3;
    volatile uint32_t CCR4;
    volatile uint32_t BDTR;
    volatile uint32_t DCR;
    volatile uint32_t DMAR;
} TIM_TypeDef;

typedef struct {
    volatile uint32_t ISR;
    volatile uint32_t IER;
    volatile uint32_t CR;
    volatile uint32_t CFGR1;
    volatile uint32_t CFGR2;
    volatile uint32_t SMPR;
    volatile uint32_t RESERVED0;
    volatile uint32_t RESERVED1;
    volatile uint32_t CHSELR;
    volatile uint32_t RESERVED2[5];
    volatile uint32_t DR;
} ADC_TypeDef;

typedef struct {
    volatile uint32_t CCR;
} DMA_Channel_TypeDef;

typedef struct {
    volatile uint32_t ISR;
    volatile uint32_t IFCR;
} DMA_TypeDef;

typedef struct {
    volatile uint32_t SR;
    volatile uint32_t DR;
    volatile uint32_t BRR;
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t CR3;
    volatile uint32_t GTPR;
} USART_TypeDef;

typedef struct {
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t SR;
    volatile uint32_t DR;
    volatile uint32_t CRCPR;
    volatile uint32_t RXCRCR;
    volatile uint32_t TXCRCR;
} SPI_TypeDef;

typedef struct {
    volatile uint32_t ACR;
    volatile uint32_t KEYR;
    volatile uint32_t OPTKEYR;
    volatile uint32_t SR;
    volatile uint32_t CR;
    volatile uint32_t AR;
    volatile uint32_t RESERVED0;
    volatile uint32_t OBR;
    volatile uint32_t WRPR;
} FLASH_TypeDef;

/*============================================================================*/
/* Peripheral Pointers                                                        */
/*============================================================================*/

#define GPIOA               ((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB               ((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOC               ((GPIO_TypeDef *) GPIOC_BASE)
#define GPIOD               ((GPIO_TypeDef *) GPIOD_BASE)

#define RCC                 ((RCC_TypeDef *) RCC_BASE)
#define SYSCFG              ((SYSCFG_TypeDef *) SYSCFG_BASE)

#define TIM1                ((TIM_TypeDef *) TIM1_BASE)
#define TIM2                ((TIM_TypeDef *) TIM2_BASE)
#define TIM3                ((TIM_TypeDef *) TIM3_BASE)

#define ADC1                ((ADC_TypeDef *) ADC1_BASE)

#define DMA1                ((DMA_TypeDef *) DMA1_BASE)

#define USART1              ((USART_TypeDef *) UART1_BASE)
#define USART2              ((USART_TypeDef *) UART2_BASE)

#define SPI1                ((SPI_TypeDef *) SPI1_BASE)
#define SPI2                ((SPI_TypeDef *) SPI2_BASE)

#define FLASH               ((FLASH_TypeDef *) FLASH_BASE_REGS)

/*============================================================================*/
/* RCC Bit Definitions                                                        */
/*============================================================================*/

/* CR register bits */
#define RCC_CR_HSION        (1 << 0)
#define RCC_CR_HSIRDY       (1 << 1)
#define RCC_CR_HSEON        (1 << 16)
#define RCC_CR_HSERDY       (1 << 17)
#define RCC_CR_PLLON        (1 << 24)
#define RCC_CR_PLLRDY       (1 << 25)

/* CFGR register bits */
#define RCC_CFGR_SW         (0x3 << 0)
#define RCC_CFGR_SW_HSI     (0x0 << 0)
#define RCC_CFGR_SW_HSE     (0x1 << 0)
#define RCC_CFGR_SW_PLL     (0x2 << 0)
#define RCC_CFGR_SWS        (0x3 << 2)
#define RCC_CFGR_SWS_PLL    (0x2 << 2)
#define RCC_CFGR_HPRE       (0xF << 4)
#define RCC_CFGR_PPRE1      (0x7 << 8)
#define RCC_CFGR_PPRE1_DIV1 (0x0 << 8)
#define RCC_CFGR_PPRE2      (0x7 << 11)
#define RCC_CFGR_PPRE2_DIV1 (0x0 << 11)
#define RCC_CFGR_PLLSRC     (1 << 16)
#define RCC_CFGR_PLLMUL     (0xF << 18)
#define RCC_CFGR_PLLMUL8    (0x6 << 18)

/* AHBENR register bits */
#define RCC_AHBENR_GPIOAEN  (1 << 17)
#define RCC_AHBENR_GPIOBEN  (1 << 18)
#define RCC_AHBENR_GPIOCEN  (1 << 19)
#define RCC_AHBENR_GPIODEN  (1 << 20)
#define RCC_AHBENR_DMA1EN   (1 << 0)

/* APB1ENR register bits */
#define RCC_APB1ENR_TIM2EN  (1 << 0)
#define RCC_APB1ENR_TIM3EN  (1 << 1)
#define RCC_APB1ENR_UART1EN (1 << 14)
#define RCC_APB1ENR_CAN1EN  (1 << 25)

/* APB2ENR register bits */
#define RCC_APB2ENR_TIM1EN  (1 << 11)
#define RCC_APB2ENR_ADC1EN  (1 << 9)
#define RCC_APB2ENR_SPI1EN  (1 << 12)
#define RCC_APB2ENR_SYSCFGCOMPEN (1 << 0)

/*============================================================================*/
/* GPIO Bit Definitions                                                       */
/*============================================================================*/

#define GPIO_PIN_0          (1 << 0)
#define GPIO_PIN_1          (1 << 1)
#define GPIO_PIN_2          (1 << 2)
#define GPIO_PIN_3          (1 << 3)
#define GPIO_PIN_4          (1 << 4)
#define GPIO_PIN_5          (1 << 5)
#define GPIO_PIN_6          (1 << 6)
#define GPIO_PIN_7          (1 << 7)
#define GPIO_PIN_8          (1 << 8)
#define GPIO_PIN_9          (1 << 9)
#define GPIO_PIN_10         (1 << 10)
#define GPIO_PIN_11         (1 << 11)
#define GPIO_PIN_12         (1 << 12)
#define GPIO_PIN_13         (1 << 13)
#define GPIO_PIN_14         (1 << 14)
#define GPIO_PIN_15         (1 << 15)
#define GPIO_PIN_ALL        (0xFFFF)

/*============================================================================*/
/* ADC Bit Definitions                                                        */
/*============================================================================*/

#define ADC_CR_ADCAL        (1 << 31)
#define ADC_CR_ADEN         (1 << 0)
#define ADC_CR_ADSTART      (1 << 2)
#define ADC_ISR_ADRDY       (1 << 0)

#define ADC_SMPR_SMP        (0x7 << 0)
#define ADC_SMPR_SMP_0      (1 << 0)
#define ADC_SMPR_SMP_1      (1 << 1)
#define ADC_SMPR_SMP_2      (1 << 2)

#define ADC_CFGR1_CONT      (1 << 1)
#define ADC_CFGR1_DMAEN     (1 << 14)

#define ADC_CHSELR_CHSEL0   (1 << 0)
#define ADC_CHSELR_CHSEL1   (1 << 1)
#define ADC_CHSELR_CHSEL2   (1 << 2)
#define ADC_CHSELR_CHSEL3   (1 << 3)
#define ADC_CHSELR_CHSEL4   (1 << 4)
#define ADC_CHSELR_CHSEL5   (1 << 5)

/*============================================================================*/
/* TIM Bit Definitions                                                        */
/*============================================================================*/

#define TIM_CR1_CEN         (1 << 0)
#define TIM_CR1_UDIS        (1 << 1)
#define TIM_CR1_URS         (1 << 2)
#define TIM_CR1_ARPE        (1 << 7)

#define TIM_CR2_MMS_2       (1 << 6)

#define TIM_BDTR_MOE        (1 << 15)
#define TIM_BDTR_AOE        (1 << 14)
#define TIM_BDTR_DTG_Pos    0

#define TIM_CCMR1_OC1M_Pos  4
#define TIM_CCMR1_OC1PE     (1 << 3)
#define TIM_CCMR1_OC2M_Pos  12
#define TIM_CCMR1_OC2PE     (1 << 11)

#define TIM_CCMR2_OC3M_Pos  4
#define TIM_CCMR2_OC3PE     (1 << 3)
#define TIM_CCMR2_OC4M_Pos  12
#define TIM_CCMR2_OC4PE     (1 << 11)

#define TIM_CCER_CC1E       (1 << 0)
#define TIM_CCER_CC1NE      (1 << 2)
#define TIM_CCER_CC2E       (1 << 4)
#define TIM_CCER_CC2NE      (1 << 6)
#define TIM_CCER_CC3E       (1 << 8)
#define TIM_CCER_CC3NE      (1 << 10)

/*============================================================================*/
/* EXTI Bit Definitions                                                       */
/*============================================================================*/

#define EXTI_IMR_IM6        (1 << 6)
#define EXTI_IMR_IM7        (1 << 7)
#define EXTI_IMR_IM8        (1 << 8)
#define EXTI_EMR_EM6        (1 << 6)
#define EXTI_EMR_EM7        (1 << 7)
#define EXTI_EMR_EM8        (1 << 8)
#define EXTI_RTSR_RT6       (1 << 6)
#define EXTI_RTSR_RT7       (1 << 7)
#define EXTI_RTSR_RT8       (1 << 8)
#define EXTI_FTSR_FT6       (1 << 6)
#define EXTI_FTSR_FT7       (1 << 7)
#define EXTI_FTSR_FT8       (1 << 8)
#define EXTI_PR_PR6         (1 << 6)
#define EXTI_PR_PR7         (1 << 7)
#define EXTI_PR_PR8         (1 << 8)

/*============================================================================*/
/* SYSCFG Bit Definitions                                                     */
/*============================================================================*/

#define SYSCFG_EXTICR2_EXTI6_PB  (0x1 << 8)
#define SYSCFG_EXTICR2_EXTI7_PB  (0x1 << 12)
#define SYSCFG_EXTICR2_EXTI8_PB  (0x1 << 16)

/*============================================================================*/
/* FLASH Bit Definitions                                                      */
/*============================================================================*/

#define FLASH_ACR_LATENCY_2     (0x2 << 0)

/*============================================================================*/
/* DMA Bit Definitions                                                        */
/*============================================================================*/

#define DMA_CCR_EN          (1 << 0)
#define DMA_CCR_TCIE        (1 << 1)
#define DMA_CCR_HTIE        (1 << 2)
#define DMA_CCR_TEIE        (1 << 3)
#define DMA_CCR_DIR         (1 << 4)
#define DMA_CCR_CIRC        (1 << 5)
#define DMA_CCR_PINC        (1 << 6)
#define DMA_CCR_MINC        (1 << 7)
#define DMA_CCR_PSIZE       (0x3 << 8)
#define DMA_CCR_MSIZE       (0x3 << 10)
#define DMA_CCR_PL_1        (1 << 13)

/*============================================================================*/
/* GPIO Init Structure                                                        */
/*============================================================================*/

typedef struct {
    uint16_t Pin;
    uint8_t Mode;
    uint8_t Pull;
    uint8_t Speed;
    uint8_t Alternate;
} GPIO_InitTypeDef;

/* GPIO Mode */
#define GPIO_MODE_INPUT          0x00
#define GPIO_MODE_OUTPUT_PP      0x01
#define GPIO_MODE_OUTPUT_OD      0x02
#define GPIO_MODE_AF_PP          0x03
#define GPIO_MODE_AF_OD          0x04
#define GPIO_MODE_ANALOG         0x05
#define GPIO_MODE_IT_RISING      0x10
#define GPIO_MODE_IT_FALLING     0x11
#define GPIO_MODE_IT_RISING_FALLING 0x12

/* GPIO Pull */
#define GPIO_NOPULL              0x00
#define GPIO_PULLUP              0x01
#define GPIO_PULLDOWN            0x02

/* GPIO Speed */
#define GPIO_SPEED_FREQ_LOW      0x00
#define GPIO_SPEED_FREQ_MEDIUM   0x01
#define GPIO_SPEED_FREQ_HIGH     0x02
#define GPIO_SPEED_FREQ_VERY_HIGH 0x03

/*============================================================================*/
/* HAL Functions                                                              */
/*============================================================================*/

void HAL_GPIO_Init(GPIO_TypeDef *GPIOx, GPIO_InitTypeDef *GPIO_Init);
void HAL_GPIO_DeInit(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void HAL_GPIO_WritePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, uint8_t PinState);
void HAL_GPIO_TogglePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
uint8_t HAL_GPIO_ReadPin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

/*============================================================================*/
/* Core Functions                                                             */
/*============================================================================*/

void __enable_irq(void);
void __disable_irq(void);
void NVIC_EnableIRQ(uint8_t IRQn);
void NVIC_DisableIRQ(uint8_t IRQn);
void NVIC_SetPriority(uint8_t IRQn, uint8_t Priority);

/*============================================================================*/
/* IRQ Numbers                                                                */
/*============================================================================*/

#define EXTI4_15_IRQn       23
#define TIM1_BRK_UP_TRG_COM_IRQn  24
#define TIM1_CC_IRQn        25
#define TIM2_IRQn           26
#define TIM3_IRQn           27

#ifdef __cplusplus
}
#endif

#endif /* HK32M070_H */
