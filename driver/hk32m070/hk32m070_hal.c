/**
 * @file hk32m070_hal.c
 * @brief HK32M070 HAL Layer Implementation
 */

#include "hk32m070.h"

/*============================================================================*/
/* Core Functions                                                             */
/*============================================================================*/

void __enable_irq(void) {
    __asm volatile ("cpsie i");
}

void __disable_irq(void) {
    __asm volatile ("cpsid i");
}

void NVIC_EnableIRQ(uint8_t IRQn) {
    volatile uint32_t *nvic_iser = (volatile uint32_t *)0xE000E100;
    nvic_iser[IRQn >> 5] = (1 << (IRQn & 0x1F));
}

void NVIC_DisableIRQ(uint8_t IRQn) {
    volatile uint32_t *nvic_icer = (volatile uint32_t *)0xE000E180;
    nvic_icer[IRQn >> 5] = (1 << (IRQn & 0x1F));
}

void NVIC_SetPriority(uint8_t IRQn, uint8_t Priority) {
    volatile uint8_t *nvic_ipr = (volatile uint8_t *)0xE000E400;
    nvic_ipr[IRQn] = (Priority << 6);
}

/*============================================================================*/
/* GPIO HAL Functions                                                         */
/*============================================================================*/

/**
 * @brief Initialize GPIO pin
 */
void HAL_GPIO_Init(GPIO_TypeDef *GPIOx, GPIO_InitTypeDef *GPIO_Init) {
    uint32_t position = 0;
    uint32_t current_pin;
    uint32_t mode_config = 0;
    uint32_t pull_config = 0;
    uint32_t speed_config = 0;
    
    while ((GPIO_Init->Pin >> position) != 0) {
        current_pin = GPIO_Init->Pin & (1 << position);
        
        if (current_pin) {
            /* Configure mode */
            switch (GPIO_Init->Mode) {
                case GPIO_MODE_INPUT:
                    mode_config = 0x0;
                    break;
                case GPIO_MODE_OUTPUT_PP:
                    mode_config = 0x1;
                    break;
                case GPIO_MODE_OUTPUT_OD:
                    mode_config = 0x5;
                    break;
                case GPIO_MODE_AF_PP:
                    mode_config = 0x2;
                    break;
                case GPIO_MODE_AF_OD:
                    mode_config = 0x6;
                    break;
                case GPIO_MODE_ANALOG:
                    mode_config = 0x3;
                    break;
                default:
                    mode_config = 0x0;
                    break;
            }
            
            /* Configure pull-up/pull-down */
            switch (GPIO_Init->Pull) {
                case GPIO_NOPULL:
                    pull_config = 0x0;
                    break;
                case GPIO_PULLUP:
                    pull_config = 0x1;
                    break;
                case GPIO_PULLDOWN:
                    pull_config = 0x2;
                    break;
                default:
                    pull_config = 0x0;
                    break;
            }
            
            /* Configure speed */
            switch (GPIO_Init->Speed) {
                case GPIO_SPEED_FREQ_LOW:
                    speed_config = 0x0;
                    break;
                case GPIO_SPEED_FREQ_MEDIUM:
                    speed_config = 0x1;
                    break;
                case GPIO_SPEED_FREQ_HIGH:
                    speed_config = 0x2;
                    break;
                case GPIO_SPEED_FREQ_VERY_HIGH:
                    speed_config = 0x3;
                    break;
                default:
                    speed_config = 0x0;
                    break;
            }
            
            /* Configure CRL or CRH based on pin number */
            if (position < 8) {
                /* Low pins (0-7): CRL register */
                volatile uint32_t *crl = &GPIOx->CRL;
                uint32_t shift = position * 4;
                
                *crl &= ~(0xF << shift);           /* Clear CNF and MODE */
                *crl |= (mode_config << shift);     /* Set MODE */
                
                /* Handle CNF bits for pull-up/down */
                if (GPIO_Init->Mode == GPIO_MODE_INPUT) {
                    if (GPIO_Init->Pull == GPIO_PULLUP || GPIO_Init->Pull == GPIO_PULLDOWN) {
                        *crl |= (0x8 << shift);     /* Input with pull */
                    }
                }
            } else {
                /* High pins (8-15): CRH register */
                volatile uint32_t *crh = &GPIOx->CRH;
                uint32_t shift = (position - 8) * 4;
                
                *crh &= ~(0xF << shift);
                *crh |= (mode_config << shift);
                
                if (GPIO_Init->Mode == GPIO_MODE_INPUT) {
                    if (GPIO_Init->Pull == GPIO_PULLUP || GPIO_Init->Pull == GPIO_PULLDOWN) {
                        *crh |= (0x8 << shift);
                    }
                }
            }
            
            /* Set pull-up/pull-down via ODR */
            if (GPIO_Init->Pull == GPIO_PULLUP) {
                GPIOx->BSRR = current_pin;
            } else if (GPIO_Init->Pull == GPIO_PULLDOWN) {
                GPIOx->BRR = current_pin;
            }
        }
        
        position++;
    }
}

/**
 * @brief DeInitialize GPIO pin
 */
void HAL_GPIO_DeInit(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
    uint32_t position = 0;
    uint32_t current_pin;
    
    while ((GPIO_Pin >> position) != 0) {
        current_pin = GPIO_Pin & (1 << position);
        
        if (current_pin) {
            /* Reset pin to analog mode (lowest power) */
            if (position < 8) {
                GPIOx->CRL &= ~(0xF << (position * 4));
                GPIOx->CRL |= (0x4 << (position * 4)); /* Floating input */
            } else {
                GPIOx->CRH &= ~(0xF << ((position - 8) * 4));
                GPIOx->CRH |= (0x4 << ((position - 8) * 4));
            }
        }
        
        position++;
    }
}

/**
 * @brief Write GPIO pin
 */
void HAL_GPIO_WritePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, uint8_t PinState) {
    if (PinState != 0) {
        GPIOx->BSRR = GPIO_Pin;
    } else {
        GPIOx->BRR = GPIO_Pin;
    }
}

/**
 * @brief Toggle GPIO pin
 */
void HAL_GPIO_TogglePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
    GPIOx->ODR ^= GPIO_Pin;
}

/**
 * @brief Read GPIO pin
 */
uint8_t HAL_GPIO_ReadPin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
    return (GPIOx->IDR & GPIO_Pin) ? 1 : 0;
}
