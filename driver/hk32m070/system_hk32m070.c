/**
 * @file system_hk32m070.c
 * @brief System initialization for HK32M070
 */

#include "hk32m070.h"

/*============================================================================*/
/* System Core Clock Variable                                                  */
/*============================================================================*/

uint32_t SystemCoreClock = 64000000UL;

/*============================================================================*/
/* SystemInit - Called before main()                                          */
/*============================================================================*/

void SystemInit(void) {
    /* Reset the RCC clock configuration to the default reset state */
    
    /* Set HSION bit */
    RCC->CR |= RCC_CR_HSION;
    
    /* Reset CFGR register */
    RCC->CFGR = 0x00000000;
    
    /* Reset HSEON, CSSON and PLLON bits */
    RCC->CR &= ~(RCC_CR_HSEON | RCC_CR_PLLON);
    
    /* Reset PLL configuration */
    RCC->CFGR &= ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLMUL);
    
    /* Update SystemCoreClock variable */
    SystemCoreClock = 8000000UL;  /* Default HSI 8MHz */
}

/*============================================================================*/
/* SystemCoreClockUpdate - Update SystemCoreClock variable                    */
/*============================================================================*/

void SystemCoreClockUpdate(void) {
    uint32_t hsi_freq = 8000000UL;
    uint32_t hse_freq = 8000000UL;  /* Assume 8MHz external crystal */
    uint32_t pll_mul;
    uint32_t clock_source;
    
    /* Get clock source */
    clock_source = RCC->CFGR & RCC_CFGR_SWS;
    
    switch (clock_source) {
        case 0x00:  /* HSI used as system clock */
            SystemCoreClock = hsi_freq;
            break;
            
        case 0x04:  /* HSE used as system clock */
            SystemCoreClock = hse_freq;
            break;
            
        case 0x08:  /* PLL used as system clock */
            /* Get PLL multiplication factor */
            pll_mul = (RCC->CFGR & RCC_CFGR_PLLMUL) >> 18;
            
            /* PLL multiplier: 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16 */
            pll_mul = pll_mul + 2;
            
            /* Check PLL source */
            if (RCC->CFGR & RCC_CFGR_PLLSRC) {
                /* HSE as PLL source */
                SystemCoreClock = hse_freq * pll_mul;
            } else {
                /* HSI/2 as PLL source */
                SystemCoreClock = (hsi_freq / 2) * pll_mul;
            }
            break;
            
        default:
            SystemCoreClock = hsi_freq;
            break;
    }
}
