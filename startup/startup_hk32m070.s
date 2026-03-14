/**
 * @file startup_hk32m070.s
 * @brief Startup code for HK32M070 (Cortex-M0)
 * 
 * This file contains:
 *  - Interrupt vector table
 *  - Reset handler
 *  - Default interrupt handlers
 */

    .syntax unified
    .cpu cortex-m0
    .thumb

/*============================================================================*/
/* Memory Definitions                                                         */
/*============================================================================*/

    .equ    FLASH_BASE,     0x08000000
    .equ    SRAM_BASE,      0x20000000
    .equ    SRAM_SIZE,      8192        /* 8KB */
    .equ    STACK_SIZE,     1024        /* 1KB stack */

/*============================================================================*/
/* Vector Table                                                               */
/*============================================================================*/

    .section .isr_vector, "a", %progbits
    .type   g_pfnVectors, %object
    .size   g_pfnVectors, .-g_pfnVectors

g_pfnVectors:
    .word   _estack                 /* 0: Initial Stack Pointer */
    .word   Reset_Handler           /* 1: Reset Handler */
    .word   NMI_Handler             /* 2: NMI Handler */
    .word   HardFault_Handler       /* 3: Hard Fault Handler */
    .word   0                       /* 4: Reserved */
    .word   0                       /* 5: Reserved */
    .word   0                       /* 6: Reserved */
    .word   0                       /* 7: Reserved */
    .word   0                       /* 8: Reserved */
    .word   0                       /* 9: Reserved */
    .word   0                       /* 10: Reserved */
    .word   SVC_Handler             /* 11: SVCall Handler */
    .word   0                       /* 12: Reserved */
    .word   0                       /* 13: Reserved */
    .word   PendSV_Handler          /* 14: PendSV Handler */
    .word   SysTick_Handler         /* 15: SysTick Handler */

    /* External Interrupts */
    .word   WWDG_IRQHandler         /* 16: Window Watchdog */
    .word   PVD_IRQHandler          /* 17: PVD */
    .word   RTC_IRQHandler          /* 18: RTC */
    .word   FLASH_IRQHandler        /* 19: Flash */
    .word   RCC_CRS_IRQHandler      /* 20: RCC/CRS */
    .word   EXTI0_1_IRQHandler      /* 21: EXTI 0-1 */
    .word   EXTI2_3_IRQHandler      /* 22: EXTI 2-3 */
    .word   EXTI4_15_IRQHandler     /* 23: EXTI 4-15 */
    .word   TSC_IRQHandler          /* 24: Touch Sensing */
    .word   DMA1_Channel1_IRQHandler /* 25: DMA1 Ch1 */
    .word   DMA1_Channel2_3_IRQHandler /* 26: DMA1 Ch2-3 */
    .word   DMA1_Channel4_5_6_7_IRQHandler /* 27: DMA1 Ch4-7 */
    .word   ADC1_COMP_IRQHandler    /* 28: ADC/Comparator */
    .word   TIM1_BRK_UP_TRG_COM_IRQHandler /* 29: TIM1 Break/Update/Trigger/Comm */
    .word   TIM1_CC_IRQHandler      /* 30: TIM1 Capture/Compare */
    .word   TIM2_IRQHandler         /* 31: TIM2 */
    .word   TIM3_IRQHandler         /* 32: TIM3 */
    .word   TIM6_DAC_IRQHandler     /* 33: TIM6/DAC */
    .word   TIM7_IRQHandler         /* 34: TIM7 */
    .word   TIM14_IRQHandler        /* 35: TIM14 */
    .word   TIM15_IRQHandler        /* 36: TIM15 */
    .word   TIM16_IRQHandler        /* 37: TIM16 */
    .word   TIM17_IRQHandler        /* 38: TIM17 */
    .word   I2C1_IRQHandler         /* 39: I2C1 */
    .word   I2C2_IRQHandler         /* 40: I2C2 */
    .word   SPI1_IRQHandler         /* 41: SPI1 */
    .word   SPI2_IRQHandler         /* 42: SPI2 */
    .word   USART1_IRQHandler       /* 43: USART1 */
    .word   USART2_IRQHandler       /* 44: USART2 */
    .word   USART3_4_IRQHandler     /* 45: USART3/4 */
    .word   CEC_CAN_IRQHandler      /* 46: CEC/CAN */
    .word   USB_IRQHandler          /* 47: USB */

/*============================================================================*/
/* Reset Handler                                                              */
/*============================================================================*/

    .section .text.Reset_Handler
    .weak   Reset_Handler
    .type   Reset_Handler, %function

Reset_Handler:
    /* Set stack pointer */
    ldr     r0, =_estack
    mov     sp, r0

    /* Copy .data section from FLASH to SRAM */
    ldr     r0, =_sdata             /* Destination: SRAM start */
    ldr     r1, =_edata             /* Destination end */
    ldr     r2, =_sidata            /* Source: FLASH */
    
copy_data_loop:
    cmp     r0, r1
    bge     copy_data_done
    ldr     r3, [r2], #4
    str     r3, [r0], #4
    b       copy_data_loop
copy_data_done:

    /* Zero fill .bss section */
    ldr     r0, =_sbss
    ldr     r1, =_ebss
    mov     r2, #0

zero_bss_loop:
    cmp     r0, r1
    bge     zero_bss_done
    str     r2, [r0], #4
    b       zero_bss_loop
zero_bss_done:

    /* Call SystemInit (if needed) */
    bl      SystemInit

    /* Call main() */
    bl      main

    /* Infinite loop if main returns */
infinite_loop:
    b       infinite_loop

    .size   Reset_Handler, .-Reset_Handler

/*============================================================================*/
/* Default Handlers                                                           */
/*============================================================================*/

    .section .text.Default_Handler, "ax", %progbits
Default_Handler:
    b       Default_Handler
    .size   Default_Handler, .-Default_Handler

/* Weak aliases for default handlers */
    .weak   NMI_Handler
    .thumb_set NMI_Handler, Default_Handler

    .weak   HardFault_Handler
    .thumb_set HardFault_Handler, Default_Handler

    .weak   SVC_Handler
    .thumb_set SVC_Handler, Default_Handler

    .weak   PendSV_Handler
    .thumb_set PendSV_Handler, Default_Handler

    .weak   SysTick_Handler
    .thumb_set SysTick_Handler, Default_Handler

    .weak   WWDG_IRQHandler
    .thumb_set WWDG_IRQHandler, Default_Handler

    .weak   PVD_IRQHandler
    .thumb_set PVD_IRQHandler, Default_Handler

    .weak   RTC_IRQHandler
    .thumb_set RTC_IRQHandler, Default_Handler

    .weak   FLASH_IRQHandler
    .thumb_set FLASH_IRQHandler, Default_Handler

    .weak   RCC_CRS_IRQHandler
    .thumb_set RCC_CRS_IRQHandler, Default_Handler

    .weak   EXTI0_1_IRQHandler
    .thumb_set EXTI0_1_IRQHandler, Default_Handler

    .weak   EXTI2_3_IRQHandler
    .thumb_set EXTI2_3_IRQHandler, Default_Handler

    .weak   EXTI4_15_IRQHandler
    .thumb_set EXTI4_15_IRQHandler, Default_Handler

    .weak   TSC_IRQHandler
    .thumb_set TSC_IRQHandler, Default_Handler

    .weak   DMA1_Channel1_IRQHandler
    .thumb_set DMA1_Channel1_IRQHandler, Default_Handler

    .weak   DMA1_Channel2_3_IRQHandler
    .thumb_set DMA1_Channel2_3_IRQHandler, Default_Handler

    .weak   DMA1_Channel4_5_6_7_IRQHandler
    .thumb_set DMA1_Channel4_5_6_7_IRQHandler, Default_Handler

    .weak   ADC1_COMP_IRQHandler
    .thumb_set ADC1_COMP_IRQHandler, Default_Handler

    .weak   TIM1_BRK_UP_TRG_COM_IRQHandler
    .thumb_set TIM1_BRK_UP_TRG_COM_IRQHandler, Default_Handler

    .weak   TIM1_CC_IRQHandler
    .thumb_set TIM1_CC_IRQHandler, Default_Handler

    .weak   TIM2_IRQHandler
    .thumb_set TIM2_IRQHandler, Default_Handler

    .weak   TIM3_IRQHandler
    .thumb_set TIM3_IRQHandler, Default_Handler

    .weak   TIM6_DAC_IRQHandler
    .thumb_set TIM6_DAC_IRQHandler, Default_Handler

    .weak   TIM7_IRQHandler
    .thumb_set TIM7_IRQHandler, Default_Handler

    .weak   TIM14_IRQHandler
    .thumb_set TIM14_IRQHandler, Default_Handler

    .weak   TIM15_IRQHandler
    .thumb_set TIM15_IRQHandler, Default_Handler

    .weak   TIM16_IRQHandler
    .thumb_set TIM16_IRQHandler, Default_Handler

    .weak   TIM17_IRQHandler
    .thumb_set TIM17_IRQHandler, Default_Handler

    .weak   I2C1_IRQHandler
    .thumb_set I2C1_IRQHandler, Default_Handler

    .weak   I2C2_IRQHandler
    .thumb_set I2C2_IRQHandler, Default_Handler

    .weak   SPI1_IRQHandler
    .thumb_set SPI1_IRQHandler, Default_Handler

    .weak   SPI2_IRQHandler
    .thumb_set SPI2_IRQHandler, Default_Handler

    .weak   USART1_IRQHandler
    .thumb_set USART1_IRQHandler, Default_Handler

    .weak   USART2_IRQHandler
    .thumb_set USART2_IRQHandler, Default_Handler

    .weak   USART3_4_IRQHandler
    .thumb_set USART3_4_IRQHandler, Default_Handler

    .weak   CEC_CAN_IRQHandler
    .thumb_set CEC_CAN_IRQHandler, Default_Handler

    .weak   USB_IRQHandler
    .thumb_set USB_IRQHandler, Default_Handler

    .end
