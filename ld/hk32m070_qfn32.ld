/**
 * Linker Script for HK32M070 QFN32
 * 32KB Flash, 8KB SRAM
 */

MEMORY
{
    FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 32K
    SRAM (rwx)  : ORIGIN = 0x20000000, LENGTH = 8K
}

/* Stack and heap sizes */
_stack_size = 1024;     /* 1KB stack */
_heap_size = 512;       /* 512B heap */

/* Entry point */
ENTRY(Reset_Handler)

SECTIONS
{
    /* Interrupt vector table */
    .isr_vector :
    {
        . = ALIGN(4);
        KEEP(*(.isr_vector))
        . = ALIGN(4);
    } > FLASH

    /* Code section */
    .text :
    {
        . = ALIGN(4);
        *(.text)
        *(.text*)
        *(.rodata)
        *(.rodata*)
        . = ALIGN(4);
        _etext = .;
    } > FLASH

    /* Initialized data - loaded from FLASH to SRAM */
    _sidata = LOADADDR(.data);
    
    .data :
    {
        . = ALIGN(4);
        _sdata = .;
        *(.data)
        *(.data*)
        . = ALIGN(4);
        _edata = .;
    } > SRAM AT> FLASH

    /* Uninitialized data */
    .bss :
    {
        . = ALIGN(4);
        _sbss = .;
        *(.bss)
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
    } > SRAM

    /* Heap */
    .heap :
    {
        . = ALIGN(4);
        _heap_start = .;
        . = . + _heap_size;
        _heap_end = .;
    } > SRAM

    /* Stack */
    .stack :
    {
        . = ALIGN(8);
        _stack_end = .;
        . = . + _stack_size;
        . = ALIGN(8);
        _stack_start = .;
    } > SRAM

    /* Provide symbols for startup code */
    _end = .;
}

/* Provide stack pointer initial value */
PROVIDE(_initial_sp = _stack_start);

/* Memory info for runtime */
PROVIDE(_flash_start = ORIGIN(FLASH));
PROVIDE(_flash_end = ORIGIN(FLASH) + LENGTH(FLASH));
PROVIDE(_sram_start = ORIGIN(SRAM));
PROVIDE(_sram_end = ORIGIN(SRAM) + LENGTH(SRAM));
