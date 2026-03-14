/**
 * @file types.h
 * @brief Common type definitions for HK32M070 FOC VESC
 */

#ifndef TYPES_H
#define TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/*============================================================================*/
/* Basic Types                                                                */
/*============================================================================*/

typedef int16_t     q15_t;      // Q1.15 fixed point (-1.0 to 0.9999)
typedef int32_t     q31_t;      // Q1.31 fixed point
typedef int16_t     q8_t;       // Q8.8 fixed point (0-255.99)
typedef int32_t     q16_t;      // Q16.16 fixed point

typedef uint16_t    uq15_t;     // Unsigned Q0.15
typedef uint32_t    uq31_t;     // Unsigned Q0.31

/*============================================================================*/
/* Fixed Point Macros                                                         */
/*============================================================================*/

// Q15 operations
#define Q15_MAX         32767
#define Q15_MIN         (-32768)
#define Q15_ONE         32767
#define Q15_HALF        16384

// Q31 operations
#define Q31_MAX         2147483647
#define Q31_MIN         (-2147483648)
#define Q31_ONE         2147483647

// Q8.8 operations
#define Q8_MAX          32767
#define Q8_ONE          256

// Convert float to Q15 (for compile-time constants only)
#define FLOAT_TO_Q15(x) ((q15_t)((x) * 32767.0f))

// Convert float to Q31 (for compile-time constants only)
#define FLOAT_TO_Q31(x) ((q31_t)((x) * 2147483647.0f))

// Convert Q15 to float
#define Q15_TO_FLOAT(x) ((float)(x) / 32767.0f)

/*============================================================================*/
/* Math Macros                                                                */
/*============================================================================*/

#define MIN(a, b)       ((a) < (b) ? (a) : (b))
#define MAX(a, b)       ((a) > (b) ? (a) : (b))
#define CLAMP(x, lo, hi) MIN(MAX(x, lo), hi)
#define ABS(x)          ((x) < 0 ? -(x) : (x))

#define PI              3.14159265358979f
#define TWO_PI          6.28318530717959f
#define INV_TWO_PI      0.15915494309190f

/*============================================================================*/
/* Bit Manipulation Macros                                                    */
/*============================================================================*/

#define BIT_SET(reg, bit)       ((reg) |= (bit))
#define BIT_CLR(reg, bit)       ((reg) &= ~(bit))
#define BIT_TOGGLE(reg, bit)    ((reg) ^= (bit))
#define BIT_CHECK(reg, bit)     ((reg) & (bit))

/*============================================================================*/
/* Compiler Attributes                                                        */
/*============================================================================*/

#define STATIC_ASSERT(cond)     _Static_assert(cond, #cond)

#ifdef __GNUC__
    #define UNUSED          __attribute__((unused))
    #define PACKED          __attribute__((packed))
    #define WEAK            __attribute__((weak))
    #define ALIGNED(n)      __attribute__((aligned(n)))
    #define NORETURN        __attribute__((noreturn))
    #define SECTION(s)      __attribute__((section(s)))
    #define RAM_FUNC        __attribute__((section(".ram_code")))
#else
    #define UNUSED
    #define PACKED
    #define WEAK
    #define ALIGNED(n)
    #define NORETURN
    #define SECTION(s)
    #define RAM_FUNC
#endif

/*============================================================================*/
/* Common Macros                                                              */
/*============================================================================*/

#define ARRAY_SIZE(x)       (sizeof(x) / sizeof((x)[0]))
#define BYTE0(x)            ((uint8_t)((x) & 0xFF))
#define BYTE1(x)            ((uint8_t)(((x) >> 8) & 0xFF))
#define BYTE2(x)            ((uint8_t)(((x) >> 16) & 0xFF))
#define BYTE3(x)            ((uint8_t)(((x) >> 24) & 0xFF))

#ifdef __cplusplus
}
#endif

#endif /* TYPES_H */
