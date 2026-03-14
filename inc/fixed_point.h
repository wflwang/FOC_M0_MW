/**
 * @file fixed_point.h
 * @brief Fixed Point Math Library Header
 */

#ifndef FIXED_POINT_H
#define FIXED_POINT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "types.h"

/*============================================================================*/
/* Sin/Cos Lookup Table Functions                                             */
/*============================================================================*/

/**
 * @brief Initialize sin/cos lookup table
 */
void foc_sincos_init(void);

/**
 * @brief Get sin and cos values from angle
 * @param angle Electrical angle (0-65535 = 0-2π)
 * @param sin_out Sin output (Q15)
 * @param cos_out Cos output (Q15)
 */
void foc_sincos(q15_t angle, q15_t *sin_out, q15_t *cos_out);

/**
 * @brief Fast sin approximation (Q15)
 */
q15_t foc_sin(q15_t angle);

/**
 * @brief Fast cos approximation (Q15)
 */
q15_t foc_cos(q15_t angle);

/*============================================================================*/
/* Arithmetic Functions                                                       */
/*============================================================================*/

/**
 * @brief Q15 multiplication with rounding
 */
static inline q15_t q15_mul(q15_t a, q15_t b) {
    return (q15_t)(((int32_t)a * b + 0x4000) >> 15);
}

/**
 * @brief Q31 multiplication with rounding
 */
static inline q31_t q31_mul(q31_t a, q31_t b) {
    int64_t result = (int64_t)a * b;
    return (q31_t)((result + 0x40000000LL) >> 31);
}

/**
 * @brief Q15 division
 */
static inline q15_t q15_div(q15_t a, q15_t b) {
    if (b == 0) return (a >= 0) ? Q15_MAX : Q15_MIN;
    return (q15_t)(((int32_t)a << 15) / b);
}

/*============================================================================*/
/* CORDIC Atan2 (No Floating Point)                                           */
/*============================================================================*/

/**
 * @brief Calculate atan2 using CORDIC algorithm
 * @param y Y coordinate (Q15)
 * @param x X coordinate (Q15)
 * @return Angle in Q15 format (0-65535 = 0-2π)
 */
q15_t cordic_atan2(q15_t y, q15_t x);

/*============================================================================*/
/* Low Pass Filter                                                            */
/*============================================================================*/

typedef struct {
    q31_t state;
    q15_t coeff;   // Filter coefficient (Q15)
} lpf_q15_t;

/**
 * @brief Initialize LPF
 */
static inline void lpf_init(lpf_q15_t *lpf, q15_t coeff) {
    lpf->state = 0;
    lpf->coeff = coeff;
}

/**
 * @brief Update LPF
 */
static inline q15_t lpf_update(lpf_q15_t *lpf, q15_t input) {
    lpf->state += (q31_t)input * lpf->coeff;
    lpf->state -= (lpf->state >> 15) * lpf->coeff;
    return (q15_t)(lpf->state >> 15);
}

/*============================================================================*/
/* Transform Functions                                                        */
/*============================================================================*/

/**
 * @brief Clarke transform (3-phase to 2-phase)
 */
static inline void clarke_transform(q15_t ia, q15_t ib, q15_t ic, 
                                     q15_t *alpha, q15_t *beta) {
    *alpha = ia;
    *beta = (q15_t)(((int32_t)ia + 2 * (int32_t)ib) * 18919 >> 15);
    (void)ic; // ic = -ia - ib, not needed
}

/**
 * @brief Park transform (stationary to rotating frame)
 */
static inline void park_transform(q15_t alpha, q15_t beta,
                                   q15_t sin_theta, q15_t cos_theta,
                                   q15_t *d, q15_t *q) {
    *d = (q15_t)(((int32_t)alpha * cos_theta + (int32_t)beta * sin_theta) >> 15);
    *q = (q15_t)((-(int32_t)alpha * sin_theta + (int32_t)beta * cos_theta) >> 15);
}

/**
 * @brief Inverse Park transform (rotating to stationary frame)
 */
static inline void inv_park_transform(q15_t d, q15_t q,
                                       q15_t sin_theta, q15_t cos_theta,
                                       q15_t *alpha, q15_t *beta) {
    *alpha = (q15_t)(((int32_t)d * cos_theta - (int32_t)q * sin_theta) >> 15);
    *beta = (q15_t)(((int32_t)d * sin_theta + (int32_t)q * cos_theta) >> 15);
}

#ifdef __cplusplus
}
#endif

#endif /* FIXED_POINT_H */
