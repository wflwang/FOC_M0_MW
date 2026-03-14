/**
 * @file foc_svpwm.h
 * @brief SVPWM Generation Header
 */

#ifndef FOC_SVPWM_H
#define FOC_SVPWM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "types.h"

/**
 * @brief Calculate SVPWM duty cycles
 * @param v_alpha Alpha voltage (Q15)
 * @param v_beta Beta voltage (Q15)
 * @param v_bus Bus voltage (Q8.8)
 * @param duty Output duty cycles [3] (0-65535)
 */
void svpwm_calculate(q15_t v_alpha, q15_t v_beta, q15_t v_bus, uint16_t duty[3]);

/**
 * @brief Get current sector
 */
uint8_t svpwm_get_sector(q15_t v_alpha, q15_t v_beta);

#ifdef __cplusplus
}
#endif

#endif /* FOC_SVPWM_H */
