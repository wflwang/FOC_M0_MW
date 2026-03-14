/**
 * @file foc_svpwm.c
 * @brief SVPWM Generation Implementation
 */

#include "foc_svpwm.h"
#include "fixed_point.h"

/*============================================================================*/
/* SVPWM Calculation                                                          */
/*============================================================================*/

void svpwm_calculate(q15_t v_alpha, q15_t v_beta, q15_t v_bus, uint16_t duty[3]) {
    // Calculate modulation index
    // m = sqrt(v_alpha^2 + v_beta^2) / (v_bus * 2/3)
    
    // For simplicity, use direct alpha/beta to duty mapping
    // This is a simplified SVPWM implementation
    
    // Calculate reference voltages for each phase
    q15_t va = v_alpha;
    q15_t vb = (q15_t)((-(int32_t)v_alpha + 37838 * (int32_t)v_beta) >> 15);  // -0.5*alpha + sqrt(3)/2*beta
    q15_t vc = (q15_t)((-(int32_t)v_alpha - 37838 * (int32_t)v_beta) >> 15);  // -0.5*alpha - sqrt(3)/2*beta
    
    // Find min and max for neutral shift (maximizes DC bus utilization)
    q15_t v_min = va;
    q15_t v_max = va;
    if (vb < v_min) v_min = vb;
    if (vb > v_max) v_max = vb;
    if (vc < v_min) v_min = vc;
    if (vc > v_max) v_max = vc;
    
    // Neutral shift for SVPWM
    q15_t v_neutral = -(v_min + v_max) / 2;
    
    va += v_neutral;
    vb += v_neutral;
    vc += v_neutral;
    
    // Convert to duty cycles (0-65535)
    // Assuming v_bus normalized and voltage in Q15
    duty[0] = (uint16_t)(32768 + va);
    duty[1] = (uint16_t)(32768 + vb);
    duty[2] = (uint16_t)(32768 + vc);
    
    // Clamp duty cycles
    if (duty[0] > 63000) duty[0] = 63000;
    if (duty[0] < 2500) duty[0] = 2500;
    if (duty[1] > 63000) duty[1] = 63000;
    if (duty[1] < 2500) duty[1] = 2500;
    if (duty[2] > 63000) duty[2] = 63000;
    if (duty[2] < 2500) duty[2] = 2500;
}

uint8_t svpwm_get_sector(q15_t v_alpha, q15_t v_beta) {
    // Determine sector based on angle
    q15_t angle = cordic_atan2(v_beta, v_alpha);
    
    // Each sector is 60° (10923 in Q15)
    if (angle < 10923) return 1;
    if (angle < 21845) return 2;
    if (angle < 32768) return 3;
    if (angle < 43691) return 4;
    if (angle < 54613) return 5;
    return 6;
}
