/**
 * @file fixed_point.c
 * @brief Fixed Point Math Library Implementation
 */

#include "fixed_point.h"

/*============================================================================*/
/* Sin/Cos Lookup Table                                                       */
/*============================================================================*/

// 256-entry sin table for Q15 (0 to π/2, scaled to 0-32767)
static const q15_t sin_table[65] = {
    0, 804, 1608, 2412, 3216, 4019, 4821, 5623,
    6424, 7224, 8022, 8820, 9616, 10411, 11204, 11996,
    12785, 13573, 14359, 15142, 15924, 16703, 17479, 18253,
    19024, 19792, 20557, 21320, 22078, 22834, 23586, 24335,
    25080, 25821, 26558, 27291, 28020, 28745, 29466, 30182,
    30893, 31600, 32303, 33000, 33692, 34380, 35062, 35738,
    36410, 37076, 37737, 38391, 39040, 39683, 40320, 40951,
    41576, 42194, 42806, 43412, 44011, 44604, 45190, 45769,
    32767
};

void foc_sincos(q15_t angle, q15_t *sin_out, q15_t *cos_out) {
    // Normalize angle to 0-65535
    uint16_t a = (uint16_t)angle;
    
    // Determine quadrant
    uint16_t quadrant = (a >> 14) & 0x03;
    uint16_t idx_full = (a >> 8) & 0x3F;  // 0-63 index
    uint16_t frac = a & 0xFF;             // Interpolation fraction
    
    q15_t s0, s1, val;
    
    switch (quadrant) {
        case 0:  // 0 to π/2
            s0 = sin_table[idx_full];
            s1 = sin_table[idx_full + 1];
            val = s0 + (q15_t)(((int32_t)(s1 - s0) * frac) >> 8);
            *sin_out = val;
            *cos_out = sin_table[64 - idx_full];
            break;
        case 1:  // π/2 to π
            s0 = sin_table[64 - idx_full];
            s1 = sin_table[63 - idx_full];
            val = s0 + (q15_t)(((int32_t)(s1 - s0) * frac) >> 8);
            *sin_out = val;
            *cos_out = -sin_table[idx_full];
            break;
        case 2:  // π to 3π/2
            s0 = sin_table[idx_full];
            s1 = sin_table[idx_full + 1];
            val = s0 + (q15_t)(((int32_t)(s1 - s0) * frac) >> 8);
            *sin_out = -val;
            *cos_out = -sin_table[64 - idx_full];
            break;
        case 3:  // 3π/2 to 2π
            s0 = sin_table[64 - idx_full];
            s1 = sin_table[63 - idx_full];
            val = s0 + (q15_t)(((int32_t)(s1 - s0) * frac) >> 8);
            *sin_out = -val;
            *cos_out = sin_table[idx_full];
            break;
    }
}

q15_t foc_sin(q15_t angle) {
    q15_t s, c;
    foc_sincos(angle, &s, &c);
    return s;
}

q15_t foc_cos(q15_t angle) {
    return foc_sin((q15_t)(angle + 16384));
}

/*============================================================================*/
/* CORDIC Atan2                                                               */
/*============================================================================*/

static const q15_t cordic_atan[15] = {
    8192, 4836, 2555, 1297, 651, 326, 163, 81, 41, 20, 10, 5, 3, 1, 1
};

q15_t cordic_atan2(q15_t y, q15_t x) {
    int32_t z = 0;
    int32_t x_tmp = x;
    int32_t y_tmp = y;
    
    if (x == 0 && y == 0) return 0;
    
    // Determine quadrant and adjust
    int sign_x = (x_tmp >= 0) ? 1 : -1;
    int sign_y = (y_tmp >= 0) ? 1 : -1;
    x_tmp = ABS(x_tmp);
    y_tmp = ABS(y_tmp);
    
    // CORDIC iterations
    for (int i = 0; i < 15; i++) {
        int32_t x_new, y_new;
        
        if (y_tmp >= 0) {
            x_new = x_tmp + (y_tmp >> i);
            y_new = y_tmp - (x_tmp >> i);
            z += cordic_atan[i];
        } else {
            x_new = x_tmp - (y_tmp >> i);
            y_new = y_tmp + (x_tmp >> i);
            z -= cordic_atan[i];
        }
        
        x_tmp = x_new;
        y_tmp = y_new;
    }
    
    // Quadrant adjustment
    if (sign_x < 0) {
        if (sign_y >= 0) z = 32768 - z;
        else z = -32768 - z;
    } else {
        if (sign_y < 0) z = -z;
    }
    
    return (q15_t)z;
}
