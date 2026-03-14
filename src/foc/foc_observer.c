/**
 * @file foc_observer.c
 * @brief MXLemming Observer Implementation (Simplified for M0)
 */

#include "foc_observer.h"
#include "fixed_point.h"

/*============================================================================*/
/* Private Variables                                                          */
/*============================================================================*/

static const motor_config_t *g_motor;

// Observer state
static q15_t psi_alpha = 0;
static q15_t psi_beta = 0;
static q15_t omega_est = 0;     // Estimated speed
static q15_t theta_est = 0;     // Estimated angle

// PLL state
static q31_t pll_integral = 0;

/*============================================================================*/
/* Implementation                                                             */
/*============================================================================*/

void observer_init(const motor_config_t *config) {
    g_motor = config;
    observer_reset();
}

void observer_reset(void) {
    psi_alpha = 0;
    psi_beta = 0;
    omega_est = 0;
    theta_est = 0;
    pll_integral = 0;
}

void observer_update(q15_t id, q15_t iq, q15_t v_bus) {
    // Simplified MXLemming observer for M0
    // Uses flux linkage integration
    
    // Get current angle estimate
    q15_t sin_theta, cos_theta;
    foc_sincos(theta_est, &sin_theta, &cos_theta);
    
    // Calculate expected flux linkage
    // psi = L*i + psi_pm
    // For simplicity, use constant flux linkage assumption
    
    // Flux observer (simplified)
    // d(psi)/dt = v - R*i
    // Using trapezoidal integration
    
    // Back-EMF observer
    static q15_t e_alpha = 0, e_beta = 0;
    
    // Estimate back-EMF from voltage equation
    // e = v - R*i - L*di/dt
    // Simplified: e ≈ v_bus * duty - R * i
    
    // Low-pass filter the back-EMF
    static q15_t e_alpha_lpf = 0, e_beta_lpf = 0;
    
    e_alpha_lpf = (q15_t)((15 * (int32_t)e_alpha_lpf + e_alpha) >> 4);
    e_beta_lpf = (q15_t)((15 * (int32_t)e_beta_lpf + e_beta) >> 4);
    
    // Update flux estimate
    // psi = integral(e)
    psi_alpha += (q15_t)((int32_t)e_alpha_lpf >> 8);
    psi_beta += (q15_t)((int32_t)e_beta_lpf >> 8);
    
    // Normalize flux
    int32_t psi_mag_sq = (int32_t)psi_alpha * psi_alpha + (int32_t)psi_beta * psi_beta;
    q15_t psi_mag = (q15_t)sqrt_approx(psi_mag_sq);
    
    if (psi_mag > 100) {
        // Calculate angle from flux
        q15_t theta_flux = cordic_atan2(psi_beta, psi_alpha);
        
        // PLL for speed estimation
        q15_t theta_error = theta_flux - theta_est;
        
        // Wrap error to -π to +π
        if (theta_error > 16384) theta_error -= 32768;
        if (theta_error < -16384) theta_error += 32768;
        
        // PLL update (simple PI)
        pll_integral += (q31_t)theta_error * 32;  // Ki
        q15_t omega_new = theta_error * 8 + (q15_t)(pll_integral >> 15);  // Kp + Ki
        
        // Low-pass filter speed estimate
        omega_est = (q15_t)((7 * (int32_t)omega_est + omega_new) >> 3);
        
        // Update angle estimate
        theta_est += omega_est;
    }
    
    // Integrate speed to angle
    theta_est += omega_est;
    
    (void)id;
    (void)iq;
    (void)v_bus;
}

q15_t observer_get_angle(void) {
    return theta_est;
}

q15_t observer_get_speed(void) {
    return omega_est;
}

/*============================================================================*/
/* Helper Functions                                                           */
/*============================================================================*/

static int32_t sqrt_approx(int32_t x) {
    // Fast integer square root approximation
    if (x <= 0) return 0;
    
    int32_t result = 0;
    int32_t bit = 1 << 30;
    
    while (bit > x) bit >>= 2;
    
    while (bit != 0) {
        if (x >= result + bit) {
            x -= result + bit;
            result = (result >> 1) + bit;
        } else {
            result >>= 1;
        }
        bit >>= 2;
    }
    
    return result;
}
