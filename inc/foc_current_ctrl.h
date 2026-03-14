/**
 * @file foc_current_ctrl.h
 * @brief FOC Current Controller with Decoupling
 * @author FOC Team
 * @date 2024
 * 
 * This module implements the current control loop for FOC, including:
 * - PI current controllers for Id and Iq
 * - Cross-coupling decoupling
 * - Back-EMF feedforward compensation
 * - Anti-windup and voltage saturation
 */

#ifndef FOC_CURRENT_CTRL_H_
#define FOC_CURRENT_CTRL_H_

#include "types.h"
#include "fixed_point.h"

/*============================================================================*/
/*                          CONSTANTS                                         */
/*============================================================================*/

/* Decoupling modes */
typedef enum {
    DECOUPLE_NONE = 0,          /* No decoupling */
    DECOUPLE_CROSS,             /* Cross-coupling compensation */
    DECOUPLE_BEMF,              /* Back-EMF feedforward */
    DECOUPLE_CROSS_BEMF         /* Both cross and BEMF */
} decoupling_mode_t;

/*============================================================================*/
/*                          STRUCTURES                                        */
/*============================================================================*/

/**
 * @brief Current controller configuration
 */
typedef struct {
    /* PI gains */
    q15_t kp;                    /* Proportional gain (Q15) */
    q31_t ki;                    /* Integral gain scaled by dt (Q31) */
    
    /* Current limits */
    q8_8_t current_max;          /* Maximum current (A) */
    q8_8_t current_min;          /* Minimum current (A) */
    
    /* Voltage limits */
    q15_t duty_max;              /* Maximum duty cycle (Q15) */
    
    /* Decoupling */
    decoupling_mode_t decouple_mode;
    q12_20_t inductance_d;       /* D-axis inductance (H) */
    q12_20_t inductance_q;       /* Q-axis inductance (H) */
    q12_20_t flux_linkage;       /* Flux linkage (Wb) */
    
    /* Anti-windup */
    q15_t integral_max;          /* Maximum integral term */
    
    /* Filter */
    q15_t current_filter_coeff;  /* LPF coefficient for current */
    
} current_ctrl_config_t;

/**
 * @brief Current controller runtime state
 */
typedef struct {
    /* Current references */
    q8_8_t id_ref;               /* D-axis current reference (A) */
    q8_8_t iq_ref;               /* Q-axis current reference (A) */
    
    /* Measured currents */
    q8_8_t id;                   /* D-axis measured current */
    q8_8_t iq;                   /* Q-axis measured current */
    q8_8_t id_filtered;          /* Filtered D-axis current */
    q8_8_t iq_filtered;          /* Filtered Q-axis current */
    
    /* Current errors */
    q8_8_t id_error;             /* D-axis error */
    q8_8_t iq_error;             /* Q-axis error */
    
    /* PI outputs */
    q15_t vd;                    /* D-axis voltage (normalized) */
    q15_t vq;                    /* Q-axis voltage (normalized) */
    q31_t vd_integral;           /* D-axis integral term */
    q31_t vq_integral;           /* Q-axis integral term */
    
    /* Decoupling terms */
    q15_t dec_vd;                /* D-axis decoupling voltage */
    q15_t dec_vq;                /* Q-axis decoupling voltage */
    q15_t bemf_comp;             /* Back-EMF compensation */
    
    /* Modulation output */
    q15_t mod_d;                 /* D-axis modulation index */
    q15_t mod_q;                 /* Q-axis modulation index */
    q15_t mod_alpha;             /* α-axis modulation */
    q15_t mod_beta;              /* β-axis modulation */
    
    /* Bus values */
    q8_8_t v_bus;                /* Bus voltage */
    q15_t duty_now;              /* Current duty cycle */
    
    /* Saturation flags */
    bool vd_saturated;           /* D-axis voltage saturated */
    bool vq_saturated;           /* Q-axis voltage saturated */
    
} current_ctrl_state_t;

/*============================================================================*/
/*                          FUNCTION DECLARATIONS                             */
/*============================================================================*/

/**
 * @brief Initialize current controller
 */
void current_ctrl_init(current_ctrl_state_t *state, 
                        const current_ctrl_config_t *config);

/**
 * @brief Reset current controller state
 */
void current_ctrl_reset(current_ctrl_state_t *state);

/**
 * @brief Run current control loop
 * 
 * @param state Current controller state
 * @param config Current controller configuration
 * @param id_ref D-axis current reference (Q8.8 A)
 * @param iq_ref Q-axis current reference (Q8.8 A)
 * @param i_alpha α-axis measured current (Q8.8 A)
 * @param i_beta β-axis measured current (Q8.8 A)
 * @param angle Rotor angle (Q16.16 rad)
 * @param speed_e Electrical speed (Q16.16 rad/s)
 * @param v_bus Bus voltage (Q8.8 V)
 * @param dt Time step (Q16.16 s)
 * @return Modulation alpha (for SVPWM)
 */
q15_t current_ctrl_update(current_ctrl_state_t *state,
                            const current_ctrl_config_t *config,
                            q8_8_t id_ref, q8_8_t iq_ref,
                            q8_8_t i_alpha, q8_8_t i_beta,
                            q16_16_t angle, q31_t speed_e,
                            q8_8_t v_bus, q16_16_t dt);

/**
 * @brief Set current references
 */
void current_ctrl_set_refs(current_ctrl_state_t *state,
                            q8_8_t id_ref, q8_8_t iq_ref);

/**
 * @brief Get modulation outputs
 */
void current_ctrl_get_modulation(const current_ctrl_state_t *state,
                                  q15_t *mod_alpha, q15_t *mod_beta);

/**
 * @brief Get measured currents in dq frame
 */
void current_ctrl_get_dq_currents(const current_ctrl_state_t *state,
                                   q8_8_t *id, q8_8_t *iq);

/**
 * @brief Calculate PI gains from motor parameters
 * 
 * Uses the formula:
 *   Kp = L * bandwidth
 *   Ki = R * bandwidth
 * 
 * @param resistance Phase resistance (Ohm)
 * @param inductance Phase inductance (H)
 * @param bandwidth Control bandwidth (Hz)
 * @param kp_out Output proportional gain
 * @param ki_out Output integral gain
 */
void current_ctrl_calc_gains(float resistance, float inductance, 
                              float bandwidth, float sample_freq,
                              q15_t *kp_out, q31_t *ki_out);

/**
 * @brief Apply field weakening current
 * 
 * Adjusts Id reference based on duty cycle to extend speed range.
 * 
 * @param state Current controller state
 * @param config Configuration
 * @param duty Current duty cycle
 * @param duty_start Duty cycle to start FW
 * @param fw_current_max Maximum FW current
 */
void current_ctrl_apply_fw(current_ctrl_state_t *state,
                            const current_ctrl_config_t *config,
                            q15_t duty, q15_t duty_start,
                            q8_8_t fw_current_max);

#endif /* FOC_CURRENT_CTRL_H_ */
