/**
 * @file foc_angle_fusion.h
 * @brief Angle Fusion between Hall Sensor and Observer
 * @author FOC Team
 * @date 2024
 * 
 * This module implements smooth transition between Hall sensor and
 * observer-based angle estimation, following VESC's hybrid sensor mode.
 * 
 * Strategy:
 * - Low speed (< foc_sl_erpm_start): Use Hall sensor only
 * - Transition region: Weighted blend between Hall and Observer
 * - High speed (> foc_sl_erpm): Use Observer only
 * 
 * The transition is designed to be seamless with hysteresis to prevent
 * oscillation between modes.
 */

#ifndef FOC_ANGLE_FUSION_H_
#define FOC_ANGLE_FUSION_H_

#include "types.h"
#include "foc_observer.h"
#include "foc_hall.h"

/*============================================================================*/
/*                          CONSTANTS                                         */
/*============================================================================*/

/* Hysteresis for mode transitions (as fraction of transition range) */
#define FUSION_HYSTERESIS_FRAC    (q15_t)(Q15_ONE / 10)   /* 10% hysteresis */

/*============================================================================*/
/*                          STRUCTURES                                        */
/*============================================================================*/

/**
 * @brief Angle fusion configuration
 */
typedef struct {
    /* Transition speeds (electrical RPM) */
    q12_4_t hall_only_rpm;          /* Below this: Hall only */
    q12_4_t transition_start_rpm;   /* Start blending */
    q12_4_t transition_end_rpm;     /* Observer dominant */
    q12_4_t observer_only_rpm;      /* Above this: Observer only */
    
    /* Blend rates */
    q15_t blend_rate_up;            /* Rate to increase observer weight */
    q15_t blend_rate_down;          /* Rate to decrease observer weight */
    
    /* Hall interpolation */
    q12_4_t hall_interp_min_rpm;    /* Below this: no Hall interpolation */
    
    /* Fault detection */
    q15_t angle_diff_fault_threshold;  /* Max allowed Hall-Observer difference */
    uint16_t fault_count_threshold;    /* Cycles before fault */
    
} angle_fusion_config_t;

/**
 * @brief Angle fusion runtime state
 */
typedef struct {
    /* Current angle sources */
    q16_16_t angle_hall;            /* Hall sensor angle */
    q16_16_t angle_observer;        /* Observer angle */
    q16_16_t angle_fused;           /* Fused output angle */
    
    /* Blending */
    q15_t observer_weight;          /* 0 = Hall only, Q15_ONE = Observer only */
    q15_t observer_weight_target;   /* Target weight for smooth transition */
    
    /* Speed input */
    q12_4_t speed_rpm;              /* Current speed (electrical RPM) */
    q12_4_t speed_rpm_abs;          /* Absolute speed */
    
    /* Mode tracking */
    angle_source_t active_source;   /* Currently dominant source */
    angle_source_t previous_source; /* Previous dominant source */
    
    /* Diagnostic */
    q15_t angle_difference;         /* Difference between Hall and Observer */
    bool transition_active;         /* In transition region */
    uint16_t fault_counter;         /* For detecting Hall/Observer mismatch */
    bool fault_detected;            /* Angle difference too large */
    
    /* Hysteresis state */
    bool was_using_hall;            /* Previous cycle used Hall */
    bool was_using_observer;        /* Previous cycle used Observer */
    
} angle_fusion_state_t;

/*============================================================================*/
/*                          FUNCTION DECLARATIONS                             */
/*============================================================================*/

/**
 * @brief Initialize angle fusion module
 */
void angle_fusion_init(angle_fusion_state_t *state, 
                        const angle_fusion_config_t *config);

/**
 * @brief Update angle fusion
 * 
 * @param state Fusion runtime state
 * @param config Fusion configuration
 * @param hall_state Hall sensor state
 * @param observer_state Observer state
 * @param speed_rpm Current electrical speed in RPM
 * @param dt Time step in seconds (Q16.16)
 * @return Fused angle in Q16.16 radians
 */
q16_16_t angle_fusion_update(angle_fusion_state_t *state,
                               const angle_fusion_config_t *config,
                               const hall_state_t *hall_state,
                               const observer_runtime_t *observer_state,
                               q12_4_t speed_rpm,
                               q16_16_t dt);

/**
 * @brief Get the fused angle
 */
static inline q16_16_t angle_fusion_get_angle(const angle_fusion_state_t *state) {
    return state->angle_fused;
}

/**
 * @brief Get the current dominant angle source
 */
static inline angle_source_t angle_fusion_get_source(const angle_fusion_state_t *state) {
    return state->active_source;
}

/**
 * @brief Get the current observer weight (for debugging)
 */
static inline q15_t angle_fusion_get_observer_weight(const angle_fusion_state_t *state) {
    return state->observer_weight;
}

/**
 * @brief Check if transition is active
 */
static inline bool angle_fusion_in_transition(const angle_fusion_state_t *state) {
    return state->transition_active;
}

/**
 * @brief Check if there's a Hall/Observer mismatch fault
 */
static inline bool angle_fusion_has_fault(const angle_fusion_state_t *state) {
    return state->fault_detected;
}

/**
 * @brief Reset fusion state
 */
void angle_fusion_reset(angle_fusion_state_t *state);

/**
 * @brief Force Hall mode (for low speed or startup)
 */
void angle_fusion_force_hall(angle_fusion_state_t *state);

/**
 * @brief Force Observer mode (for high speed)
 */
void angle_fusion_force_observer(angle_fusion_state_t *state);

/**
 * @brief Interpolate between two angles
 * 
 * @param angle1 First angle (Q16.16 radians)
 * @param angle2 Second angle (Q16.16 radians)
 * @param weight Weight for angle2 (Q15, 0 = angle1, Q15_ONE = angle2)
 * @return Interpolated angle (Q16.16 radians)
 */
q16_16_t angle_interpolate(q16_16_t angle1, q16_16_t angle2, q15_t weight);

/**
 * @brief Calculate weighted blend based on speed
 * 
 * This implements VESC-style speed-based blending with hysteresis.
 * 
 * @param state Fusion state
 * @param config Fusion configuration
 * @param speed_rpm_abs Absolute speed in RPM
 */
void angle_fusion_update_weight(angle_fusion_state_t *state,
                                  const angle_fusion_config_t *config,
                                  q12_4_t speed_rpm_abs);

#endif /* FOC_ANGLE_FUSION_H_ */
