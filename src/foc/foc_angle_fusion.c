/**
 * @file foc_angle_fusion.c
 * @brief Angle Fusion Implementation - Hall and Observer Blending
 * @author FOC Team
 * @date 2024
 * 
 * Implements smooth transition between Hall sensor and observer-based
 * angle estimation following VESC's hybrid sensor mode architecture.
 */

#include "foc_angle_fusion.h"
#include <string.h>

/*============================================================================*/
/*                          INTERNAL FUNCTIONS                               */
/*============================================================================*/

/**
 * @brief Calculate angle difference (shortest path)
 * @return Difference in Q16.16 radians, normalized to [-π, π]
 */
static q15_t angle_difference_q15(q16_16_t angle1, q16_16_t angle2) {
    /* Convert to Q15 for easier math */
    q15_t a1 = (q15_t)(angle1 >> 1);  /* Q16.16 -> Q15 approximate */
    q15_t a2 = (q15_t)(angle2 >> 1);
    
    q15_t diff = a1 - a2;
    
    /* Normalize to [-π, π) in Q15 (π ≈ 10294) */
    const q15_t pi_q15 = 10294;
    const q15_t two_pi_q15 = 20588;
    
    while (diff > pi_q15) diff -= two_pi_q15;
    while (diff < -pi_q15) diff += two_pi_q15;
    
    return diff;
}

/**
 * @brief Step a value towards a target with rate limiting
 */
static q15_t step_towards(q15_t current, q15_t target, q15_t step_size) {
    if (current < target) {
        current += step_size;
        if (current > target) current = target;
    } else if (current > target) {
        current -= step_size;
        if (current < target) current = target;
    }
    return current;
}

/*============================================================================*/
/*                          PUBLIC FUNCTIONS                                  */
/*============================================================================*/

/**
 * @brief Initialize angle fusion module
 */
void angle_fusion_init(angle_fusion_state_t *state, 
                        const angle_fusion_config_t *config) {
    memset(state, 0, sizeof(angle_fusion_state_t));
    
    /* Start with Hall sensor (safer for startup) */
    state->observer_weight = 0;
    state->observer_weight_target = 0;
    state->active_source = ANGLE_SOURCE_HALL;
    state->previous_source = ANGLE_SOURCE_HALL;
    state->was_using_hall = true;
    state->was_using_observer = false;
}

/**
 * @brief Reset fusion state
 */
void angle_fusion_reset(angle_fusion_state_t *state) {
    state->observer_weight = 0;
    state->observer_weight_target = 0;
    state->active_source = ANGLE_SOURCE_HALL;
    state->transition_active = false;
    state->fault_counter = 0;
    state->fault_detected = false;
    state->was_using_hall = true;
    state->was_using_observer = false;
}

/**
 * @brief Force Hall mode
 */
void angle_fusion_force_hall(angle_fusion_state_t *state) {
    state->observer_weight_target = 0;
    state->active_source = ANGLE_SOURCE_HALL;
}

/**
 * @brief Force Observer mode
 */
void angle_fusion_force_observer(angle_fusion_state_t *state) {
    state->observer_weight_target = Q15_ONE;
    state->active_source = ANGLE_SOURCE_OBSERVER;
}

/**
 * @brief Update weight based on speed with hysteresis
 * 
 * This implements the VESC-style speed-based transition with hysteresis:
 * 
 * Speed increasing:
 *   - Below hall_only_rpm: weight = 0
 *   - Between transition_start and transition_end: weight ramps up
 *   - Above observer_only_rpm: weight = Q15_ONE
 * 
 * Speed decreasing:
 *   - Uses slightly different thresholds (hysteresis) to prevent oscillation
 */
void angle_fusion_update_weight(angle_fusion_state_t *state,
                                  const angle_fusion_config_t *config,
                                  q12_4_t speed_rpm_abs) {
    /* Determine target weight based on speed */
    q15_t target_weight;
    
    if (speed_rpm_abs <= config->hall_only_rpm) {
        /* Low speed: Hall only */
        target_weight = 0;
        state->active_source = ANGLE_SOURCE_HALL;
        state->transition_active = false;
        
    } else if (speed_rpm_abs >= config->observer_only_rpm) {
        /* High speed: Observer only */
        target_weight = Q15_ONE;
        state->active_source = ANGLE_SOURCE_OBSERVER;
        state->transition_active = false;
        
    } else if (speed_rpm_abs >= config->transition_end_rpm) {
        /* Approaching observer region */
        /* Weight approaches Q15_ONE */
        q12_4_t range = config->observer_only_rpm - config->transition_end_rpm;
        q12_4_t pos = speed_rpm_abs - config->transition_end_rpm;
        target_weight = (q15_t)((Q15_ONE * 8 / 10) + 
                                 ((int32_t)pos * Q15_ONE / 5) / range);
        state->active_source = ANGLE_SOURCE_FUSION;
        state->transition_active = true;
        
    } else if (speed_rpm_abs >= config->transition_start_rpm) {
        /* Transition region: blend */
        q12_4_t range = config->transition_end_rpm - config->transition_start_rpm;
        q12_4_t pos = speed_rpm_abs - config->transition_start_rpm;
        target_weight = (q15_t)(((int32_t)pos * Q15_ONE * 8 / 10) / range);
        state->active_source = ANGLE_SOURCE_FUSION;
        state->transition_active = true;
        
    } else {
        /* Below transition start but above hall_only */
        /* Small observer contribution */
        q12_4_t range = config->transition_start_rpm - config->hall_only_rpm;
        q12_4_t pos = speed_rpm_abs - config->hall_only_rpm;
        target_weight = (q15_t)(((int32_t)pos * Q15_ONE / 10) / range);
        state->active_source = ANGLE_SOURCE_HALL;
        state->transition_active = false;
    }
    
    /* Apply hysteresis */
    /* When transitioning from Hall to Observer, require higher speed */
    /* When transitioning from Observer to Hall, allow at lower speed */
    
    if (state->was_using_hall && target_weight > state->observer_weight) {
        /* Speeding up: add hysteresis delay */
        q15_t hysteresis = FUSION_HYSTERESIS_FRAC;
        target_weight = step_towards(state->observer_weight, target_weight, hysteresis);
    } else if (state->was_using_observer && target_weight < state->observer_weight) {
        /* Slowing down: add hysteresis delay */
        q15_t hysteresis = FUSION_HYSTERESIS_FRAC;
        target_weight = step_towards(state->observer_weight, target_weight, hysteresis);
    }
    
    /* Store target */
    state->observer_weight_target = target_weight;
    
    /* Update direction flags */
    if (state->observer_weight < Q15_ONE / 4) {
        state->was_using_hall = true;
        state->was_using_observer = false;
    } else if (state->observer_weight > Q15_ONE * 3 / 4) {
        state->was_using_hall = false;
        state->was_using_observer = true;
    }
}

/**
 * @brief Interpolate between two angles
 * 
 * Uses spherical linear interpolation to smoothly blend angles.
 */
q16_16_t angle_interpolate(q16_16_t angle1, q16_16_t angle2, q15_t weight) {
    if (weight <= 0) return angle1;
    if (weight >= Q15_ONE) return angle2;
    
    /* Calculate difference */
    q31_t diff = (q31_t)angle2 - angle1;
    
    /* Normalize to [-π, π) */
    const q31_t pi_q16 = 205888;
    const q31_t two_pi_q16 = 411775;
    
    while (diff > pi_q16) diff -= two_pi_q16;
    while (diff < -pi_q16) diff += two_pi_q16;
    
    /* Interpolate: angle = angle1 + weight * diff */
    q31_t interp = angle1 + ((diff * weight) >> 15);
    
    /* Normalize result */
    while (interp >= two_pi_q16) interp -= two_pi_q16;
    while (interp < 0) interp += two_pi_q16;
    
    return (q16_16_t)interp;
}

/**
 * @brief Main fusion update function
 */
q16_16_t angle_fusion_update(angle_fusion_state_t *state,
                               const angle_fusion_config_t *config,
                               const hall_state_t *hall_state,
                               const observer_runtime_t *observer_state,
                               q12_4_t speed_rpm,
                               q16_16_t dt) {
    /* Store input angles */
    state->angle_hall = hall_state->angle;
    state->angle_observer = observer_state->angle_filtered;
    state->speed_rpm = speed_rpm;
    state->speed_rpm_abs = (speed_rpm >= 0) ? speed_rpm : -speed_rpm;
    
    /* Calculate angle difference for diagnostics */
    state->angle_difference = angle_difference_q15(state->angle_observer, 
                                                     state->angle_hall);
    
    /* Check for Hall/Observer mismatch fault */
    q15_t angle_diff_abs = q15_abs(state->angle_difference);
    
    if (angle_diff_abs > config->angle_diff_fault_threshold) {
        state->fault_counter++;
        if (state->fault_counter > config->fault_count_threshold) {
            state->fault_detected = true;
        }
    } else {
        if (state->fault_counter > 0) {
            state->fault_counter--;
        }
        state->fault_detected = false;
    }
    
    /* Update blend weight based on speed */
    angle_fusion_update_weight(state, config, state->speed_rpm_abs);
    
    /* Rate-limit the weight change for smooth transitions */
    /* Calculate maximum step per cycle */
    q15_t max_step;
    if (dt > 0) {
        /* max_step = blend_rate * dt */
        /* blend_rate is in Q15 per second, dt is Q16.16 seconds */
        max_step = (q15_t)(((int32_t)config->blend_rate_up * dt) >> 16);
        if (max_step < 1) max_step = 1;
    } else {
        max_step = 1;
    }
    
    /* Step weight towards target */
    if (state->observer_weight_target > state->observer_weight) {
        state->observer_weight += max_step;
        if (state->observer_weight > state->observer_weight_target) {
            state->observer_weight = state->observer_weight_target;
        }
    } else if (state->observer_weight_target < state->observer_weight) {
        q15_t step_down = (q15_t)(((int32_t)config->blend_rate_down * dt) >> 16);
        if (step_down < 1) step_down = 1;
        state->observer_weight -= step_down;
        if (state->observer_weight < state->observer_weight_target) {
            state->observer_weight = state->observer_weight_target;
        }
    }
    
    /* Fuse angles */
    if (!hall_state->is_valid) {
        /* Hall sensor fault: use observer regardless of weight */
        state->angle_fused = state->angle_observer;
        state->active_source = ANGLE_SOURCE_OBSERVER;
    } else if (!observer_is_tracking(observer_state)) {
        /* Observer not tracking: use Hall regardless of weight */
        state->angle_fused = state->angle_hall;
        state->active_source = ANGLE_SOURCE_HALL;
    } else if (state->observer_weight <= 0) {
        /* Hall only */
        state->angle_fused = state->angle_hall;
    } else if (state->observer_weight >= Q15_ONE) {
        /* Observer only */
        state->angle_fused = state->angle_observer;
    } else {
        /* Blend between Hall and Observer */
        state->angle_fused = angle_interpolate(state->angle_hall, 
                                                state->angle_observer,
                                                state->observer_weight);
        state->active_source = ANGLE_SOURCE_FUSION;
    }
    
    /* Update previous source */
    state->previous_source = state->active_source;
    
    return state->angle_fused;
}
