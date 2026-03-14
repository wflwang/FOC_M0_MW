/**
 * @file foc_motor_detect.h
 * @brief Motor Parameter Automatic Detection (VESC Style)
 * @author FOC Team
 * @date 2024
 * 
 * This module implements automatic detection of motor parameters:
 * - Phase resistance (R)
 * - Phase inductance (L) and Ld-Lq difference
 * - Flux linkage (λ)
 * - Hall sensor table
 * - Encoder offset
 * 
 * These parameters are essential for proper FOC operation and observer
 * performance. The detection algorithms follow VESC's methodology.
 */

#ifndef FOC_MOTOR_DETECT_H_
#define FOC_MOTOR_DETECT_H_

#include "types.h"
#include "fixed_point.h"

/*============================================================================*/
/*                          CONSTANTS                                         */
/*============================================================================*/

/* Detection states */
typedef enum {
    DETECT_STATE_IDLE = 0,
    DETECT_STATE_MEASURE_R,
    DETECT_STATE_MEASURE_L,
    DETECT_STATE_MEASURE_FLUX,
    DETECT_STATE_MEASURE_HALL,
    DETECT_STATE_CALIBRATE,
    DETECT_STATE_COMPLETE,
    DETECT_STATE_FAILED
} detect_state_t;

/* Detection error codes */
typedef enum {
    DETECT_OK = 0,
    DETECT_ERR_NO_MOTOR,           /* Motor not connected */
    DETECT_ERR_LOW_RESISTANCE,     /* Resistance too low */
    DETECT_ERR_HIGH_RESISTANCE,    /* Resistance too high */
    DETECT_ERR_LOW_INDUCTANCE,     /* Inductance too low */
    DETECT_ERR_HIGH_INDUCTANCE,    /* Inductance too high */
    DETECT_ERR_FLUX_LINKAGE,       /* Flux linkage detection failed */
    DETECT_ERR_HALL_SENSOR,        /* Hall sensor detection failed */
    DETECT_ERR_TIMEOUT,            /* Detection timeout */
    DETECT_ERR_OVERCURRENT,        /* Overcurrent during detection */
    DETECT_ERR_UNDERVOLTAGE        /* Bus voltage too low */
} detect_error_t;

/* Detection configuration limits */
#define DETECT_R_MIN            (q8_8_t)(0.001 * 256)     /* 1 mOhm */
#define DETECT_R_MAX            (q8_8_t)(100.0 * 256)     /* 100 Ohm */
#define DETECT_L_MIN            (q12_20_t)(0.5e-6 * 1048576)   /* 0.5 µH */
#define DETECT_L_MAX            (q12_20_t)(100e-3 * 1048576)   /* 100 mH */
#define DETECT_FLUX_MIN         (q12_20_t)(0.001 * 1048576)    /* 0.001 Wb */
#define DETECT_FLUX_MAX         (q12_20_t)(0.5 * 1048576)      /* 0.5 Wb */

/*============================================================================*/
/*                          STRUCTURES                                        */
/*============================================================================*/

/**
 * @brief Detection configuration
 */
typedef struct {
    /* Test currents (Q8.8 Amperes) */
    q8_8_t resistance_current;     /* Current for R measurement */
    q8_8_t inductance_current;     /* Current for L measurement */
    q8_8_t flux_current;           /* Current for flux measurement */
    
    /* Test voltages (Q8.8 Volts) */
    q8_8_t inductance_voltage;     /* Voltage for L measurement (high-frequency) */
    
    /* Timing (in control cycles) */
    uint16_t resistance_samples;   /* Number of samples for R */
    uint16_t inductance_samples;   /* Number of samples for L */
    uint16_t flux_samples;         /* Number of samples for flux */
    uint16_t settle_time;          /* Settle time before measurement */
    
    /* Duty cycle limits */
    q15_t max_duty;                /* Maximum duty for detection */
    
    /* Hall detection */
    q8_8_t hall_detect_current;    /* Current for Hall detection */
    q12_4_t hall_detect_rpm;       /* RPM for Hall detection */
    uint8_t hall_detect_cycles;    /* Number of electrical cycles */
    
} detect_config_t;

/**
 * @brief Detection results
 */
typedef struct {
    /* Motor parameters */
    q8_8_t resistance;             /* Phase resistance (Ohm) */
    q12_20_t inductance;           /* Phase inductance (H) */
    q12_20_t inductance_d;         /* D-axis inductance (H) */
    q12_20_t inductance_q;         /* Q-axis inductance (H) */
    q12_20_t ld_lq_diff;           /* Lq - Ld difference (H) */
    q12_20_t flux_linkage;         /* Flux linkage (Wb) */
    
    /* Hall sensor table */
    int8_t hall_table[8];          /* Hall to angle mapping */
    
    /* Measurement quality */
    uint16_t resistance_variance;  /* Variance of R measurement */
    uint16_t inductance_variance;  /* Variance of L measurement */
    uint16_t flux_variance;        /* Variance of flux measurement */
    
    /* Detection status */
    detect_state_t state;
    detect_error_t error;
    uint8_t completion_percent;    /* 0-100% */
    
} detect_results_t;

/**
 * @brief Detection runtime state
 */
typedef struct {
    /* State machine */
    detect_state_t state;
    detect_error_t error;
    
    /* Configuration */
    const detect_config_t *config;
    detect_results_t *results;
    
    /* Internal state */
    uint32_t sample_count;
    uint32_t cycle_count;
    
    /* Accumulators for averaging */
    int64_t r_sum;
    int64_t l_sum;
    int64_t l_diff_sum;
    int64_t flux_sum;
    int64_t r_sq_sum;              /* For variance calculation */
    int64_t l_sq_sum;
    
    /* Hall learning state */
    int16_t hall_angles[8];
    uint16_t hall_counts[8];
    uint8_t hall_state_prev;
    q16_16_t openloop_angle;
    
    /* Temporal measurements */
    q31_t start_timestamp;
    q31_t timeout_cycles;
    
    /* Duty/voltage/current for tests */
    q15_t test_duty;
    q8_8_t test_current;
    q15_t test_voltage_q15;
    
    /* Measurement buffers */
    q8_8_t v_buffer[64];
    q8_8_t i_buffer[64];
    uint8_t buffer_index;
    
} detect_state_machine_t;

/*============================================================================*/
/*                          FUNCTION DECLARATIONS                             */
/*============================================================================*/

/**
 * @brief Initialize detection module
 */
void detect_init(detect_state_machine_t *sm,
                  detect_results_t *results,
                  const detect_config_t *config);

/**
 * @brief Start full motor parameter detection sequence
 * 
 * Performs complete detection in order:
 * 1. Resistance
 * 2. Inductance (L and Ld-Lq)
 * 3. Flux linkage
 * 
 * @param sm Detection state machine
 * @param config Detection configuration
 * @param results Detection results
 * @return true if started successfully
 */
bool detect_start_full(detect_state_machine_t *sm,
                        const detect_config_t *config,
                        detect_results_t *results);

/**
 * @brief Start resistance measurement only
 */
bool detect_start_resistance(detect_state_machine_t *sm,
                              const detect_config_t *config,
                              detect_results_t *results);

/**
 * @brief Start inductance measurement only
 */
bool detect_start_inductance(detect_state_machine_t *sm,
                              const detect_config_t *config,
                              detect_results_t *results);

/**
 * @brief Start flux linkage measurement only
 */
bool detect_start_flux_linkage(detect_state_machine_t *sm,
                                const detect_config_t *config,
                                detect_results_t *results);

/**
 * @brief Start Hall sensor table detection
 */
bool detect_start_hall(detect_state_machine_t *sm,
                        const detect_config_t *config,
                        detect_results_t *results);

/**
 * @brief Update detection state machine
 * 
 * Call this function at the control loop rate.
 * Returns the duty cycle to apply for detection.
 * 
 * @param sm Detection state machine
 * @param dt Time step (Q16.16 seconds)
 * @param v_bus Bus voltage (Q8.8 V)
 * @param i_a Phase A current (Q8.8 A)
 * @param i_b Phase B current (Q8.8 A)
 * @param i_c Phase C current (Q8.8 A)
 * @param hall_state Current Hall sensor state (0-7)
 * @param duty_a Output duty A
 * @param duty_b Output duty B
 * @param duty_c Output duty C
 * @return true if detection is complete
 */
bool detect_update(detect_state_machine_t *sm,
                    q16_16_t dt,
                    q8_8_t v_bus,
                    q8_8_t i_a, q8_8_t i_b, q8_8_t i_c,
                    uint8_t hall_state,
                    uint16_t *duty_a, uint16_t *duty_b, uint16_t *duty_c);

/**
 * @brief Check if detection is complete
 */
static inline bool detect_is_complete(const detect_state_machine_t *sm) {
    return sm->state == DETECT_STATE_COMPLETE;
}

/**
 * @brief Check if detection failed
 */
static inline bool detect_has_failed(const detect_state_machine_t *sm) {
    return sm->state == DETECT_STATE_FAILED;
}

/**
 * @brief Get detection progress (0-100%)
 */
static inline uint8_t detect_get_progress(const detect_state_machine_t *sm) {
    return sm->results->completion_percent;
}

/**
 * @brief Get detection error
 */
static inline detect_error_t detect_get_error(const detect_state_machine_t *sm) {
    return sm->error;
}

/**
 * @brief Cancel ongoing detection
 */
void detect_cancel(detect_state_machine_t *sm);

/*============================================================================*/
/*                          RESISTANCE MEASUREMENT                            */
/*============================================================================*/

/**
 * @brief Measure resistance using DC injection
 * 
 * Applies a small DC voltage to one phase and measures current.
 * R = V / I
 * 
 * Uses multiple samples and averaging for accuracy.
 */
void detect_measure_resistance(detect_state_machine_t *sm,
                                 q8_8_t v_bus,
                                 q8_8_t i_a, q8_8_t i_b, q8_8_t i_c,
                                 q15_t *duty_out);

/**
 * @brief Calculate resistance from measurements
 */
q8_8_t detect_calc_resistance(q8_8_t voltage, q8_8_t current);

/*============================================================================*/
/*                          INDUCTANCE MEASUREMENT                            */
/*============================================================================*/

/**
 * @brief Measure inductance using high-frequency injection
 * 
 * Applies short voltage pulses and measures current response.
 * L = V * dt / di
 * 
 * Can also detect Ld-Lq difference for IPM motors by measuring
 * inductance at different rotor positions.
 */
void detect_measure_inductance(detect_state_machine_t *sm,
                                 q8_8_t v_bus,
                                 q8_8_t i_a, q8_8_t i_b, q8_8_t i_c,
                                 q15_t *duty_a_out, q15_t *duty_b_out, q15_t *duty_c_out);

/**
 * @brief Measure Ld and Lq separately for IPM motors
 * 
 * Rotates motor slowly and measures inductance at multiple
 * positions to find Ld (min) and Lq (max).
 */
void detect_measure_ld_lq(detect_state_machine_t *sm,
                           q8_8_t v_bus,
                           q8_8_t i_a, q8_8_t i_b, q8_8_t i_c,
                           q15_t *duty_out);

/**
 * @brief Calculate inductance from pulse measurement
 */
q12_20_t detect_calc_inductance(q8_8_t voltage, q8_8_t di, q16_16_t dt);

/*============================================================================*/
/*                          FLUX LINKAGE MEASUREMENT                           */
/*============================================================================*/

/**
 * @brief Measure flux linkage using back-EMF integration
 * 
 * Spins the motor open-loop and measures the generated back-EMF.
 * λ = V_bemf / ω
 * 
 * This requires the motor to be spinning, so it must be able
 * to rotate freely.
 */
void detect_measure_flux_linkage(detect_state_machine_t *sm,
                                  q8_8_t v_bus,
                                  q8_8_t i_a, q8_8_t i_b, q8_8_t i_c,
                                  q15_t *duty_out);

/**
 * @brief Calculate flux linkage from back-EMF
 * 
 * @param v_bemf Back-EMF voltage (Q8.8 V)
 * @param speed_rad_s Electrical speed (Q16.16 rad/s)
 * @return Flux linkage (Q12.20 Wb)
 */
q12_20_t detect_calc_flux_linkage(q8_8_t v_bemf, q31_t speed_rad_s);

/**
 * @brief Alternative: measure flux using locked rotor test
 * 
 * Locks the rotor and applies current to measure torque constant.
 * Requires external torque measurement or estimation.
 */
void detect_measure_flux_locked(detect_state_machine_t *sm,
                                 q8_8_t v_bus,
                                 q8_8_t i_a, q8_8_t i_b, q8_8_t i_c);

/*============================================================================*/
/*                          HALL SENSOR DETECTION                             */
/*============================================================================*/

/**
 * @brief Learn Hall sensor table
 * 
 * Spins motor in open-loop and records Hall transitions
 * to build the lookup table.
 */
void detect_learn_hall_table(detect_state_machine_t *sm,
                              uint8_t hall_state,
                              q15_t *duty_out);

/**
 * @brief Validate Hall sensor table
 * 
 * Checks if the learned Hall table is valid (proper sequence).
 */
bool detect_validate_hall_table(const int8_t hall_table[8]);

/**
 * @brief Build Hall table from learned angles
 */
void detect_build_hall_table(detect_state_machine_t *sm);

/*============================================================================*/
/*                          UTILITY FUNCTIONS                                 */
/*============================================================================*/

/**
 * @brief Set default detection configuration
 */
void detect_get_default_config(detect_config_t *config);

/**
 * @brief Validate detection results
 */
bool detect_validate_results(const detect_results_t *results);

/**
 * @brief Get observer parameters from detection results
 */
void detect_get_observer_params(const detect_results_t *results,
                                 observer_params_t *params);

/**
 * @brief Get current controller gains from motor parameters
 */
void detect_calc_current_gains(const detect_results_t *results,
                                float bandwidth_hz,
                                q15_t *kp, q31_t *ki);

#endif /* FOC_MOTOR_DETECT_H_ */
