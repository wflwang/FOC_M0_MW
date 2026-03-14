/**
 * @file foc_motor_detect.c
 * @brief Motor Parameter Automatic Detection Implementation
 * @author FOC Team
 * @date 2024
 * 
 * Implements automatic detection of motor parameters following VESC methodology.
 * All calculations use fixed-point arithmetic optimized for Cortex-M0.
 */

#include "foc_motor_detect.h"
#include "foc_svpwm.h"
#include <string.h>

/*============================================================================*/
/*                          INTERNAL CONSTANTS                                */
/*============================================================================*/

/* Default detection parameters */
#define DEFAULT_R_CURRENT       (q8_8_t)(5.0 * 256)       /* 5A for R measurement */
#define DEFAULT_L_CURRENT       (q8_8_t)(5.0 * 256)       /* 5A for L measurement */
#define DEFAULT_L_VOLTAGE       (q8_8_t)(5.0 * 256)       /* 5V pulse for L */
#define DEFAULT_FLUX_CURRENT    (q8_8_t)(5.0 * 256)       /* 5A for flux */
#define DEFAULT_HALL_CURRENT    (q8_8_t)(3.0 * 256)       /* 3A for Hall learning */
#define DEFAULT_HALL_RPM        (q12_4_t)(300 * 16)       /* 300 RPM for Hall */

/* Timing constants */
#define DEFAULT_SETTLE_TIME     (500)      /* 500 cycles settle time */
#define DEFAULT_R_SAMPLES       (1000)     /* 1000 samples for R */
#define DEFAULT_L_SAMPLES       (100)      /* 100 samples for L */
#define DEFAULT_FLUX_SAMPLES    (500)      /* 500 samples for flux */
#define DEFAULT_TIMEOUT_MS      (10000)    /* 10 second timeout */

/* Pulse timing for inductance measurement */
#define L_PULSE_ON_CYCLES       (10)
#define L_PULSE_OFF_CYCLES      (10)

/*============================================================================*/
/*                          INTERNAL FUNCTIONS                                */
/*============================================================================*/

/**
 * @brief Progress through state machine
 */
static void advance_state(detect_state_machine_t *sm, detect_state_t next_state) {
    sm->state = next_state;
    sm->sample_count = 0;
    sm->cycle_count = 0;
    sm->buffer_index = 0;
    sm->r_sum = 0;
    sm->l_sum = 0;
    sm->flux_sum = 0;
}

/**
 * @brief Calculate variance and update result
 */
static void finalize_resistance(detect_state_machine_t *sm) {
    if (sm->sample_count > 0) {
        /* Average resistance */
        sm->results->resistance = (q8_8_t)(sm->r_sum / sm->sample_count);
        
        /* Calculate variance if we tracked squares */
        if (sm->r_sq_sum > 0) {
            int64_t mean = sm->r_sum / sm->sample_count;
            int64_t variance = (sm->r_sq_sum / sm->sample_count) - (mean * mean);
            sm->results->resistance_variance = (uint16_t)(variance >> 8);
        }
    }
}

/**
 * @brief Generate open-loop angle for detection
 */
static q16_16_t generate_openloop_angle(detect_state_machine_t *sm, q16_16_t dt, q12_4_t rpm) {
    /* Increment angle based on target RPM */
    /* dθ/dt = RPM * 2π/60 */
    /* In Q16.16: dθ = rpm * 2π/60 * dt */
    /* Simplified: dθ ≈ rpm * 205888 / 60 * dt */
    
    int64_t d_theta = (int64_t)rpm * 3431 * dt;  /* 3431 ≈ 205888/60 */
    d_theta >>= 16;
    
    sm->openloop_angle += (q16_16_t)d_theta;
    
    /* Normalize to [0, 2π) */
    while (sm->openloop_angle >= 411775) {
        sm->openloop_angle -= 411775;
    }
    while (sm->openloop_angle < 0) {
        sm->openloop_angle += 411775;
    }
    
    return sm->openloop_angle;
}

/*============================================================================*/
/*                          PUBLIC FUNCTIONS                                  */
/*============================================================================*/

/**
 * @brief Initialize detection module
 */
void detect_init(detect_state_machine_t *sm,
                  detect_results_t *results,
                  const detect_config_t *config) {
    memset(sm, 0, sizeof(detect_state_machine_t));
    memset(results, 0, sizeof(detect_results_t));
    
    sm->config = config;
    sm->results = results;
    sm->state = DETECT_STATE_IDLE;
    sm->error = DETECT_OK;
    
    /* Initialize Hall table to invalid */
    for (int i = 0; i < 8; i++) {
        results->hall_table[i] = -1;
    }
}

/**
 * @brief Get default detection configuration
 */
void detect_get_default_config(detect_config_t *config) {
    config->resistance_current = DEFAULT_R_CURRENT;
    config->inductance_current = DEFAULT_L_CURRENT;
    config->inductance_voltage = DEFAULT_L_VOLTAGE;
    config->flux_current = DEFAULT_FLUX_CURRENT;
    config->resistance_samples = DEFAULT_R_SAMPLES;
    config->inductance_samples = DEFAULT_L_SAMPLES;
    config->flux_samples = DEFAULT_FLUX_SAMPLES;
    config->settle_time = DEFAULT_SETTLE_TIME;
    config->max_duty = (q15_t)(0.5 * Q15_ONE);
    config->hall_detect_current = DEFAULT_HALL_CURRENT;
    config->hall_detect_rpm = DEFAULT_HALL_RPM;
    config->hall_detect_cycles = 2;
}

/**
 * @brief Start full motor parameter detection
 */
bool detect_start_full(detect_state_machine_t *sm,
                        const detect_config_t *config,
                        detect_results_t *results) {
    sm->config = config;
    sm->results = results;
    sm->state = DETECT_STATE_MEASURE_R;
    sm->error = DETECT_OK;
    sm->sample_count = 0;
    sm->cycle_count = 0;
    sm->results->completion_percent = 0;
    
    /* Set timeout */
    sm->timeout_cycles = 200000;  /* ~10 seconds at 20kHz */
    
    return true;
}

/**
 * @brief Start resistance measurement only
 */
bool detect_start_resistance(detect_state_machine_t *sm,
                              const detect_config_t *config,
                              detect_results_t *results) {
    detect_init(sm, results, config);
    sm->state = DETECT_STATE_MEASURE_R;
    return true;
}

/**
 * @brief Start inductance measurement only
 */
bool detect_start_inductance(detect_state_machine_t *sm,
                              const detect_config_t *config,
                              detect_results_t *results) {
    detect_init(sm, results, config);
    sm->state = DETECT_STATE_MEASURE_L;
    return true;
}

/**
 * @brief Start flux linkage measurement only
 */
bool detect_start_flux_linkage(detect_state_machine_t *sm,
                                const detect_config_t *config,
                                detect_results_t *results) {
    detect_init(sm, results, config);
    sm->state = DETECT_STATE_MEASURE_FLUX;
    return true;
}

/**
 * @brief Start Hall sensor table detection
 */
bool detect_start_hall(detect_state_machine_t *sm,
                        const detect_config_t *config,
                        detect_results_t *results) {
    detect_init(sm, results, config);
    sm->state = DETECT_STATE_MEASURE_HALL;
    
    /* Initialize Hall learning */
    for (int i = 0; i < 8; i++) {
        sm->hall_angles[i] = 0;
        sm->hall_counts[i] = 0;
    }
    sm->hall_state_prev = 0xFF;
    sm->openloop_angle = 0;
    
    return true;
}

/**
 * @brief Cancel ongoing detection
 */
void detect_cancel(detect_state_machine_t *sm) {
    sm->state = DETECT_STATE_IDLE;
    sm->error = DETECT_OK;
}

/*============================================================================*/
/*                          MAIN UPDATE FUNCTION                              */
/*============================================================================*/

/**
 * @brief Main detection update
 */
bool detect_update(detect_state_machine_t *sm,
                    q16_16_t dt,
                    q8_8_t v_bus,
                    q8_8_t i_a, q8_8_t i_b, q8_8_t i_c,
                    uint8_t hall_state,
                    uint16_t *duty_a, uint16_t *duty_b, uint16_t *duty_c) {
    
    /* Check timeout */
    sm->cycle_count++;
    if (sm->cycle_count > sm->timeout_cycles) {
        sm->state = DETECT_STATE_FAILED;
        sm->error = DETECT_ERR_TIMEOUT;
        return true;
    }
    
    /* Default: no output */
    *duty_a = 0;
    *duty_b = 0;
    *duty_c = 0;
    
    q15_t duty_q15 = 0;
    
    switch (sm->state) {
        case DETECT_STATE_IDLE:
            /* Nothing to do */
            break;
            
        case DETECT_STATE_MEASURE_R:
            detect_measure_resistance(sm, v_bus, i_a, i_b, i_c, &duty_q15);
            
            /* Convert single duty to three-phase (apply to A only) */
            *duty_a = (uint16_t)(duty_q15 > 0 ? 32768 + duty_q15 : 32768);
            *duty_b = 32768;
            *duty_c = 32768;
            
            sm->results->completion_percent = (uint8_t)((sm->sample_count * 33) / sm->config->resistance_samples);
            
            if (sm->sample_count >= sm->config->resistance_samples) {
                finalize_resistance(sm);
                
                /* Validate resistance */
                if (sm->results->resistance < DETECT_R_MIN) {
                    sm->state = DETECT_STATE_FAILED;
                    sm->error = DETECT_ERR_LOW_RESISTANCE;
                    return true;
                }
                if (sm->results->resistance > DETECT_R_MAX) {
                    sm->state = DETECT_STATE_FAILED;
                    sm->error = DETECT_ERR_HIGH_RESISTANCE;
                    return true;
                }
                
                /* Move to next measurement */
                advance_state(sm, DETECT_STATE_MEASURE_L);
            }
            break;
            
        case DETECT_STATE_MEASURE_L:
            detect_measure_inductance(sm, v_bus, i_a, i_b, i_c, &duty_q15, duty_a, duty_b, duty_c);
            
            sm->results->completion_percent = 33 + (uint8_t)((sm->sample_count * 33) / sm->config->inductance_samples);
            
            if (sm->sample_count >= sm->config->inductance_samples) {
                /* Finalize inductance */
                if (sm->l_sum > 0) {
                    sm->results->inductance = (q12_20_t)(sm->l_sum / sm->sample_count);
                }
                
                /* Validate inductance */
                if (sm->results->inductance < DETECT_L_MIN) {
                    sm->state = DETECT_STATE_FAILED;
                    sm->error = DETECT_ERR_LOW_INDUCTANCE;
                    return true;
                }
                if (sm->results->inductance > DETECT_L_MAX) {
                    sm->state = DETECT_STATE_FAILED;
                    sm->error = DETECT_ERR_HIGH_INDUCTANCE;
                    return true;
                }
                
                /* Calculate Ld-Lq if measured */
                if (sm->l_diff_sum != 0) {
                    sm->results->ld_lq_diff = (q12_20_t)(sm->l_diff_sum / sm->sample_count);
                    sm->results->inductance_d = sm->results->inductance - sm->results->ld_lq_diff / 2;
                    sm->results->inductance_q = sm->results->inductance + sm->results->ld_lq_diff / 2;
                }
                
                advance_state(sm, DETECT_STATE_MEASURE_FLUX);
            }
            break;
            
        case DETECT_STATE_MEASURE_FLUX:
            detect_measure_flux_linkage(sm, v_bus, i_a, i_b, i_c, &duty_q15);
            
            /* Generate three-phase PWM from angle */
            {
                svpwm_output_t svpwm_out;
                svpwm_config_t svpwm_cfg = { .pwm_period = 65535 };
                
                q15_t mod_alpha, mod_beta;
                q15_t sin_theta = fast_sin(sm->openloop_angle);
                q15_t cos_theta = fast_cos(sm->openloop_angle);
                
                /* Inverse Park with test current */
                inv_park_transform(0, duty_q15, sin_theta, cos_theta, &mod_alpha, &mod_beta);
                
                svpwm_generate(&svpwm_cfg, mod_alpha, mod_beta, &svpwm_out);
                
                *duty_a = svpwm_out.duty_a;
                *duty_b = svpwm_out.duty_b;
                *duty_c = svpwm_out.duty_c;
            }
            
            sm->results->completion_percent = 66 + (uint8_t)((sm->sample_count * 30) / sm->config->flux_samples);
            
            if (sm->sample_count >= sm->config->flux_samples) {
                /* Finalize flux linkage */
                if (sm->flux_sum > 0) {
                    sm->results->flux_linkage = (q12_20_t)(sm->flux_sum / sm->sample_count);
                }
                
                /* Validate flux */
                if (sm->results->flux_linkage < DETECT_FLUX_MIN ||
                    sm->results->flux_linkage > DETECT_FLUX_MAX) {
                    sm->state = DETECT_STATE_FAILED;
                    sm->error = DETECT_ERR_FLUX_LINKAGE;
                    return true;
                }
                
                sm->results->completion_percent = 100;
                sm->state = DETECT_STATE_COMPLETE;
                return true;
            }
            break;
            
        case DETECT_STATE_MEASURE_HALL:
            detect_learn_hall_table(sm, hall_state, &duty_q15);
            
            /* Generate open-loop spinning */
            {
                q16_16_t angle = generate_openloop_angle(sm, dt, sm->config->hall_detect_rpm);
                
                svpwm_output_t svpwm_out;
                svpwm_config_t svpwm_cfg = { .pwm_period = 65535 };
                
                q15_t mod_alpha, mod_beta;
                q15_t sin_theta = fast_sin(angle);
                q15_t cos_theta = fast_cos(angle);
                
                inv_park_transform(0, duty_q15, sin_theta, cos_theta, &mod_alpha, &mod_beta);
                
                svpwm_generate(&svpwm_cfg, mod_alpha, mod_beta, &svpwm_out);
                
                *duty_a = svpwm_out.duty_a;
                *duty_b = svpwm_out.duty_b;
                *duty_c = svpwm_out.duty_c;
            }
            
            /* Check if Hall learning is complete */
            {
                int valid_states = 0;
                for (int i = 1; i <= 6; i++) {
                    if (sm->hall_counts[i] > 10) valid_states++;
                }
                
                if (valid_states >= 6) {
                    detect_build_hall_table(sm);
                    sm->state = DETECT_STATE_COMPLETE;
                    return true;
                }
            }
            break;
            
        case DETECT_STATE_COMPLETE:
        case DETECT_STATE_FAILED:
            return true;
            
        default:
            break;
    }
    
    return false;
}

/*============================================================================*/
/*                          RESISTANCE MEASUREMENT                            */
/*============================================================================*/

/**
 * @brief Measure resistance using DC injection
 * 
 * Principle: Apply DC voltage to phase A, measure resulting current.
 * R = V / I
 * 
 * We use a two-phase connection (A to B) to complete the circuit.
 */
void detect_measure_resistance(detect_state_machine_t *sm,
                                 q8_8_t v_bus,
                                 q8_8_t i_a, q8_8_t i_b, q8_8_t i_c,
                                 q15_t *duty_out) {
    
    const detect_config_t *cfg = sm->config;
    
    /* Wait for settle time */
    if (sm->cycle_count < cfg->settle_time) {
        /* Apply test voltage to establish current */
        /* Simple P controller to reach target current */
        q8_8_t target_current = cfg->resistance_current;
        q8_8_t current_error = target_current - q8_8_abs(i_a);
        
        /* Adjust duty based on error */
        sm->test_duty += (q15_t)(current_error << 4);
        
        /* Limit duty */
        if (sm->test_duty > cfg->max_duty) sm->test_duty = cfg->max_duty;
        if (sm->test_duty < 0) sm->test_duty = 0;
        
        *duty_out = sm->test_duty;
        return;
    }
    
    /* Sampling phase */
    *duty_out = sm->test_duty;
    
    /* Use phase A and B currents (they should be opposite) */
    q8_8_t i_mag = q8_8_abs(i_a);
    
    if (i_mag > (q8_8_t)(0.5 * 256)) {  /* At least 0.5A */
        /* Calculate voltage applied */
        /* V = duty * v_bus */
        q8_8_t v_applied = (q8_8_t)(((int32_t)sm->test_duty * v_bus) >> 15);
        
        /* R = V / I */
        /* Account for two phases in series: R_phase = V / (2 * I) */
        if (i_mag > 0) {
            q8_8_t r_sample = (q8_8_t)(((int32_t)v_applied << 8) / (2 * i_mag));
            
            /* Accumulate for averaging */
            sm->r_sum += r_sample;
            sm->r_sq_sum += (int64_t)r_sample * r_sample;
            sm->sample_count++;
        }
    }
}

/**
 * @brief Calculate resistance from measurements
 */
q8_8_t detect_calc_resistance(q8_8_t voltage, q8_8_t current) {
    if (current == 0) return 0;
    return (q8_8_t)(((int32_t)voltage << 8) / current);
}

/*============================================================================*/
/*                          INDUCTANCE MEASUREMENT                            */
/*============================================================================*/

/**
 * @brief Measure inductance using pulse injection
 * 
 * Principle: Apply voltage pulse, measure di/dt
 * L = V * dt / di
 * 
 * Uses high-frequency pulses to avoid rotor movement.
 */
void detect_measure_inductance(detect_state_machine_t *sm,
                                 q8_8_t v_bus,
                                 q8_8_t i_a, q8_8_t i_b, q8_8_t i_c,
                                 q15_t *duty_a_out, q15_t *duty_b_out, q15_t *duty_c_out) {
    
    const detect_config_t *cfg = sm->config;
    
    /* Pulse state machine */
    uint16_t pulse_phase = sm->cycle_count % (L_PULSE_ON_CYCLES + L_PULSE_OFF_CYCLES);
    
    q15_t duty = 0;
    
    if (pulse_phase < L_PULSE_ON_CYCLES) {
        /* ON phase: apply voltage pulse */
        duty = (q15_t)(((int32_t)cfg->inductance_voltage << 15) / v_bus);
        
        /* Store current at start of pulse */
        if (pulse_phase == 0) {
            sm->v_buffer[0] = cfg->inductance_voltage;
            sm->i_buffer[0] = i_a;
        }
        
        /* Store current at end of pulse */
        if (pulse_phase == L_PULSE_ON_CYCLES - 1) {
            sm->i_buffer[1] = i_a;
            
            /* Calculate L = V * dt / di */
            q8_8_t di = sm->i_buffer[1] - sm->i_buffer[0];
            
            if (q8_8_abs(di) > (q8_8_t)(0.1 * 256)) {  /* At least 0.1A change */
                /* dt = pulse_cycles / f_pwm */
                /* Simplified: use relative units */
                q12_20_t l_sample = (q12_20_t)(
                    ((int64_t)cfg->inductance_voltage * L_PULSE_ON_CYCLES << 20) / 
                    ((int64_t)q8_8_abs(di) << 8)
                );
                
                /* Scale appropriately */
                l_sample = l_sample >> 8;
                
                sm->l_sum += l_sample;
                sm->sample_count++;
            }
        }
    } else {
        /* OFF phase: let current decay */
        duty = 0;
    }
    
    /* Apply to phases A and B */
    *duty_a_out = duty;
    *duty_b_out = 0;
    *duty_c_out = 32768;  /* 50% duty (neutral) */
}

/**
 * @brief Calculate inductance from pulse measurement
 */
q12_20_t detect_calc_inductance(q8_8_t voltage, q8_8_t di, q16_16_t dt) {
    if (di == 0) return 0;
    
    /* L = V * dt / di */
    int64_t l = ((int64_t)voltage * dt << 12) / di;
    return (q12_20_t)l;
}

/*============================================================================*/
/*                          FLUX LINKAGE MEASUREMENT                           */
/*============================================================================*/

/**
 * @brief Measure flux linkage using back-EMF integration
 * 
 * Principle: Spin motor open-loop, measure generated back-EMF
 * λ = V_bemf / ω
 * 
 * The motor is spun at a known speed and the voltage required
 * to maintain that speed is measured.
 */
void detect_measure_flux_linkage(detect_state_machine_t *sm,
                                  q8_8_t v_bus,
                                  q8_8_t i_a, q8_8_t i_b, q8_8_t i_c,
                                  q15_t *duty_out) {
    
    const detect_config_t *cfg = sm->config;
    
    /* Generate open-loop angle */
    q16_16_t angle = generate_openloop_angle(sm, (q16_16_t)3277,  /* dt ≈ 50µs at 20kHz */
                                              (q12_4_t)(500 * 16));  /* 500 RPM */
    
    /* Calculate current magnitude */
    int32_t i_alpha = ((int32_t)i_a - i_b / 2 - i_c / 2) * 2 / 3;
    int32_t i_beta = ((int32_t)i_b - i_c) * 18919 >> 15;
    q8_8_t i_mag = (q8_8_t)fast_sqrt((uint32_t)(i_alpha * i_alpha + i_beta * i_beta));
    
    /* Wait for motor to spin up */
    if (sm->cycle_count < cfg->settle_time * 2) {
        /* Ramp up duty */
        sm->test_duty = (q15_t)((int64_t)sm->cycle_count * cfg->max_duty / (cfg->settle_time * 2));
        *duty_out = sm->test_duty;
        return;
    }
    
    /* Measure back-EMF */
    /* At steady state: V = R*I + ω*λ */
    /* So: λ = (V - R*I) / ω */
    
    /* Calculate speed in rad/s */
    q31_t speed_rad_s = (q31_t)(500 * 205888 / 60);  /* 500 RPM -> rad/s */
    
    /* Calculate voltage */
    q8_8_t v_applied = (q8_8_t)(((int32_t)sm->test_duty * v_bus) >> 15);
    
    /* Subtract resistive drop: V_bemf = V - R*I */
    q8_8_t r = sm->results->resistance;
    q8_8_t v_bemf = v_applied - (q8_8_t)(((int32_t)r * i_mag) >> 8);
    
    if (v_bemf > 0 && speed_rad_s > 0) {
        /* λ = V_bemf / ω */
        q12_20_t flux = (q12_20_t)(((int64_t)v_bemf << 20) / speed_rad_s);
        
        sm->flux_sum += flux;
        sm->sample_count++;
    }
    
    /* Maintain spinning */
    *duty_out = sm->test_duty;
}

/**
 * @brief Calculate flux linkage from back-EMF
 */
q12_20_t detect_calc_flux_linkage(q8_8_t v_bemf, q31_t speed_rad_s) {
    if (speed_rad_s == 0) return 0;
    
    /* λ = V_bemf / ω */
    return (q12_20_t)(((int64_t)v_bemf << 20) / speed_rad_s);
}

/*============================================================================*/
/*                          HALL SENSOR DETECTION                             */
/*============================================================================*/

/**
 * @brief Learn Hall sensor table
 * 
 * Records Hall transitions at each open-loop position.
 */
void detect_learn_hall_table(detect_state_machine_t *sm,
                              uint8_t hall_state,
                              q15_t *duty_out) {
    
    /* Apply test current */
    *duty_out = (q15_t)(((int32_t)sm->config->hall_detect_current << 15) / 
                        (sm->results->resistance > 0 ? sm->results->resistance : 256));
    
    /* Limit duty */
    if (*duty_out > sm->config->max_duty) {
        *duty_out = sm->config->max_duty;
    }
    
    /* Record angle at each Hall transition */
    if (hall_state >= 1 && hall_state <= 6) {
        /* Check for Hall state change */
        if (hall_state != sm->hall_state_prev) {
            /* Record the angle at this transition */
            int16_t angle_normalized = (int16_t)(sm->openloop_angle * 200 / 411775);
            
            sm->hall_angles[hall_state] += angle_normalized;
            sm->hall_counts[hall_state]++;
            
            sm->hall_state_prev = hall_state;
        }
    }
}

/**
 * @brief Build Hall table from learned angles
 */
void detect_build_hall_table(detect_state_machine_t *sm) {
    for (int i = 1; i <= 6; i++) {
        if (sm->hall_counts[i] > 0) {
            /* Average angle for this Hall state */
            int16_t avg_angle = sm->hall_angles[i] / sm->hall_counts[i];
            
            /* Normalize to 0-200 range */
            while (avg_angle < 0) avg_angle += 200;
            while (avg_angle >= 200) avg_angle -= 200;
            
            sm->results->hall_table[i] = (int8_t)avg_angle;
        }
    }
    
    /* Mark invalid states */
    sm->results->hall_table[0] = -1;
    sm->results->hall_table[7] = -1;
}

/**
 * @brief Validate Hall sensor table
 */
bool detect_validate_hall_table(const int8_t hall_table[8]) {
    /* Check that states 1-6 are defined */
    for (int i = 1; i <= 6; i++) {
        if (hall_table[i] < 0 || hall_table[i] > 200) {
            return false;
        }
    }
    
    /* Check that states 0 and 7 are invalid */
    if (hall_table[0] != -1 || hall_table[7] != -1) {
        return false;
    }
    
    /* Check monotonic sequence */
    int prev = hall_table[1];
    for (int i = 2; i <= 6; i++) {
        int curr = hall_table[i];
        int diff = curr - prev;
        if (diff < 0) diff += 200;
        
        /* Should advance by roughly 33 counts (60°) */
        if (diff < 20 || diff > 46) {  /* Allow some tolerance */
            return false;
        }
        prev = curr;
    }
    
    return true;
}

/*============================================================================*/
/*                          UTILITY FUNCTIONS                                 */
/*============================================================================*/

/**
 * @brief Validate detection results
 */
bool detect_validate_results(const detect_results_t *results) {
    /* Check resistance */
    if (results->resistance < DETECT_R_MIN || results->resistance > DETECT_R_MAX) {
        return false;
    }
    
    /* Check inductance */
    if (results->inductance < DETECT_L_MIN || results->inductance > DETECT_L_MAX) {
        return false;
    }
    
    /* Check flux linkage */
    if (results->flux_linkage < DETECT_FLUX_MIN || results->flux_linkage > DETECT_FLUX_MAX) {
        return false;
    }
    
    return true;
}

/**
 * @brief Get observer parameters from detection results
 */
void detect_get_observer_params(const detect_results_t *results,
                                 observer_params_t *params) {
    params->resistance = results->resistance;
    params->inductance = results->inductance;
    params->inductance_diff = results->ld_lq_diff;
    params->flux_linkage = results->flux_linkage;
    
    /* Set default observer gain: gamma ≈ R / L */
    if (results->inductance > 0) {
        float r = q8_8_to_float(results->resistance);
        float l = q12_20_to_float(results->inductance);
        params->gamma = float_to_q31(r / l * 1000.0f);  /* Scaled */
    }
}

/**
 * @brief Calculate current controller gains from motor parameters
 */
void detect_calc_current_gains(const detect_results_t *results,
                                float bandwidth_hz,
                                q15_t *kp, q31_t *ki) {
    /* Kp = L * bandwidth */
    /* Ki = R * bandwidth */
    
    float l = q12_20_to_float(results->inductance);
    float r = q8_8_to_float(results->resistance);
    
    *kp = float_to_q15(l * bandwidth_hz * 0.001f);  /* Scale for Q15 */
    *ki = float_to_q31(r * bandwidth_hz * 0.001f);  /* Scale for Q31 */
}
