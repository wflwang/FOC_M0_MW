/**
 * @file foc_hall.h
 * @brief Hall Sensor Processing for FOC Motor Control
 * @author FOC Team
 * @date 2024
 * 
 * This module handles Hall sensor signal processing, including:
 * - Sector detection and angle interpolation
 * - Speed estimation from Hall transitions
 * - Direction detection
 * - Automatic Hall table learning
 */

#ifndef FOC_HALL_H_
#define FOC_HALL_H_

#include "types.h"
#include "fixed_point.h"

/*============================================================================*/
/*                          CONSTANTS                                         */
/*============================================================================*/

/* Hall sensor state codes (3-bit, values 0-7) */
#define HALL_STATE_0       0    /* 000 - Invalid */
#define HALL_STATE_1       1    /* 001 */
#define HALL_STATE_2       2    /* 010 */
#define HALL_STATE_3       3    /* 011 */
#define HALL_STATE_4       4    /* 100 */
#define HALL_STATE_5       5    /* 101 */
#define HALL_STATE_6       6    /* 110 */
#define HALL_STATE_7       7    /* 111 - Invalid */

/* Hall angle in encoder counts (0-65535 = 0-360°) */
/* Standard 120° Hall sensor spacing gives 6 sectors of 60° each */
#define HALL_SECTOR_SIZE   10923    /* 65536 / 6 = 10923 counts per sector */

/* Hall transition timeout (in control cycles) */
#define HALL_TIMEOUT_CYCLES    100000

/* Invalid Hall state marker */
#define HALL_ANGLE_INVALID     (-1)

/*============================================================================*/
/*                          STRUCTURES                                        */
/*============================================================================*/

/**
 * @brief Hall sensor configuration
 */
typedef struct {
    /* Hall GPIO pins configuration */
    void *hall_port_h1;
    uint16_t hall_pin_h1;
    void *hall_port_h2;
    uint16_t hall_pin_h2;
    void *hall_port_h3;
    uint16_t hall_pin_h3;
    
    /* Hall lookup table: maps Hall state (0-7) to angle (0-200) */
    /* 0-200 represents 0-360° in 1.8° steps */
    /* -1 indicates invalid Hall state */
    int8_t hall_table[8];
    
    /* Interpolation parameters */
    q12_4_t interp_min_rpm;         /* Minimum RPM for interpolation */
    q12_4_t interp_max_rpm;         /* Maximum RPM for interpolation */
    
    /* Filtering */
    uint8_t filter_samples;         /* Number of samples for debounce */
    q15_t angle_lpf_coeff;          /* Angle low-pass filter coefficient */
    
} hall_config_t;

/**
 * @brief Hall sensor runtime state
 */
typedef struct {
    /* Current Hall sensor state */
    uint8_t hall_state;             /* Current Hall value (0-7) */
    uint8_t hall_state_prev;        /* Previous Hall value */
    uint8_t hall_state_filtered;    /* Debounced Hall value */
    
    /* Angle estimation */
    q16_16_t angle;                 /* Interpolated Hall angle (radians) */
    q16_16_t angle_raw;             /* Raw sector angle (no interpolation) */
    q16_16_t angle_sector_center;   /* Center angle of current sector */
    
    /* Speed estimation */
    q12_4_t speed_rpm;              /* Estimated speed from Hall transitions */
    q31_t transition_timestamp;     /* Timestamp of last Hall transition */
    q31_t sector_time;              /* Time for last 60° sector (in ticks) */
    q31_t sector_time_filtered;     /* Filtered sector time */
    
    /* Direction */
    int8_t direction;               /* 1 = CW, -1 = CCW, 0 = unknown */
    int8_t direction_confidence;    /* Direction confidence (0-100) */
    
    /* Sector tracking */
    uint8_t sector;                 /* Current sector (0-5) */
    uint8_t sector_prev;            /* Previous sector */
    int8_t sector_transition;       /* Sector transition (+1, -1, or error = 0) */
    
    /* Interpolation state */
    q16_16_t interp_angle_start;    /* Angle at sector start */
    q31_t interp_time_start;        /* Time at sector start */
    q16_16_t interp_speed;          /* Interpolation speed (rad/s) */
    
    /* Learning mode */
    bool learning_active;           /* Hall table learning in progress */
    uint8_t learning_state;         /* State being learned */
    int16_t learning_angle[8];      /* Accumulated angles during learning */
    uint8_t learning_count[8];      /* Sample count for each state */
    
    /* Status */
    bool is_valid;                  /* Hall readings are valid */
    bool error_detected;            /* Invalid Hall sequence detected */
    uint32_t error_count;           /* Total Hall errors */
    uint32_t timeout_counter;       /* Timeout counter for no transitions */
    
} hall_state_t;

/*============================================================================*/
/*                          FUNCTION DECLARATIONS                             */
/*============================================================================*/

/**
 * @brief Initialize Hall sensor processing
 */
void hall_init(hall_state_t *state, const hall_config_t *config);

/**
 * @brief Read current Hall sensor state from GPIO
 * @param config Hall configuration
 * @return Hall state (0-7)
 */
uint8_t hall_read_state(const hall_config_t *config);

/**
 * @brief Update Hall sensor processing
 * 
 * Call this function at the control loop rate.
 * 
 * @param state Hall runtime state
 * @param config Hall configuration
 * @param timestamp Current timestamp (in control cycles)
 * @return Estimated angle from Hall sensor (Q16.16 radians)
 */
q16_16_t hall_update(hall_state_t *state, 
                      const hall_config_t *config,
                      q31_t timestamp);

/**
 * @brief Get the current Hall angle (interpolated)
 */
static inline q16_16_t hall_get_angle(const hall_state_t *state) {
    return state->angle;
}

/**
 * @brief Get the estimated speed from Hall transitions
 */
static inline q12_4_t hall_get_speed(const hall_state_t *state) {
    return state->speed_rpm;
}

/**
 * @brief Get the direction of rotation
 */
static inline int8_t hall_get_direction(const hall_state_t *state) {
    return state->direction;
}

/**
 * @brief Check if Hall sensor is valid
 */
static inline bool hall_is_valid(const hall_state_t *state) {
    return state->is_valid;
}

/**
 * @brief Start Hall table learning sequence
 * 
 * During learning, the motor is driven in open loop and Hall transitions
 * are recorded to build the lookup table.
 * 
 * @param state Hall runtime state
 */
void hall_start_learning(hall_state_t *state);

/**
 * @brief Update learning with open-loop angle
 * 
 * Call during learning mode with the forced open-loop angle.
 * 
 * @param state Hall runtime state
 * @param openloop_angle Current open-loop angle (Q16.16 radians)
 * @return true if learning is complete
 */
bool hall_update_learning(hall_state_t *state, q16_16_t openloop_angle);

/**
 * @brief Get learned Hall table
 * @param state Hall runtime state
 * @param table Output buffer for Hall table (8 entries)
 */
void hall_get_learned_table(const hall_state_t *state, int8_t table[8]);

/**
 * @brief Set Hall table manually
 */
void hall_set_table(hall_config_t *config, const int8_t table[8]);

/**
 * @brief Get Hall sector from angle
 * @param angle Angle in Q16.16 radians
 * @return Sector number (0-5)
 */
uint8_t hall_angle_to_sector(q16_16_t angle);

/**
 * @brief Get center angle of a Hall sector
 * @param sector Sector number (0-5)
 * @return Center angle in Q16.16 radians
 */
q16_16_t hall_sector_center_angle(uint8_t sector);

/**
 * @brief Reset Hall state
 */
void hall_reset(hall_state_t *state);

#endif /* FOC_HALL_H_ */
