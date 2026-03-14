/**
 * @file foc_config.h
 * @brief FOC Configuration Definitions
 */

#ifndef FOC_CONFIG_H
#define FOC_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "types.h"

/*============================================================================*/
/* Control Mode Definitions                                                   */
/*============================================================================*/

typedef enum {
    CTRL_MODE_DUTY = 0,         // Open loop duty cycle control
    CTRL_MODE_CURRENT,          // Torque control (current)
    CTRL_MODE_SPEED,            // Speed control
    CTRL_MODE_POSITION,         // Position control
    CTRL_MODE_HANDBRAKE,        // Handbrake (holding current)
    CTRL_MODE_BRAKE,            // Passive brake
} control_mode_e;

/*============================================================================*/
/* Motor Configuration Structure                                              */
/*============================================================================*/

typedef struct {
    // Electrical parameters
    uint8_t     pole_pairs;         // Number of pole pairs
    q15_t       resistance;         // Phase resistance (Q12.4, mΩ)
    q15_t       inductance_ld;      // d-axis inductance (Q12.4, µH)
    q15_t       inductance_lq;      // q-axis inductance (Q12.4, µH)
    q15_t       flux_linkage;       // Flux linkage (Q12.4, mWb)
    
    // Mechanical parameters
    int16_t     inertia;            // Rotor inertia (Q8.8, g·cm²)
    int16_t     friction;           // Friction coefficient (Q8.8)
    
    // Limits
    q15_t       max_current;        // Maximum phase current (Q12.4, A)
    q15_t       rated_current;      // Rated current (Q12.4, A)
    int16_t     max_speed;          // Maximum speed (RPM)
    
    // Sensor configuration
    uint8_t     hall_table[8];      // Hall sensor pattern to angle map
    uint8_t     hall_interpolation; // Enable hall interpolation
    
    // Observer parameters
    q15_t       observer_gain;      // Observer gain (Q8.8)
    q15_t       pll_kp;             // PLL proportional gain (Q8.8)
    q15_t       pll_ki;             // PLL integral gain (Q8.8)
    
} motor_config_t;

/*============================================================================*/
/* Control Configuration Structure                                            */
/*============================================================================*/

typedef struct {
    // Control mode
    control_mode_e ctrl_mode;
    
    // Current loop PI gains (Q8.8 format)
    q15_t       current_kp;
    q15_t       current_ki;
    q15_t       current_kd;
    
    // Speed loop PI gains (Q8.8 format)
    q15_t       speed_kp;
    q15_t       speed_ki;
    q15_t       speed_kd;
    
    // Position loop PI gains
    q15_t       position_kp;
    q15_t       position_ki;
    q15_t       position_kd;
    
    // Observer configuration
    q15_t       observer_gain;
    
    // Hall/Observer blending speeds (RPM)
    int16_t     hall_blend_start_speed;
    int16_t     hall_blend_end_speed;
    
    // Field weakening
    q15_t       fw_current_max;     // Maximum field weakening current (Q12.4)
    q15_t       fw_voltage_margin;  // Voltage margin for FW (Q8.8)
    
    // Regenerative braking
    q15_t       regen_voltage_limit; // Maximum bus voltage during regen (Q8.8)
    q15_t       regen_current_max;   // Maximum regenerative current (Q12.4)
    
    // Anti-dither (for smooth low speed)
    int16_t     anti_dither_speed;   // Speed threshold for anti-dither (RPM)
    q15_t       anti_dither_gain;    // Anti-dither gain (Q8.8)
    
} control_config_t;

/*============================================================================*/
/* Motor Detection Result                                                     */
/*============================================================================*/

typedef struct {
    q15_t   resistance;          // Detected resistance (Q12.4, mΩ)
    q15_t   inductance_ld;       // Detected d-axis inductance (Q12.4, µH)
    q15_t   inductance_lq;       // Detected q-axis inductance (Q12.4, µH)
    q15_t   flux_linkage;        // Detected flux linkage (Q12.4, mWb)
    uint8_t pole_pairs;          // Detected pole pairs
    uint8_t hall_table[8];       // Detected hall table
    int16_t hall_order;          // Hall sensor rotation direction
    int16_t encoder_offset;      // Encoder zero offset
    q15_t   temp_motor;          // Motor temperature during detection
} motor_detect_result_t;

/*============================================================================*/
/* Default Configurations                                                     */
/*============================================================================*/

// Default motor config for typical BLDC motor
#define MOTOR_CONFIG_DEFAULT { \
    .pole_pairs = 7, \
    .resistance = 50,         /* 50mΩ in Q12.4 */ \
    .inductance_ld = 500,     /* 50µH = 500 in Q12.4 (0.05mH) */ \
    .inductance_lq = 500, \
    .flux_linkage = 100,      /* 10mWb in Q12.4 = 100 */ \
    .max_current = 30000,     /* 30A in Q12.4 */ \
    .rated_current = 15000,   /* 15A in Q12.4 */ \
    .max_speed = 10000, \
}

// Default control config
#define CONTROL_CONFIG_DEFAULT { \
    .ctrl_mode = CTRL_MODE_SPEED, \
    .current_kp = 128,       /* 0.5 in Q8.8 */ \
    .current_ki = 12800,     /* 50 in Q8.8 */ \
    .speed_kp = 1,           /* 0.005 in Q8.8 = 1 */ \
    .speed_ki = 26,          /* 0.1 in Q8.8 ≈ 26 */ \
    .observer_gain = 256000, /* 1000 in Q8.8 */ \
    .hall_blend_start_speed = 300, \
    .hall_blend_end_speed = 800, \
}

/*============================================================================*/
/* Runtime State Structure                                                    */
/*============================================================================*/

typedef struct {
    // Current state
    q15_t   id;                 // d-axis current (Q15)
    q15_t   iq;                 // q-axis current (Q15)
    q15_t   id_target;          // Target d-axis current (Q15)
    q15_t   iq_target;          // Target q-axis current (Q15)
    
    // Voltage outputs
    q15_t   vd;                 // d-axis voltage (Q15)
    q15_t   vq;                 // q-axis voltage (Q15)
    
    // Angle and speed
    q15_t   angle_elec;         // Electrical angle (Q15, 0-65535 = 0-2π)
    int16_t speed_rpm;          // Mechanical speed (RPM)
    
    // Observer state
    q15_t   observer_angle;     // Observer estimated angle (Q15)
    q15_t   observer_speed;     // Observer estimated speed (Q15)
    
    // Hall state
    uint8_t hall_state;         // Current hall sensor state
    q15_t   hall_angle;         // Hall interpolated angle (Q15)
    
    // Fusion state
    q15_t   fusion_angle;       // Fused angle output (Q15)
    q15_t   fusion_weight;      // Hall vs Observer weight (Q15, 0=hall, 32767=observer)
    
    // Fault flags
    uint16_t fault_code;        // Active faults
    
} foc_state_t;

/*============================================================================*/
/* Fault Codes                                                                */
/*============================================================================*/

#define FAULT_NONE              0x0000
#define FAULT_OVER_VOLTAGE      0x0001
#define FAULT_UNDER_VOLTAGE     0x0002
#define FAULT_OVER_CURRENT      0x0004
#define FAULT_OVER_TEMP         0x0008
#define FAULT_HALL_ERROR        0x0010
#define FAULT_OBSERVER_ERROR    0x0020
#define FAULT_DRV_FAULT         0x0040
#define FAULT_PHASE_LOSS        0x0080
#define FAULT_DETECTION_FAILED  0x0100

#ifdef __cplusplus
}
#endif

#endif /* FOC_CONFIG_H */
