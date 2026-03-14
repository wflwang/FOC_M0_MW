/**
 * @file foc_control.c
 * @brief FOC Main Control Implementation
 */

#include "foc_control.h"
#include "foc_observer.h"
#include "foc_angle_fusion.h"
#include "foc_svpwm.h"
#include "hw.h"
#include <string.h>

/*============================================================================*/
/* Private Variables                                                          */
/*============================================================================*/

static motor_config_t       g_motor_config;
static control_config_t     g_control_config;
static foc_state_t          g_foc_state;

// PI controller states
static q31_t   id_integral = 0;
static q31_t   iq_integral = 0;
static q31_t   speed_integral = 0;

// Current references
static q15_t   id_target = 0;
static q15_t   iq_target = 0;
static int16_t speed_target = 0;

// Control mode
static control_mode_e g_ctrl_mode = CTRL_MODE_SPEED;
static mc_state_e g_mc_state = MC_STATE_OFF;

/*============================================================================*/
/* Initialization                                                             */
/*============================================================================*/

void foc_control_init(const motor_config_t *motor_cfg, const control_config_t *ctrl_cfg) {
    // Copy configurations
    memcpy(&g_motor_config, motor_cfg, sizeof(motor_config_t));
    memcpy(&g_control_config, ctrl_cfg, sizeof(control_config_t));
    
    // Clear state
    memset(&g_foc_state, 0, sizeof(foc_state_t));
    
    // Initialize observer
    observer_init(&g_motor_config);
    
    // Initialize angle fusion
    angle_fusion_init(
        g_control_config.hall_blend_start_speed,
        g_control_config.hall_blend_end_speed
    );
    
    // Set initial state
    g_mc_state = MC_STATE_IDLE;
}

void foc_control_start(void) {
    if (g_mc_state == MC_STATE_IDLE || g_mc_state == MC_STATE_FAULT) {
        // Reset integrators
        id_integral = 0;
        iq_integral = 0;
        speed_integral = 0;
        
        // Enable PWM
        hw_pwm_enable(true);
        
        g_mc_state = MC_STATE_RUNNING;
    }
}

void foc_control_stop(void) {
    // Disable PWM
    hw_pwm_enable(false);
    
    // Reset states
    id_target = 0;
    iq_target = 0;
    
    g_mc_state = MC_STATE_IDLE;
}

/*============================================================================*/
/* Main Control Loop                                                          */
/*============================================================================*/

void foc_control_run(void) {
    if (g_mc_state != MC_STATE_RUNNING) {
        return;
    }
    
    // Get current measurements
    q15_t currents[3];
    hw_get_currents(currents);
    g_foc_state.id = currents[0];
    g_foc_state.iq = currents[1];
    
    // Get bus voltage
    q15_t v_bus = hw_get_bus_voltage();
    
    // Get hall state
    uint8_t hall_state = hw_get_hall_state();
    g_foc_state.hall_state = hall_state;
    
    // Get angle from hall (simplified)
    static const q15_t hall_angles[8] = {
        0,      // Invalid
        0,      // 001
        10923,  // 010 = 60°
        5461,   // 011 = 30°
        21845,  // 100 = 120°
        16384,  // 101 = 90°
        27306,  // 110 = 150°
        0       // 111 = Invalid
    };
    q15_t hall_angle = hall_angles[hall_state];
    g_foc_state.hall_angle = hall_angle;
    
    // Run observer
    observer_update(currents[0], currents[1], v_bus);
    g_foc_state.observer_angle = observer_get_angle();
    g_foc_state.observer_speed = observer_get_speed();
    
    // Fuse angles
    angle_fusion_update(hall_angle, g_foc_state.observer_angle, g_foc_state.speed_rpm);
    g_foc_state.fusion_angle = angle_fusion_get_angle();
    g_foc_state.fusion_weight = angle_fusion_get_weight();
    
    // Use fused angle for control
    q15_t theta = g_foc_state.fusion_angle;
    
    // Clarke transform (abc -> alpha-beta)
    // ia, ib already measured, ic = -ia - ib
    q15_t i_alpha = currents[0];
    q15_t i_beta = (q15_t)(((int32_t)currents[0] + 2 * (int32_t)currents[1]) * 18919 >> 15);
    
    // Park transform (alpha-beta -> dq)
    q15_t sin_theta, cos_theta;
    foc_sincos(theta, &sin_theta, &cos_theta);
    
    q15_t id = (q15_t)(((int32_t)i_alpha * cos_theta + (int32_t)i_beta * sin_theta) >> 15);
    q15_t iq = (q15_t)((-(int32_t)i_alpha * sin_theta + (int32_t)i_beta * cos_theta) >> 15);
    
    g_foc_state.id = id;
    g_foc_state.iq = iq;
    
    // Calculate speed from observer
    g_foc_state.speed_rpm = (int16_t)((g_foc_state.observer_speed * 60) >> 15);
    
    // Speed controller (if in speed mode)
    if (g_ctrl_mode == CTRL_MODE_SPEED) {
        int16_t speed_error = speed_target - g_foc_state.speed_rpm;
        
        speed_integral += (q31_t)speed_error * g_control_config.speed_ki;
        
        // Anti-windup
        if (speed_integral > (30000 << 15)) speed_integral = (30000 << 15);
        if (speed_integral < -(30000 << 15)) speed_integral = -(30000 << 15);
        
        iq_target = (q15_t)((g_control_config.speed_kp * speed_error) + (speed_integral >> 15));
        
        // Clamp
        if (iq_target > g_motor_config.max_current) iq_target = g_motor_config.max_current;
        if (iq_target < -g_motor_config.max_current) iq_target = -g_motor_config.max_current;
    }
    
    // Current PI controllers
    q15_t id_error = id_target - id;
    q15_t iq_error = iq_target - iq;
    
    // Update integrals
    id_integral += (q31_t)id_error * g_control_config.current_ki;
    iq_integral += (q31_t)iq_error * g_control_config.current_ki;
    
    // Anti-windup limits
    if (id_integral > (30000 << 15)) id_integral = (30000 << 15);
    if (id_integral < -(30000 << 15)) id_integral = -(30000 << 15);
    if (iq_integral > (30000 << 15)) iq_integral = (30000 << 15);
    if (iq_integral < -(30000 << 15)) iq_integral = -(30000 << 15);
    
    // Calculate voltage outputs
    q15_t vd = (q15_t)((g_control_config.current_kp * id_error >> 8) + (id_integral >> 15));
    q15_t vq = (q15_t)((g_control_config.current_kp * iq_error >> 8) + (iq_integral >> 15));
    
    g_foc_state.vd = vd;
    g_foc_state.vq = vq;
    
    // Inverse Park transform (dq -> alpha-beta)
    q15_t v_alpha = (q15_t)(((int32_t)vd * cos_theta - (int32_t)vq * sin_theta) >> 15);
    q15_t v_beta = (q15_t)(((int32_t)vd * sin_theta + (int32_t)vq * cos_theta) >> 15);
    
    // SVPWM generation
    uint16_t duty[3];
    svpwm_calculate(v_alpha, v_beta, v_bus, duty);
    
    // Apply PWM
    hw_set_pwm(duty);
}

/*============================================================================*/
/* Command Setters                                                            */
/*============================================================================*/

void foc_set_control_mode(control_mode_e mode) {
    g_ctrl_mode = mode;
}

void foc_set_current_command(q15_t current) {
    iq_target = current;
    id_target = 0;
}

void foc_set_speed_command(int16_t speed_rpm) {
    speed_target = speed_rpm;
}

void foc_set_position_command(q31_t position) {
    // TODO: Implement position control
    (void)position;
}

void foc_set_duty_command(q15_t duty) {
    // TODO: Implement duty control
    (void)duty;
}

/*============================================================================*/
/* State Getters                                                              */
/*============================================================================*/

foc_state_t* foc_get_state(void) {
    return &g_foc_state;
}

int16_t foc_get_motor_speed(void) {
    return g_foc_state.speed_rpm;
}

q15_t foc_get_motor_angle(void) {
    return g_foc_state.fusion_angle;
}

bool foc_is_motor_running(void) {
    return g_mc_state == MC_STATE_RUNNING;
}

uint16_t foc_get_fault_code(void) {
    return g_foc_state.fault_code;
}

void foc_clear_fault(void) {
    g_foc_state.fault_code = FAULT_NONE;
    g_mc_state = MC_STATE_IDLE;
}
