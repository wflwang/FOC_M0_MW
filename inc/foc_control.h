/**
 * @file foc_control.h
 * @brief FOC Main Control Loop Header
 */

#ifndef FOC_CONTROL_H
#define FOC_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "types.h"
#include "foc_config.h"

/*============================================================================*/
/* Control Mode Enumeration                                                   */
/*============================================================================*/

typedef enum {
    MC_STATE_OFF = 0,
    MC_STATE_IDLE,
    MC_STATE_DETECTING,
    MC_STATE_RUNNING,
    MC_STATE_FAULT,
    MC_STATE_BRAKE,
} mc_state_e;

/*============================================================================*/
/* Function Declarations                                                      */
/*============================================================================*/

/**
 * @brief Initialize FOC controller with motor and control configurations
 */
void foc_control_init(const motor_config_t *motor_cfg, const control_config_t *ctrl_cfg);

/**
 * @brief Start motor control
 */
void foc_control_start(void);

/**
 * @brief Stop motor control
 */
void foc_control_stop(void);

/**
 * @brief Main control loop - call from PWM interrupt
 */
void foc_control_run(void);

/**
 * @brief Set control mode
 */
void foc_set_control_mode(control_mode_e mode);

/**
 * @brief Set current (torque) command
 * @param current Target current in Q12.4 format (A)
 */
void foc_set_current_command(q15_t current);

/**
 * @brief Set speed command
 * @param speed_rpm Target speed in RPM
 */
void foc_set_speed_command(int16_t speed_rpm);

/**
 * @brief Set position command
 * @param position Target position in Q16.16 format (turns)
 */
void foc_set_position_command(q31_t position);

/**
 * @brief Set duty cycle command
 * @param duty Duty cycle (0-32767)
 */
void foc_set_duty_command(q15_t duty);

/**
 * @brief Get current FOC state
 */
foc_state_t* foc_get_state(void);

/**
 * @brief Get motor speed in RPM
 */
int16_t foc_get_motor_speed(void);

/**
 * @brief Get motor angle in electrical radians
 */
q15_t foc_get_motor_angle(void);

/**
 * @brief Check if motor is running
 */
bool foc_is_motor_running(void);

/**
 * @brief Get fault code
 */
uint16_t foc_get_fault_code(void);

/**
 * @brief Clear fault
 */
void foc_clear_fault(void);

#ifdef __cplusplus
}
#endif

#endif /* FOC_CONTROL_H */
