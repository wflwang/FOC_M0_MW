/**
 * @file hw.h
 * @brief Hardware Abstraction Layer - Main Header
 * @author FOC VESC Team
 * @date 2024
 * 
 * This file includes the appropriate hardware configuration based on
 * the HW_DEF macro defined in the build configuration.
 */

#ifndef HW_H
#define HW_H

#ifdef __cplusplus
extern "C" {
#endif

#include "types.h"

/*============================================================================*/
/* Hardware Selection - Choose one hardware definition                        */
/*============================================================================*/

// Default hardware (can be overridden by Makefile or compiler option)
#ifndef HW_DEF
#define HW_DEF          HW_HK32M070_QFN32
#endif

/*============================================================================*/
/* Include Hardware Specific Header                                           */
/*============================================================================*/

#if HW_DEF == HW_HK32M070_QFN32
    #include "hw_hk32m070_qfn32.h"
#elif HW_DEF == HW_HK32M070_QFN48
    #include "hw_hk32m070_qfn48.h"
#else
    #error "Unknown hardware definition!"
#endif

/*============================================================================*/
/* Common Hardware Interface                                                  */
/*============================================================================*/

/**
 * @brief Initialize all hardware peripherals
 */
void hw_init(void);

/**
 * @brief Get ADC readings for current sensing
 * @param phase Pointer to store phase currents [Ia, Ib, Ic]
 */
void hw_get_currents(q15_t *phase);

/**
 * @brief Get DC bus voltage (scaled to fixed point)
 * @return Bus voltage in Q8.8 format (e.g., 48.0V = 12288)
 */
q15_t hw_get_bus_voltage(void);

/**
 * @brief Get temperature reading
 * @return Temperature in Q8.8 format (e.g., 25.0°C = 6400)
 */
q15_t hw_get_temperature(void);

/**
 * @brief Get hall sensor state
 * @return Hall state (0-7)
 */
uint8_t hw_get_hall_state(void);

/**
 * @brief Set PWM duty cycles for three phases
 * @param duty Pointer to duty cycles (0-65535 maps to 0-100%)
 */
void hw_set_pwm(uint16_t *duty);

/**
 * @brief Enable/disable PWM output
 * @param enable true to enable, false to disable
 */
void hw_pwm_enable(bool enable);

/**
 * @brief Get current timer value for angle interpolation
 * @return Timer counter value
 */
uint16_t hw_get_timer_value(void);

/**
 * @brief Set LED state
 * @param state LED state (0=off, 1=on, 2=toggle)
 */
void hw_set_led(uint8_t state);

/**
 * @brief Check if button is pressed
 * @return true if button pressed
 */
bool hw_button_pressed(void);

/**
 * @brief Delay in microseconds (blocking)
 * @param us Microseconds to delay
 */
void hw_delay_us(uint32_t us);

/**
 * @brief Delay in milliseconds (blocking)
 * @param ms Milliseconds to delay
 */
void hw_delay_ms(uint32_t ms);

/**
 * @brief Get microsecond timestamp
 * @return Microseconds since boot
 */
uint32_t hw_get_micros(void);

/**
 * @brief Get millisecond timestamp
 * @return Milliseconds since boot
 */
uint32_t hw_get_millis(void);

/*============================================================================*/
/* Hardware Configuration Query Functions                                     */
/*============================================================================*/

/**
 * @brief Check if hardware has hall sensors
 */
static inline bool hw_has_hall_sensors(void) {
#ifdef HW_HAS_HALL
    return true;
#else
    return false;
#endif
}

/**
 * @brief Check if hardware has temperature sensor
 */
static inline bool hw_has_temp_sensor(void) {
#ifdef HW_HAS_TEMP_SENSOR
    return true;
#else
    return false;
#endif
}

/**
 * @brief Check if hardware has CAN bus
 */
static inline bool hw_has_can(void) {
#ifdef HW_HAS_CAN
    return true;
#else
    return false;
#endif
}

#ifdef __cplusplus
}
#endif

#endif /* HW_H */
