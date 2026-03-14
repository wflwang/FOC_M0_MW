/**
 * @file foc_observer.h
 * @brief MXLemming Observer Header
 */

#ifndef FOC_OBSERVER_H
#define FOC_OBSERVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "types.h"
#include "foc_config.h"

/**
 * @brief Initialize observer with motor configuration
 */
void observer_init(const motor_config_t *config);

/**
 * @brief Update observer with current measurements
 * @param id d-axis current (Q15)
 * @param iq q-axis current (Q15)
 * @param v_bus Bus voltage (Q8.8)
 */
void observer_update(q15_t id, q15_t iq, q15_t v_bus);

/**
 * @brief Get estimated angle
 * @return Electrical angle (Q15, 0-65535)
 */
q15_t observer_get_angle(void);

/**
 * @brief Get estimated speed
 * @return Electrical speed (Q15, rad/s normalized)
 */
q15_t observer_get_speed(void);

/**
 * @brief Reset observer state
 */
void observer_reset(void);

#ifdef __cplusplus
}
#endif

#endif /* FOC_OBSERVER_H */
