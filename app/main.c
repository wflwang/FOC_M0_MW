/**
 * @file main.c
 * @brief Main application entry point for HK32M070 FOC VESC
 */

#include "hw.h"
#include "foc_control.h"
#include "foc_motor_detect.h"
#include <string.h>

/*============================================================================*/
/* Global Variables                                                           */
/*============================================================================*/

// Motor configuration
motor_config_t motor_config = {
    .pole_pairs = 7,
    .resistance = 0.05f,        // 50mΩ
    .inductance_ld = 0.00005f,  // 50µH
    .inductance_lq = 0.00005f,
    .flux_linkage = 0.01f,      // 10mWb
    .max_current = 30.0f,
    .rated_current = 15.0f,
    .max_speed = 10000,         // 10000 RPM
};

// Control configuration
control_config_t control_config = {
    .ctrl_mode = CTRL_MODE_SPEED,
    .current_kp = 0.5f,
    .current_ki = 50.0f,
    .speed_kp = 0.005f,
    .speed_ki = 0.1f,
    .observer_gain = 1000.0f,
    .hall_blend_start_speed = 300,
    .hall_blend_end_speed = 800,
};

// Runtime state
static volatile uint32_t control_loop_counter = 0;
static volatile bool motor_running = false;
static volatile bool fault_active = false;

/*============================================================================*/
/* Private Functions                                                          */
/*============================================================================*/

/**
 * @brief Initialize all subsystems
 */
static void init_subsystems(void) {
    // Initialize hardware
    hw_init();
    
    // Initialize FOC controller
    foc_control_init(&motor_config, &control_config);
    
    // Enable LED
    hw_set_led(1);
    hw_delay_ms(100);
    hw_set_led(0);
}

/**
 * @brief Run motor parameter detection
 */
static void run_motor_detection(void) {
    // Blink LED to indicate detection mode
    for (int i = 0; i < 5; i++) {
        hw_set_led(1);
        hw_delay_ms(100);
        hw_set_led(0);
        hw_delay_ms(100);
    }
    
    // Run detection sequence
    motor_detect_result_t result;
    if (foc_motor_detect_run(&result)) {
        // Update motor configuration with detected values
        motor_config.resistance = result.resistance;
        motor_config.inductance_ld = result.inductance_ld;
        motor_config.inductance_lq = result.inductance_lq;
        motor_config.flux_linkage = result.flux_linkage;
        
        // Long blink to indicate success
        hw_set_led(1);
        hw_delay_ms(1000);
        hw_set_led(0);
    }
}

/**
 * @brief Check for faults
 */
static bool check_faults(void) {
    q15_t bus_voltage = hw_get_bus_voltage();
    
    // Over-voltage check (Q8.8 format, 55V = 14080)
    if (bus_voltage > 14080) {
        return true;
    }
    
    // Under-voltage check (15V = 3840)
    if (bus_voltage < 3840) {
        return true;
    }
    
    // Temperature check (if available)
#ifdef HW_HAS_TEMP_SENSOR
    q15_t temp = hw_get_temperature();
    // Over-temperature (85°C = 21760 in Q8.8)
    if (temp > 21760) {
        return true;
    }
#endif
    
    // Hardware fault pin
    if (hw_check_fault()) {
        return true;
    }
    
    return false;
}

/**
 * @brief Control loop (called from timer interrupt)
 */
void control_loop_callback(void) {
    control_loop_counter++;
    
    // Check faults
    if (check_faults()) {
        fault_active = true;
        foc_control_stop();
        hw_pwm_enable(false);
        return;
    }
    
    // Run FOC control iteration
    foc_control_run();
    
    // LED toggle every 1000 iterations (indicates running)
    if ((control_loop_counter % 1000) == 0) {
        hw_set_led(2);  // Toggle
    }
}

/*============================================================================*/
/* Main Function                                                              */
/*============================================================================*/

int main(void) {
    // Initialize all subsystems
    init_subsystems();
    
    // Wait for stable power
    hw_delay_ms(100);
    
    // Optionally run motor detection on startup (hold button)
    if (hw_button_pressed()) {
        run_motor_detection();
    }
    
    // Main loop
    while (1) {
        static uint32_t last_millis = 0;
        uint32_t now = hw_get_millis();
        
        // Check for fault recovery
        if (fault_active) {
            if (!check_faults()) {
                // Clear fault after 1 second
                if ((now - last_millis) > 1000) {
                    fault_active = false;
                    hw_set_led(0);
                }
            }
            last_millis = now;
            hw_delay_ms(10);
            continue;
        }
        
        // Check button for start/stop
        if (hw_button_pressed()) {
            hw_delay_ms(50);  // Debounce
            if (hw_button_pressed()) {
                if (motor_running) {
                    // Stop motor
                    foc_control_stop();
                    hw_pwm_enable(false);
                    motor_running = false;
                    hw_set_led(0);
                } else {
                    // Start motor
                    hw_pwm_enable(true);
                    foc_control_start();
                    motor_running = true;
                }
                
                // Wait for button release
                while (hw_button_pressed()) {
                    hw_delay_ms(10);
                }
            }
        }
        
        // Low-priority tasks (100Hz)
        if ((now - last_millis) >= 10) {
            last_millis = now;
            
            // Update speed command (example: potentiometer or serial input)
            // Here we just set a fixed speed for demonstration
            if (motor_running) {
                foc_set_speed_command(2000);  // 2000 RPM target
            }
        }
        
        // Small delay to prevent tight loop
        hw_delay_ms(1);
    }
    
    return 0;
}

/*============================================================================*/
/* Interrupt Handlers                                                         */
/*============================================================================*/

/**
 * @brief TIM1 Update interrupt (PWM period)
 * @note This is called at PWM frequency (20kHz)
 */
void TIM1_BRK_UP_TRG_COM_IRQHandler(void) {
    if (TIM1->SR & 0x01) {  // Update interrupt flag
        TIM1->SR &= ~0x01;  // Clear flag
        
        // Call control loop
        control_loop_callback();
    }
}

/**
 * @brief ADC interrupt (optional, for current sampling)
 */
void ADC1_COMP_IRQHandler(void) {
    // Handle ADC conversion complete
    if (ADC1->ISR & 0x04) {  // EOC flag
        ADC1->ISR = 0x04;
    }
}

/**
 * @brief Hard fault handler
 */
void HardFault_Handler(void) {
    while (1) {
        // Rapid LED blink to indicate hard fault
        hw_set_led(1);
        for (volatile int i = 0; i < 100000; i++);
        hw_set_led(0);
        for (volatile int i = 0; i < 100000; i++);
    }
}
