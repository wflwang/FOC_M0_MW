# HK32M070 FOC Motor Controller - VESC Style

## Overview

A professional FOC (Field Oriented Control) motor controller firmware for HK32M070 QFN32 MCU, following VESC architecture patterns.

## Features

- **Full FOC Control** with Hall sensor and observer fusion
- **MXLemming Observer** optimized for M0 core with fixed-point arithmetic
- **Automatic Parameter Detection**: Resistance, Inductance, Flux Linkage
- **Smooth Control**: Anti-jitter algorithms, fast direction change
- **Field Weakening**: Extended speed range support
- **Regenerative Braking Management**: Prevents bus voltage spikes
- **Temperature Compensation**: Motor resistance adjustment

## Hardware Specifications

- **MCU**: HK32M070 QFN32
- **Core**: ARM Cortex-M0
- **Clock**: 64MHz
- **Motor**: Single PMSM/BLDC with Hall sensors

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Application Layer                         │
├─────────────────────────────────────────────────────────────┤
│  Control Modes: Current | Speed | Position | Duty           │
├─────────────────────────────────────────────────────────────┤
│                    FOC Core Engine                           │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────┐  │
│  │   Hall      │  │  Observer   │  │   Angle Fusion      │  │
│  │   Sensor    │  │ MXLemming   │  │   (Speed-based)     │  │
│  └─────────────┘  └─────────────┘  └─────────────────────┘  │
│  ┌─────────────────────────────────────────────────────────┐│
│  │            Current Controller (PI with Decoupling)      ││
│  └─────────────────────────────────────────────────────────┘│
│  ┌─────────────────────────────────────────────────────────┐│
│  │            SVPWM Generator (Space Vector Modulation)    ││
│  └─────────────────────────────────────────────────────────┘│
├─────────────────────────────────────────────────────────────┤
│                    HAL Layer                                 │
│  ADC | PWM | GPIO | Timer | UART                            │
└─────────────────────────────────────────────────────────────┘
```

## Angle Fusion Strategy

The system uses a smooth transition between Hall sensor and Observer:

```
Speed (ERPM)
    │
    │         Observer Only
    │    ┌───────────────────────
    │    │  Transition Zone
    │    │   (Weighted Blend)
    │────┤───────────────────────  foc_sl_erpm
    │    │  Hall Dominant
    │────┤───────────────────────  foc_sl_erpm_start
    │    │  Hall Only
    │    │
    └────┴───────────────────────> Speed
```

## Fixed-Point Arithmetic

To optimize for M0 core without FPU, the system uses Q15/Q31 fixed-point:

| Parameter | Format | Range | Resolution |
|-----------|--------|-------|------------|
| Current | Q15 | -16A to +16A | 0.0005A |
| Voltage | Q15 | 0-60V | 0.001V |
| Angle | Q16 | 0-2π | 0.0001 rad |
| Flux | Q20 | 0-0.1 Wb | 0.0000001 Wb |

## Directory Structure

```
HK32M070_FOC_VESC/
├── src/
│   ├── foc/
│   │   ├── foc_core.c/h          # Main FOC algorithm
│   │   ├── foc_observer.c/h      # MXLemming observer
│   │   ├── foc_hall.c/h          # Hall sensor processing
│   │   ├── foc_angle_fusion.c/h  # Angle fusion logic
│   │   ├── foc_current_ctrl.c/h  # Current PI controller
│   │   ├── foc_svpwm.c/h         # Space vector modulation
│   │   └── foc_auto_detect.c/h   # Parameter detection
│   ├── control/
│   │   ├── speed_ctrl.c/h        # Speed PID controller
│   │   ├── position_ctrl.c/h     # Position PID controller
│   │   └── trajectory.c/h        # Motion trajectory
│   ├── motor/
│   │   ├── motor_interface.c/h   # Motor control interface
│   │   ├── motor_config.c/h      # Configuration management
│   │   └── motor_protection.c/h  # Fault protection
│   ├── hal/
│   │   ├── hal_adc.c/h           # ADC driver
│   │   ├── hal_pwm.c/h           # PWM driver
│   │   ├── hal_timer.c/h         # Timer driver
│   │   └── hal_gpio.c/h          # GPIO driver
│   └── utils/
│       ├── fixed_point.c/h       # Fixed-point math
│       ├── filter.c/h            # Digital filters
│       └── utils.c/h             # Utility functions
├── inc/
│   ├── config.h                  # System configuration
│   ├── types.h                   # Data types
│   └── constants.h               # Physical constants
├── driver/
│   └── HK32M070/                 # MCU driver library
└── docs/
    └── architecture.md           # Detailed documentation
```

## Building

```bash
# Using ARM GCC toolchain
arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -O2 -I./inc -I./driver src/main.c -o build/firmware.elf
```

## License

GPL v3 (following VESC license)

## Credits

Based on VESC firmware by Benjamin Vedder
MXLemming observer by David Molony (MESC project)
