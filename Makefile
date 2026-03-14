#==============================================================================#
# Makefile for HK32M070 FOC VESC Firmware                                      #
# Target: ARM Cortex-M0 (HK32M070)                                             #
#==============================================================================#

#-------------------------------------------------------------------------------
# Configuration
#-------------------------------------------------------------------------------

# Default hardware target (can be overridden: make HW=hw_hk32m070_qfn48)
HW ?= hw_hk32m070_qfn32

# Build type: DEBUG or RELEASE
BUILD ?= RELEASE

# Project name
PROJECT = hk32m070_foc_vesc

# Output directory
OUT_DIR = build/$(HW)

#-------------------------------------------------------------------------------
# Toolchain
#-------------------------------------------------------------------------------

# ARM GCC toolchain prefix
PREFIX = arm-none-eabi-

# Tools
CC      = $(PREFIX)gcc
AS      = $(PREFIX)as
LD      = $(PREFIX)ld
OBJCOPY = $(PREFIX)objcopy
OBJDUMP = $(PREFIX)objdump
SIZE    = $(PREFIX)size

#-------------------------------------------------------------------------------
# MCU Configuration
#-------------------------------------------------------------------------------

# CPU and architecture
CPU     = -mcpu=cortex-m0
ARCH    = -mthumb

# FPU (M0 has no FPU)
FPU     =

# C standard
C_STD   = -std=c99

#-------------------------------------------------------------------------------
# Directories
#-------------------------------------------------------------------------------

# Source directories
SRC_DIRS = app \
           src/foc \
           src/utils \
           driver/hk32m070 \
           startup

# Include directories
INC_DIRS = inc \
           hwconf \
           hwconf/$(HW) \
           driver/hk32m070

#-------------------------------------------------------------------------------
# Hardware Definition
#-------------------------------------------------------------------------------

# Map hardware name to definition
ifeq ($(HW),hw_hk32m070_qfn32)
    HW_DEF = HW_HK32M070_QFN32
    LDSCRIPT = ld/hk32m070_qfn32.ld
else ifeq ($(HW),hw_hk32m070_qfn48)
    HW_DEF = HW_HK32M070_QFN48
    LDSCRIPT = ld/hk32m070_qfn48.ld
else
    $(error Unknown hardware: $(HW))
endif

#-------------------------------------------------------------------------------
# Compiler Flags
#-------------------------------------------------------------------------------

# Common flags
CFLAGS_COMMON = $(CPU) $(ARCH) $(FPU) $(C_STD)
CFLAGS_COMMON += -ffunction-sections -fdata-sections
CFLAGS_COMMON += -Wall -Wextra -Wshadow -Wredundant-decls
CFLAGS_COMMON += -fno-common -fno-builtin-printf
CFLAGS_COMMON += -ffreestanding -nostdlib

# Include paths
CFLAGS_COMMON += $(addprefix -I,$(INC_DIRS))

# Defines
CFLAGS_COMMON += -D$(HW_DEF)
CFLAGS_COMMON += -DUSE_HAL_DRIVER
CFLAGS_COMMON += -DARM_MATH_CM0

# Debug/Release specific flags
ifeq ($(BUILD),DEBUG)
    CFLAGS = $(CFLAGS_COMMON) -O0 -g3 -DDEBUG
    OPT_LEVEL = -O0 -g3
else
    CFLAGS = $(CFLAGS_COMMON) -Os -g0 -DNDEBUG
    OPT_LEVEL = -Os
endif

#-------------------------------------------------------------------------------
# Assembler Flags
#-------------------------------------------------------------------------------

ASFLAGS = $(CPU) $(ARCH) -g

#-------------------------------------------------------------------------------
# Linker Flags
#-------------------------------------------------------------------------------

LDFLAGS  = $(CPU) $(ARCH) $(FPU)
LDFLAGS += -T$(LDSCRIPT)
LDFLAGS += -nostartfiles -nostdlib
LDFLAGS += -Wl,--gc-sections
LDFLAGS += -Wl,-Map=$(OUT_DIR)/$(PROJECT).map
LDFLAGS += -Wl,--print-memory-usage
LDFLAGS += -specs=nano.specs -specs=nosys.specs
LDFLAGS += -Wl,--start-group -lc -lm -Wl,--end-group

#-------------------------------------------------------------------------------
# Source Files
#-------------------------------------------------------------------------------

# C sources
C_SOURCES = $(wildcard app/*.c)
C_SOURCES += $(wildcard src/foc/*.c)
C_SOURCES += $(wildcard src/utils/*.c)
C_SOURCES += $(wildcard driver/hk32m070/*.c)
C_SOURCES += hwconf/$(HW)/$(HW).c

# Assembly sources
ASM_SOURCES = startup/startup_hk32m070.s

#-------------------------------------------------------------------------------
# Object Files
#-------------------------------------------------------------------------------

# Generate object file names
OBJECTS = $(addprefix $(OUT_DIR)/, $(C_SOURCES:.c=.o))
OBJECTS += $(addprefix $(OUT_DIR)/, $(ASM_SOURCES:.s=.o))

# Dependencies
DEPS = $(OBJECTS:.o=.d)

#-------------------------------------------------------------------------------
# Targets
#-------------------------------------------------------------------------------

.PHONY: all clean flash size info

all: $(OUT_DIR)/$(PROJECT).elf $(OUT_DIR)/$(PROJECT).hex $(OUT_DIR)/$(PROJECT).bin
	@echo ""
	@echo "Build complete: $(HW)"
	@$(SIZE) $(OUT_DIR)/$(PROJECT).elf

#-------------------------------------------------------------------------------
# Create output directory
#-------------------------------------------------------------------------------

$(OUT_DIR):
	@mkdir -p $(OUT_DIR)
	@mkdir -p $(OUT_DIR)/app
	@mkdir -p $(OUT_DIR)/src/foc
	@mkdir -p $(OUT_DIR)/src/utils
	@mkdir -p $(OUT_DIR)/driver/hk32m070
	@mkdir -p $(OUT_DIR)/hwconf/$(HW)
	@mkdir -p $(OUT_DIR)/startup

#-------------------------------------------------------------------------------
# Compile C files
#-------------------------------------------------------------------------------

$(OUT_DIR)/%.o: %.c | $(OUT_DIR)
	@echo "CC: $<"
	@$(CC) $(CFLAGS) -MMD -MP -c $< -o $@

#-------------------------------------------------------------------------------
# Assemble ASM files
#-------------------------------------------------------------------------------

$(OUT_DIR)/%.o: %.s | $(OUT_DIR)
	@echo "AS: $<"
	@$(AS) $(ASFLAGS) -o $@ $<

#-------------------------------------------------------------------------------
# Link
#-------------------------------------------------------------------------------

$(OUT_DIR)/$(PROJECT).elf: $(OBJECTS)
	@echo "LD: $@"
	@$(CC) $(LDFLAGS) $(OBJECTS) -o $@

#-------------------------------------------------------------------------------
# Generate binary and hex files
#-------------------------------------------------------------------------------

$(OUT_DIR)/$(PROJECT).hex: $(OUT_DIR)/$(PROJECT).elf
	@echo "HEX: $@"
	@$(OBJCOPY) -O ihex $< $@

$(OUT_DIR)/$(PROJECT).bin: $(OUT_DIR)/$(PROJECT).elf
	@echo "BIN: $@"
	@$(OBJCOPY) -O binary $< $@

#-------------------------------------------------------------------------------
# Size report
#-------------------------------------------------------------------------------

size: $(OUT_DIR)/$(PROJECT).elf
	@$(SIZE) -A -x $<

#-------------------------------------------------------------------------------
# Show build info
#-------------------------------------------------------------------------------

info:
	@echo "Project: $(PROJECT)"
	@echo "Hardware: $(HW)"
	@echo "Hardware Define: $(HW_DEF)"
	@echo "Build Type: $(BUILD)"
	@echo "Linker Script: $(LDSCRIPT)"
	@echo "Output Directory: $(OUT_DIR)"
	@echo ""
	@echo "Source Files:"
	@for f in $(C_SOURCES); do echo "  $$f"; done

#-------------------------------------------------------------------------------
# Flash (using OpenOCD)
#-------------------------------------------------------------------------------

flash: $(OUT_DIR)/$(PROJECT).elf
	openocd -f interface/stlink.cfg -f target/hk32m070.cfg \
		-c "program $< verify reset exit"

#-------------------------------------------------------------------------------
# Flash using J-Link
#-------------------------------------------------------------------------------

jlink: $(OUT_DIR)/$(PROJECT).hex
	JLink.exe -device HK32M070 -if SWD -speed 4000 \
		-commandfile jlink/flash.jlink

#-------------------------------------------------------------------------------
# Clean
#-------------------------------------------------------------------------------

clean:
	@echo "Cleaning build files..."
	@rm -rf build
	@echo "Done."

#-------------------------------------------------------------------------------
# Help
#-------------------------------------------------------------------------------

help:
	@echo "HK32M070 FOC VESC Build System"
	@echo ""
	@echo "Usage: make [target] [options]"
	@echo ""
	@echo "Targets:"
	@echo "  all     - Build firmware (default)"
	@echo "  clean   - Remove build files"
	@echo "  flash   - Flash using OpenOCD"
	@echo "  jlink   - Flash using J-Link"
	@echo "  size    - Show firmware size"
	@echo "  info    - Show build configuration"
	@echo "  help    - Show this help message"
	@echo ""
	@echo "Options:"
	@echo "  HW=xxx       - Hardware target"
	@echo "                 hw_hk32m070_qfn32 (default)"
	@echo "                 hw_hk32m070_qfn48"
	@echo ""
	@echo "  BUILD=xxx    - Build type"
	@echo "                 RELEASE (default)"
	@echo "                 DEBUG"
	@echo ""
	@echo "Examples:"
	@echo "  make                              # Build for QFN32"
	@echo "  make HW=hw_hk32m070_qfn48         # Build for QFN48"
	@echo "  make BUILD=DEBUG                  # Debug build"
	@echo "  make clean all HW=hw_hk32m070_qfn48 BUILD=DEBUG"

# Include dependency files
-include $(DEPS)
