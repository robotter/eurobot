include ../../config.mk

## Project configuration

SRCS = main.c arms.c barometer.c stepper_motor.c servo_hat.c
ASRCS =
TARGET = main
MODULES = i2c uart timer rome idle


## Target configuration

MCU = atxmega128a1


## Build configuration

OPT = s
# link against math library (may be implied by some modules)
MATH_LIB = yes
# printf level: minimal, standard, advanced
PRINTF_LEVEL = standard

CFLAGS += -g -Wextra -Werror
LDFLAGS += -g

include $(AVARIX_DIR)/mk/project.mk

