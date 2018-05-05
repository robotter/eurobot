include ../../config.mk

## Project configuration

SRCS = main.c servos.c cylinder.c jevois_cam.c
ASRCS =
TARGET = main
MODULES = uart pwm ax12 timer rome idle


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

