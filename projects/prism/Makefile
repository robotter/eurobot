include ../../config.mk

## Project configuration

SRCS = $(filter-out avoidance%, $(wildcard *.c))
ASRCS =
TARGET = main
MODULES = uart clock pwm timer

## Target configuration
MCU = atxmega128a1

## Build configuration

OPT = 3
# link against math library (may be implied by some modules)
MATH_LIB = yes
# printf level: minimal, standard, advanced
PRINTF_LEVEL = advanced
#PRINTF_LEVEL = standard

CFLAGS += -g -Wextra -Werror
LDFLAGS += -g

include $(AVARIX_DIR)/mk/project.mk

