
## Project configuration

SRCS = $(wildcard *.c)
ASRCS =
TARGET = main
MODULES = uart clock pwm ax12 timer encoder/aeat

## Target configuration
MCU = atxmega128a1

## Build configuration

OPT = 0
# link against math library (may be implied by some modules)
MATH_LIB = yes
# printf level: minimal, standard, advanced
PRINTF_LEVEL = standard

CFLAGS += -g -Wextra -Werror
LDFLAGS += -g

include ../../../robotter.avarix/mk/project.mk

