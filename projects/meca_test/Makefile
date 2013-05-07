
## Project configuration

SRCS = $(wildcard *.c) ../meca/acm.c
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
PRINTF_LEVEL = advanced

CFLAGS += -g -Wextra -Werror
LDFLAGS += -g

include ../../avarix/mk/project.mk

