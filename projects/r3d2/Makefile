
## Project configuration

SRCS = $(wildcard *.c)
ASRCS =
TARGET = main
MODULES = uart pwm timer rome


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

ROME_MESSAGES = ../../rome_messages.py
export ROME_MESSAGES

include ../../avarix/mk/project.mk

