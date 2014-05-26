
## Project configuration

SRCS = $(wildcard *.c)
ASRCS =
TARGET = main
MODULES = uart clock timer rome

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

export ROME_MESSAGES=../../rome_messages.py

include ../../avarix/mk/project.mk

