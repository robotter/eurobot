include ../../config.mk

## Project configuration

SRCS = $(wildcard *.c)
ASRCS =
TARGET = main
MODULES =

MCU = attiny2313


## Build configuration

OPT = s
# link against math library (may be implied by some modules)
MATH_LIB = yes
# printf level: minimal, standard, advanced
PRINTF_LEVEL = standard

CFLAGS += -g -Wextra -Werror
LDFLAGS += -g

include $(AVARIX_DIR)/mk/project.mk

