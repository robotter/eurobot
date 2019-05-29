include ../../config.mk

## Project configuration

SRCS = $(filter-out $(wildcard *.inc.c),$(wildcard *.c))
ASRCS =
TARGET = main
MODULES = uart xbee idle rome timer
GEN_FILES = modules/resources.inc.c


## Target configuration

MCU = atxmega64a4u


## Build configuration

OPT = s
# link against math library (may be implied by some modules)
MATH_LIB = yes
# printf level: minimal, standard, advanced
PRINTF_LEVEL = standard

CFLAGS += -g -Wextra -Werror
LDFLAGS += -g

include $(AVARIX_DIR)/mk/project.mk

$(gen_dir)/modules/resources.inc.c: gen_resources.py
	./$< > $@ || (rm $@; false)

