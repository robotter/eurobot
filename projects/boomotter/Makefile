include ../../config.mk

## Project configuration

SRCS = $(filter-out $(wildcard *.inc.c),$(wildcard *.c))
ASRCS =
TARGET = main
MODULES = uart rome
GEN_FILES = modules/font_bitmap.inc.c


## Target configuration

MCU = atxmega64d4


## Build configuration

OPT = s
# link against math library (may be implied by some modules)
MATH_LIB = yes
# printf level: minimal, standard, advanced
PRINTF_LEVEL = standard

CFLAGS += -g -Wextra -Werror
LDFLAGS += -g

include $(AVARIX_DIR)/mk/project.mk

$(gen_dir)/modules/font_bitmap.inc.c: gen_font_bitmap.py
	./$< > $@

