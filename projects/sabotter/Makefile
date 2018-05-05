include ../../config.mk

## Project configuration
ifeq ($(R),)
$(warning Define robot, with make R=XXX)
else
ifeq ($(R),$(filter $(R),galipeur G))
CFLAGS += -DGALIPEUR
TARGET = galipeur
obj_dir = build_galipeur
endif
ifeq ($(R),$(filter $(R),galipette pmi g))
CFLAGS += -DGALIPETTE
TARGET = galipette
obj_dir = build_galipette
endif
endif

SRCS = $(filter-out $(wildcard *.inc.c), $(wildcard *.c))
ASRCS =
MODULES = uart clock timer idle rome pathfinding

## Target configuration
MCU = atxmega256a3

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

