include ../../config.mk

## Project configuration

SRCS = $(wildcard *.c)
ASRCS =
TARGET = main
MODULES = uart clock timer idle rome
R = galipeur

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

ifeq ($(R),$(filter $(R),galipeur))
CFLAGS += -DGALIPEUR
else ifeq ($(R),$(filter $(R),galipette pmi))
CFLAGS += -DGALIPETTE
else
$(error invalid robot (R variable))
endif

include $(AVARIX_DIR)/mk/project.mk

