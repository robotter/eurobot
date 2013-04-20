
## Project configuration

SRCS = $(wildcard *.c)
ASRCS =
TARGET = main
MODULES = uart pwm perlimpinpin ax12 timer encoder/aeat


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

ROOM_TRANSACTIONS = ../../room_transactions.py
export ROOM_TRANSACTIONS

include ../../avarix/mk/project.mk

