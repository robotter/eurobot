
## Project configuration

SRCS = $(wildcard *.c) rgb_led.c tcs3772x.c apds9800.c battery_monitor.c adc.c
ASRCS =
TARGET = main
MODULES = uart pwm timer i2c clock


## Target configuration

MCU = atxmega32d4


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

include ../../../avarix/mk/project.mk

