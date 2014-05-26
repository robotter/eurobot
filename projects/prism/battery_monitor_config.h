#ifndef BATTERY_MONITOR_CONFIG_H
#define BATTERY_MONITOR_CONFIG_H

#include "battery_monitor.h"

#define BATTERY_TYPE  LIPO

#define BATTERY_SIZE  LIPO_2S 

/*@brief if it is a lead battery, define its nominal voltage (6V 12V, 24V)
 * if not, undef this define
 */
//#define BATTERY_PB_VOLTAGE_V  12 

#define BATTERY_ADC_PORTPIN PORTPIN(A,2)
#define BATTERY_ADC   ADCA

#define BATTERY_VOLT_DIVIDER_GAIN                   7.47f/0.475f

/* @brief define BATTERY_LED_ALARM_PORTPIN if a led is defined to display battery alarm problems
 * if not defined, the program will simply avoid signalling low level battery with led
 */
#define BATTERY_LED_ALARM_PORTPIN PORTPIN(E,1)


/* @brief coefficient of the low pass IIR filter that filters the adc value 
 *  must be lower than 1.00 (good values : 0.01 to 0.3)
 */
#define BATTERY_LOW_PASS_FILTER_COEFF   0.18f

#define BATTMON_UPDATE_PERIOD_US  500000UL
#define BATTMON_UPDATE_INTLVL  INTLVL_LO

#endif //BATTERY_MONITOR_CONFIG_H 
