#ifndef BATTERY_MONITOR_CONFIG_H
#define BATTERY_MONITOR_CONFIG_H

#include "battery_monitor.h"

#define BATTERY_TYPE  LIPO

#define BATTERY_SIZE  LIPO_4S 

/*@brief if it is a lead battery, define its nominal voltage (6V 12V, 24V)
 * if not, undef this define
 */
//#define BATTERY_PB_VOLTAGE_V  12 

#define BATTERY_ADC_PORTPIN PORTPIN(B,7)
#define BATTERY_ADC   ADCB
#define BATTERY_ADC_MUX   ADC_CH_MUXPOS_PIN7_gc

/* @brief battery monitoring is performed by monitoring the voltage of the battery (lipo or other).
 * the adc input may have a divider bridge made of 2 resistors.
 * the resistor between the adc pin and the ground is called BATTERY_VOLT_DIVIDER_GROUND_RESISTOR_OHMS.
 * The resistor between the adc pin and the battery voltage is called BATTERY_VOLT_DIVIDER_BATT_RESISTOR_OHMS
 * these defines shall contain the value in ohms of each resistor
 */
#define BATTERY_VOLT_DIVIDER_GROUND_RESISTOR_OHMS   15000.0
#define BATTERY_VOLT_DIVIDER_BATT_RESISTOR_OHMS     1050.0

/* @brief define BATTERY_LED_ALARM_PORTPIN if a led is defined to display battery alarm problems
 * if not defined, the program will simply avoid signalling low level battery with led
 */
//#define BATTERY_LED_ALARM_PORTPIN PORTPIN(C,2)


/* @brief coefficient of the low pass IIR filter that filters the adc value 
 *  must be lower than 1.00 (good values : 0.01 to 0.3)
 */
#define BATTERY_LOW_PASS_FILTER_COEFF   0.2f

#endif //BATTERY_MONITOR_CONFIG_H 
