#ifndef _ADC_CONFIG_H_
#define _ADC_CONFIG_H_

/* Uncomment the ADC inputs you want to use */
/* #define ADC0_USE */
/* #define ADC1_USE */
/* #define ADC2_USE */
/* #define ADC3_USE */
/* #define ADC4_USE */
/* #define ADC5_USE */
/* #define ADC6_USE */
/* #define ADC7_USE */

/* Uncomment the ADC reference you want to use */
#define ADC_REF_EXT
/* #define ADC_REF_AVCC */
/* #define ADC_REF_INTERNAL */

/* This feature is only available for ATmega48/88/168 */
#define ADC_POWER_REDUCTION

/* Frequency (kHz) of the converter - Has to be between 50 and 200 */
/* -- Not used for now -- */
#define ADC_FREQUENCY 50

/* Filter coefficient: number of loops before a constant converted value */
/* is almost equal to the result given by adc_get_result(). */
/* -- Not used for now -- */
#define ADC_FILTER_COEFF 10

/* Enable power consumption reduction */
#define USE_ADC_SHUTDOWN

#endif // _ADC_CONFIG_H_
