#ifndef ADC_H
#define ADC_H

#include <avarix.h>
#include <avr/io.h>

uint16_t adc_GetValue(ADC_t *Adc, uint8_t channel);

#endif //ADC_H
