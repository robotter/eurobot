#ifndef ADC_H
#define ADC_H

#include <avarix.h>
#include <avr/io.h>

uint16_t adc_GetValue(ADC_t *Adc, uint8_t channel);
uint16_t adc_Get3v3Value(ADC_t *Adc);

#endif //ADC_H
