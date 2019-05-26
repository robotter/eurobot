#ifndef ADC_H
#define ADC_H

#include <avr/io.h>

void adc_init(ADC_t *adc);
uint16_t adc_get_value(ADC_t *adc, ADC_CH_MUXPOS_t mux);

#endif //ADC_H
