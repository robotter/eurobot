#ifndef ADC_H
#define ADC_H

#include <avarix.h>
#include <avr/io.h>

void adc_init(ADC_t *Adc);
uint16_t adc_get_value(ADC_t *Adc, ADC_CH_MUXPOS_t mux);

#endif //ADC_H
