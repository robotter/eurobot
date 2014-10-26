#ifndef BAROMETER_H
#define BAROMETER_H

#include <avr/io.h>

typedef struct {

  ADC_t *adc;
  ADC_CH_MUXPOS_t mux;

} barometer_t;

/** @brief Initialize barometer */
void barometer_init(barometer_t *baro, ADC_t* adc, ADC_CH_MUXPOS_t mux);

/** @brief Return pressure in some unit */
uint16_t barometer_get_pressure(barometer_t *baro);

#endif//BAROMETER_H
