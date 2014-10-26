#include "barometer.h"

void barometer_init(barometer_t *baro, ADC_t* adc, ADC_CH_MUXPOS_t mux) {
  baro->adc = adc;
  baro->mux = mux;

  adc->CH0.CTRL = ADC_CH_INPUTMODE_SINGLEENDED_gc;
  // set ADC reference to 1V and enable ADCA
  adc->REFCTRL = ADC_REFSEL_INT1V_gc;
  adc->CTRLA = ADC_ENABLE_bm;
}

uint16_t barometer_get_pressure(barometer_t *baro) {
  // select input
  baro->adc->CH0.MUXCTRL = baro->mux;

  // start conversion
  baro->adc->CH0.CTRL |= ADC_CH_START_bm;
  // wait for measurement
  while(!(baro->adc->CH0.INTFLAGS & ADC_CH_CHIF_bm));
  return baro->adc->CH0.RES;
}
