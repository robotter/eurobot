#include "adc.h"
#include <avarix.h>
#include <avarix/portpin.h>

void adc_init(ADC_t *adc)
{
  adc->CH0.CTRL = ADC_CH_INPUTMODE_SINGLEENDED_gc;
  adc->REFCTRL = ADC_REFSEL_VCC_gc;
  adc->CTRLA = ADC_ENABLE_bm;
}

uint16_t adc_get_value(ADC_t *adc, ADC_CH_MUXPOS_t mux)
{
  // select ADC input
  adc->CH0.MUXCTRL = mux;
  adc->CH0.CTRL |= ADC_CH_START_bm;

  // wait for end of conversion
  while(!(adc->CH0.INTFLAGS & ADC_CH_CHIF_bm));
  // adc incorporates an offset of 205 to ensure proper 0V measure)
  return (adc->CH0.RES - 205);
}
