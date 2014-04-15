#include "adc.h"
#include <avarix.h>

uint16_t adc_GetValue(ADC_t *Adc, uint8_t channel)
{
  Adc->REFCTRL = ADC_REFSEL_INT1V_gc | ADC_BANDGAP_bm;
  Adc->CH0.MUXCTRL = (channel << ADC_CH_MUXPOS0_bp) & ADC_CH_MUXPOS_gm;

  /* ctart conversion in single ended */
  Adc->CH0.CTRL = ADC_CH_START_bm | ADC_CH_INPUTMODE0_bm;
  /* wait for end of conversion */
  while((Adc->INTFLAGS & ADC_CH0IF_bm) == 0x00u); 

  return Adc->CH0RES;
}
