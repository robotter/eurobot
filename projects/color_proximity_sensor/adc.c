#include "adc.h"
#include <avarix.h>

uint16_t adc_GetValue(ADC_t *Adc, uint8_t channel)
{
  Adc-> CTRLA = ADC_ENABLE_bm; 
  Adc->REFCTRL = ADC_REFSEL_INT1V_gc | ADC_BANDGAP_bm;
  #ifdef ADC_REFSEL_VCC_gc
  Adc->REFCTRL = ADC_REFSEL_VCC_gc;
  #elif defined ADC_REFSEL_INTVCC_gc
  Adc->REFCTRL = ADC_REFSEL_INTVCC_gc;
  #endif

  Adc->CH0.MUXCTRL = (channel << ADC_CH_MUXPOS0_bp) & ADC_CH_MUXPOS_gm;
  /* start conversion in single ended */
  Adc->CH0.CTRL = ADC_CH_START_bm | ADC_CH_INPUTMODE_SINGLEENDED_gc;
  
  /* wait for end of conversion */
  while((Adc->INTFLAGS & ADC_CH0IF_bm) == 0x00u); 

  // adc incorporates an offset of 205 to ensure proper 0V measure)
  return (Adc->CH0RES - 515u);
}

uint16_t adc_Get3v3Value(ADC_t *Adc)
{
  Adc-> CTRLA = ADC_ENABLE_bm; 
  Adc->REFCTRL = ADC_REFSEL_INT1V_gc | ADC_BANDGAP_bm;
  #ifdef ADC_REFSEL_VCC_gc
  Adc->REFCTRL = ADC_REFSEL_VCC_gc;
  #elif defined ADC_REFSEL_INTVCC_gc
  Adc->REFCTRL = ADC_REFSEL_INTVCC_gc;
  #endif
  Adc->CH0.MUXCTRL = ADC_CH_MUXINT_SCALEDVCC_gc; 
  /* start conversion in single ended */
  Adc->CH0.CTRL = ADC_CH_START_bm | ADC_CH_INPUTMODE_INTERNAL_gc; 
  
  /* wait for end of conversion */
  while((Adc->INTFLAGS & ADC_CH0IF_bm) == 0x00u); 

  // adc incorporates an offset of 205 to ensure proper 0V measure)
  return (Adc->CH0RES - 225u);
}
