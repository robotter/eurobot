#include <avr/io.h>
#include <avarix.h>
#include <avarix/portpin.h>
#include "battery_config.h"


void battery_init(void)
{
  // initialize ADC
  BATTERY_ADC.CH0.CTRL = ADC_CH_INPUTMODE_SINGLEENDED_gc;
  BATTERY_ADC.REFCTRL = ADC_REFSEL_INTVCC_gc;
  BATTERY_ADC.CTRLA = ADC_ENABLE_bm;
  portpin_dirclr(&BATTERY_ADC_PORTPIN);
}

uint16_t battery_get_value(void)
{
  // get ADC value
  BATTERY_ADC.CH0.MUXCTRL = BATTERY_ADC_MUX;
  BATTERY_ADC.CH0.CTRL |= ADC_CH_START_bm;
  while(!(BATTERY_ADC.CH0.INTFLAGS & ADC_CH_CHIF_bm));
  // adc incorporates an offset of 205 to ensure proper 0V measure)
  uint16_t adc_value = BATTERY_ADC.CH0.RES - 205;

  // convert to battery voltage
  return (float)adc_value * BATTERY_VOLTAGE_COEF + BATTERY_VOLTAGE_OFFSET;
}

