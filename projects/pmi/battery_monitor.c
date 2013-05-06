
#include "battery_monitor.h"

#include <avarix.h>

#include <perlimpinpin/perlimpinpin.h>
#include <perlimpinpin/payload/room.h>
#include <perlimpinpin/payload/log.h>
extern ppp_intf_t pppintf;

void battery_monitor_init() {

  ADCA.CH0.CTRL = ADC_CH_INPUTMODE_SINGLEENDED_gc;
  ADCA.CH0.MUXCTRL = ADC_CH_MUXPOS_PIN3_gc;
  // set ADC reference to 1V and enable ADCA
  ADCA.REFCTRL = ADC_REFSEL_INT1V_gc;
  ADCA.CTRLA = ADC_ENABLE_bm;

}

uint8_t battery_monitor_measure(void) {

  // start conversion
  ADCA.CH0.CTRL |= ADC_CH_START_bm;
  // wait for measurement
  while(!(ADCA.CH0.INTFLAGS & ADC_CH_CHIF_bm));
  uint32_t voltage_deci_volts = (160*(uint32_t)ADCA.CH0.RES)/4096;

  static uint8_t it = 0; it++;
  if(it >= 10) {
    it = 0;
    ROOM_SEND_PMI_BATTERY_VOLTAGE(&pppintf, 0xFF, voltage_deci_volts);
  }

  return (uint8_t)voltage_deci_volts;
}
