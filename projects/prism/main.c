#include <avr/io.h>
#include <avr/interrupt.h>
#include <avarix/intlvl.h>
#include <clock/clock.h>
#include <timer/timer.h>
#include <uart/uart.h>
#include <util/delay.h>
#include "beacom.h"
#include "sensor.h"
#include "battery_monitor.h"
#include "battery_monitor_config.h"

//TODO: Cette variable devrait etre statique au module battery_monitor
BATTMON_t bat;

void battmon_update(void)  {
  PORTH.OUTTGL = 0xE0;
  BATTMON_monitor(&bat);
}

int main(void) {

  clock_init();
  uart_init();
  uart_fopen(uartC1);
  CPU_SREG |= CPU_I_bm;
  INTLVL_ENABLE_ALL();


  beacom_init();
  BATTMON_Init(&bat);
  sensor_init();

  //led init
  PORTH.DIRSET = 0xE0;
  PORTH.OUTCLR = 0xE0;

  timer_init();
  timer_set_callback(timerE0, 'A', TIMER_US_TO_TICKS(E0,BATTMON_UPDATE_PERIOD_US), BATTMON_UPDATE_INTLVL, battmon_update);

  beacom_run();

  //never reached
  while(1);
}
