#include <math.h>
#include <avarix/intlvl.h>
#include <avarix/portpin.h>
#include <clock/clock.h>
#include <uart/uart.h>
#include <timer/timer.h>
#include "led_config.h"
#include "com_config.h"
#include "rgb_led.h"



int main(void)
{
  clock_init();
  timer_init();
  uart_init();
  uart_fopen(UART_COM);
  CPU_SREG |= CPU_I_bm;
  INTLVL_ENABLE_ALL();

  // initialise leds and RGB led
  rgb_led_Init();

 // timer_set_callback(timerE0, 'A', TIMER_US_TO_TICKS(E0,20000), INTLVL_LO, update_data_cb);
 // timer_set_callback(timerE0, 'B', TIMER_US_TO_TICKS(E0,200000), INTLVL_LO, send_ppp_events_cb);

  // main loop
  for(;;) {
  }
}

