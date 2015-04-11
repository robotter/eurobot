#include <math.h>
#include <avarix/intlvl.h>
#include <avarix/portpin.h>
#include <clock/clock.h>
#include <uart/uart.h>
#include <timer/timer.h>
#include "led_config.h"
#include "com_config.h"
#include "rgb_led.h"
#include <stdio.h>

#include "proximity_color_sensor_fsm.h"


int main(void)
{
  clock_init();
  timer_init();
  uart_init();
  uart_fopen(UART_COM);
  CPU_SREG |= CPU_I_bm;
  INTLVL_ENABLE_ALL();

  // leds port init
  PORTC.DIRSET = _BV(6)| _BV(7);

  
  printf("INIT TCS37725\r\n");
  printf("PORT %x %x\r\n", PORTE.IN&_BV(0), PORTE.IN&_BV(1));
  PCSFSM_Init();


 // timer_set_callback(timerE2, 'A', TIMER_US_TO_TICKS(E0,20000), INTLVL_LO, update_data_cb);
 // timer_set_callback(timerE2, 'B', TIMER_US_TO_TICKS(E0,200000), INTLVL_LO, send_ppp_events_cb);

  printf("INIT DONE !!\r\n");
  // main loop
  for(;;) {
    PCSFSM_Update();
  }
}

