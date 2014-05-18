/*  
 *  Copyright RobOtter (2013)
 * 
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <avarix.h>
#include <avarix/portpin.h>
#include <clock/clock.h>
#include <util/delay.h>

#include <stdio.h>
#include <stdlib.h>
#include <uart/uart.h>
#include <timer/timer.h>
#include <rome/rome.h>
#include "rome_acks.h"

#define ROME_UPDATE_FREQUENCY_HZ 10

#define ROME_ASSERV_UART  uartC0
#define ROME_MECA_UART    uartD0
#define ROME_PADDOCK_UART uartE1

// declare ROME interfaces
rome_intf_t rome_asserv;
rome_intf_t rome_meca;
rome_intf_t rome_paddock;

// message handlers
void rome_asserv_handler(rome_intf_t *intf, const rome_frame_t *frame) {

  if(rome_acks_handle(intf, frame))
    return;

  switch(frame->mid) {
    default:
      break;
  }
}

void rome_meca_handler(rome_intf_t *intf, const rome_frame_t *frame) {

  if(rome_acks_handle(intf, frame))
    return;

  switch(frame->mid) {
    default:
      break;
  }
}

void rome_paddock_handler(rome_intf_t *intf, const rome_frame_t *frame) {

  if(rome_acks_handle(intf, frame))
    return;

  switch(frame->mid) {
    default:
      break;
  }
}

// update rome handlers
static void _update_rome_handlers(void) {
  rome_handle_input(&rome_asserv);
  rome_handle_input(&rome_meca);
  rome_handle_input(&rome_paddock);
}

int main(void)
{
  // Booting

  // Initialize clocks
  clock_init();

  // Initialize UART
  uart_init();
  uart_fopen(uartC0);

  INTLVL_ENABLE_ALL();
  __asm__("sei");

  //----------------------------------------------------------------------
  // Initialize ROME
  rome_intf_init(&rome_asserv);
  rome_asserv.uart = ROME_ASSERV_UART;
  rome_asserv.handler = rome_asserv_handler;

  rome_intf_init(&rome_meca);
  rome_meca.uart = ROME_MECA_UART;
  rome_meca.handler = rome_meca_handler;

  rome_intf_init(&rome_paddock);
  rome_paddock.uart = ROME_PADDOCK_UART;
  rome_paddock.handler = rome_paddock_handler;

  // leds
  portpin_t *led_r = &PORTPIN(F,0);
  portpin_t *led_g = &PORTPIN(F,7);
  portpin_t *led_b = &PORTPIN(A,7);

  portpin_t *led0 = &PORTPIN(A,1);
  portpin_t *led1 = &PORTPIN(A,2);
  portpin_t *led2 = &PORTPIN(A,3);
  portpin_t *led3 = &PORTPIN(A,4);

  portpin_dirset(led_r);
  portpin_dirset(led_g);
  portpin_dirset(led_b);

  portpin_dirset(led0);
  portpin_dirset(led1);
  portpin_dirset(led2);
  portpin_dirset(led3);

  // Initialize timers
  timer_init();
  timer_set_callback(timerE0, 'A', 
    TIMER_US_TO_TICKS(E0, 1e6/ROME_UPDATE_FREQUENCY_HZ),
    INTLVL_LO, _update_rome_handlers);

  for(;;) {

    ROME_SEND_AND_WAIT(ASSERV_GOTO_XY, &rome_asserv, 0, 0, 0);

  }
}


