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

  portpin_outset(led_r);
  _delay_us(10000);
  portpin_outclr(led_r);
  _delay_us(90000);
  portpin_outset(led_g);
  _delay_us(10000);
  portpin_outclr(led_g);
  _delay_us(90000);
  portpin_outset(led_b);
  _delay_us(10000);
  portpin_outclr(led_b);
  _delay_us(90000);
  
  int c;
  for(;;) {
    if((c = uart_recv_nowait(uartC0)) >= 0) {
      uart_send(uartE1,c);
    }
    if( (c = uart_recv_nowait(uartE1)) >= 0 ) {
      uart_send(uartC0,c);
    }
    if( (c = uart_recv_nowait(uartD0)) >= 0 ) {
      uart_send(uartF0,c);
    }
    if( (c = uart_recv_nowait(uartF0)) >= 0 ) {
      uart_send(uartD0,c);
    }
  }
}


