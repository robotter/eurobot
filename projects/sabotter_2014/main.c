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
#include "strat.h"
#include "config.h"

// ROME interfaces
rome_intf_t rome_asserv;
rome_intf_t rome_meca;
rome_intf_t rome_paddock;


// message handlers

static void rome_asserv_handler(rome_intf_t *intf, const rome_frame_t *frame)
{
  switch(frame->mid) {
    case ROME_MID_ACK:
      if(frame->ack.fid <= ROME_FRAME_ID_MAX) {
        rome_acks_free_frame_id(frame->ack.fid);
        return; // don't forward
      }
      break;
    default:
      break;
  }

  // forward
  rome_send(&rome_paddock, frame);
}

static void rome_meca_handler(rome_intf_t *intf, const rome_frame_t *frame)
{
  switch(frame->mid) {
    case ROME_MID_ACK:
      if(frame->ack.fid <= ROME_FRAME_ID_MAX) {
        rome_acks_free_frame_id(frame->ack.fid);
        return; // don't forward
      }
      break;
    default:
      break;
  }

  // forward
  rome_send(&rome_paddock, frame);
}

static void rome_paddock_handler(rome_intf_t *intf, const rome_frame_t *frame)
{
  switch(frame->mid) {
    case ROME_MID_ACK:
      // should not happen
      rome_acks_free_frame_id(frame->ack.fid);
      return;
    default:
      break;
  }

  // forward to other interfaces
  rome_send(&rome_asserv, frame);
  rome_send(&rome_meca, frame);
}

// Handle input from all ROME interfaces
void update_rome_interfaces(void)
{
  rome_handle_input(&rome_asserv);
  rome_handle_input(&rome_meca);
  rome_handle_input(&rome_paddock);
}


/// current time in microseconds
static volatile uint32_t uptime;

/// Get uptime value
uint32_t get_uptime_us(void)
{
  uint32_t tmp;
  INTLVL_DISABLE_ALL_BLOCK() {
    tmp = uptime;
  }
  return tmp;
}

/// Called on uptime timer tick
static void update_uptime(void)
{
  uptime += UPDATE_TICK_US;
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
  timer_set_callback(timerE0, 'A', TIMER_US_TO_TICKS(E0,UPDATE_TICK_US),
                     UPTIME_INTLVL, update_uptime);

  //TODO move arm to init position
  //TODO select color
  //TODO wait for starting cord
  strat_run(TEAM_NONE);
}


