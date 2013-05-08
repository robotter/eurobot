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
#include <clock/clock.h>
#include <util/delay.h>

#include <stdio.h>
#include <stdlib.h>
#include <uart/uart.h>
#include <math.h>
#include <timer/timer.h>
#include <perlimpinpin/perlimpinpin.h>
#include <perlimpinpin/payload/system.h>
#include <perlimpinpin/payload/room.h>
#include <perlimpinpin/payload/log.h>

#include "hrobot_manager.h"
#include "cs.h"
#include "robot_cs.h"
#include "htrajectory.h"
#include "cli.h"
#include "motor_encoders.h"

#include "settings.h"

// error code
#define MAIN_ERROR 0x30



// CSs cpu usage in percent (0-100)
extern uint8_t cs_cpuUsage;

void vcs_update(void)
{
  cs_update(NULL);
}


ppp_intf_t pppintf;
ppp_payload_handler_t *ppp_filter(ppp_intf_t *intf);
void room_message_handler(ppp_intf_t *intf, room_payload_t *pl);


int main(void)
{
  // Booting

  // Initialize clocks
  clock_init();

  // Initialize UART
  uart_init();
  uart_fopen(CLI_USER_UART);

  // Initialize Timer
  timer_init();
  timer_set_callback(timerE0, 'A', TIMER_US_TO_TICKS(E0,CONTROL_SYSTEM_PERIOD_US), CONTROL_SYSTEM_INTLVL, vcs_update);

  INTLVL_ENABLE_ALL();
  __asm__("sei");

  // PPP init
  pppintf.filter = ppp_filter;
  pppintf.uart = uartE0;
  pppintf.addr = 0x11;
  ppp_intf_init(&pppintf);
  room_set_message_handler(room_message_handler);
  // send a system RESET to signal that we have booted
  ppp_send_system_reset(&pppintf);

  // Some advertisment :p
  PPP_LOG(&pppintf, NOTICE, "Robotter 2013 - Galipeur - SUPER-UNIOC-NG PROPULSION");
  PPP_LOG(&pppintf, NOTICE, "Compiled "__DATE__" at "__TIME__".");

  //--------------------------------------------------------
  // CS
  //--------------------------------------------------------

  PPP_LOG(&pppintf, NOTICE, "Initializing CS");
  cs_initialize();

  // remove break
  hrobot_break(0);

  //----------------------------------------------------------------------

  for(;;) {
    ppp_intf_update(&pppintf);
  }
}

