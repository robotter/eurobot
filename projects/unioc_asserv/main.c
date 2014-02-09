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
#include <rome/rome.h>
#include <math.h>
#include <timer/timer.h>

#include "hrobot_manager.h"
#include "cs.h"
#include "robot_cs.h"
#include "htrajectory.h"
#include "cli.h"
#include "motor_encoders.h"

#include "settings.h"

// ROME interface
rome_intf_t rome;
// ROME messages handler
void rome_handler(rome_intf_t *intf, const rome_frame_t *frame) {

}

// CSs cpu usage in percent (0-100)
extern uint8_t cs_cpuUsage;

void vcs_update(void)
{
  cs_update(NULL);
}


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

  // Initialize ROME
  rome_intf_init(&rome);
  rome.uart = uartE0;
  rome.handler = rome_handler;

  // XXX PPP_LOG(&pppintf, NOTICE, "Robotter 2013 - Galipeur - SUPER-UNIOC-NG PROPULSION");
  // XXX PPP_LOG(&pppintf, NOTICE, "Compiled "__DATE__" at "__TIME__".");

  //--------------------------------------------------------
  // CS
  //--------------------------------------------------------

  cs_initialize();

  // remove break
  hrobot_break(0);

  //----------------------------------------------------------------------

  for(;;) {
    // XXX ppp_intf_update(&pppintf);
  }
}


