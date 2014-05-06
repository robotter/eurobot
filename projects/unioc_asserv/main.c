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

#include "adxrs/adxrs.h"

// XXX
extern int _debug,_debug2,_debug3;
extern motor_encoders_t encoders;
extern hrobot_system_t system;
int dummy;

portpin_t leds[4];


// ROME interface
rome_intf_t rome;
// ROME messages handler
void rome_handler(rome_intf_t *intf, const rome_frame_t *frame) {

}

// CSs cpu usage in percent (0-100)
extern uint8_t cs_cpuUsage;

int up_cnt = 0;
void vcs_update(void)
{
  up_cnt++;
  cs_update(NULL);
}


#define BASE_ZGYRO_SCALE 1.1214e-6
#if defined(BUILD_GALIPEUR)
#define ZGYRO_SCALE -1.0*BASE_ZGYRO_SCALE
#elif defined(BUILD_GALIPETTE)
#define ZGYRO_SCALE 1.0*BASE_ZGYRO_SCALE
#else
# error "Please define either BUILD_GALIPEUR or BUILD_GALIPETTE"
#endif

void _adxrs_update(void) {
  adxrs_capture_manual(ZGYRO_SCALE);
}

int main(void)
{
  // Booting
  for(int k=0;k<4;k++) {
    leds[k] = PORTPIN(Q,k);
    portpin_dirset(leds+k);
    portpin_outclr(leds+k);
  }
  // Initialize clocks
  clock_init();

  // Initialize UART
  uart_init();
  uart_fopen(CLI_USER_UART);

  // Initialize Timer
  timer_init();
  timer_set_callback(timerE0, 'A', TIMER_US_TO_TICKS(E0, CONTROL_SYSTEM_PERIOD_US), CONTROL_SYSTEM_INTLVL, vcs_update);

  INTLVL_ENABLE_ALL();
  __asm__("sei");

  // Initialize ROME
  rome_intf_init(&rome);
  rome.uart = uartD0;
  rome.handler = rome_handler;
  
  // XXX PPP_LOG(&pppintf, NOTICE, "Robotter 2013 - Galipeur - SUPER-UNIOC-NG PROPULSION");
  // XXX PPP_LOG(&pppintf, NOTICE, "Compiled "__DATE__" at "__TIME__".");

  //--------------------------------------------------------
  // CS
  //--------------------------------------------------------
  cs_initialize();

  // setup ADXRS update task
  timer_set_callback(timerE0, 'B', TIMER_US_TO_TICKS(E0, ADXRS_PERIOD_US), ADXRS_INTLVL, _adxrs_update);

  // remove break
  hrobot_break(0);

  _delay_ms(500);
  printf("-- reboot --\n");
  //----------------------------------------------------------------------
  int32_t i = 0;
  for(;;) {
    vect_xy_t p;
    hposition_get_xy(&position, &p);
    double alpha;
    hposition_get_a(&position, &alpha);
    //ROME_SEND_ASSERV_TM_XYA(&rome, p.x, p.y, alpha);
    
    printf("%f %f %f\n", p.x, p.y, alpha);
    _delay_ms(100);
  }

  for(;;) {
    (void)i;
    htrajectory_gotoXY_R(&trajectory,500,0);
    htrajectory_gotoA(&trajectory,0);
    while(!htrajectory_doneXY(&trajectory));

    htrajectory_gotoXY_R(&trajectory,0,500);
    htrajectory_gotoA(&trajectory,0.5*M_PI);
    while(!htrajectory_doneXY(&trajectory));

    htrajectory_gotoXY_R(&trajectory,-500,0);
    htrajectory_gotoA(&trajectory,0);
    while(!htrajectory_doneXY(&trajectory));

    htrajectory_gotoXY_R(&trajectory,0,-500);
    htrajectory_gotoA(&trajectory,0.5*M_PI);
    while(!htrajectory_doneXY(&trajectory));
  }
}


