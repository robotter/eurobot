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

volatile uint8_t init0, init1;

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
  switch((uint8_t)frame->mid) {

    case ROME_MID_ASSERV_ACTIVATE: {
      uint8_t b = frame->asserv_activate.activate;
      // TODO
      break;
    }
    case ROME_MID_ASSERV_AUTOSET: {
      uint8_t side = frame->asserv_autoset.side;
      float x = frame->asserv_autoset.x;
      float y = frame->asserv_autoset.y;
      htrajectory_autoset(&trajectory, side, x, y);
      break;
    }
    case ROME_MID_ASSERV_GOTO_XY: {
      float x = frame->asserv_goto_xy.x;
      float y = frame->asserv_goto_xy.y;
      float a = (frame->asserv_goto_xy.a)/1000.0;
      htrajectory_gotoXY(&trajectory, x, y);
      htrajectory_gotoA(&trajectory, a);
      break;
    }
    case ROME_MID_ASSERV_GOTO_XY_REL: {
      float x = frame->asserv_goto_xy_rel.x;
      float y = frame->asserv_goto_xy_rel.y;
      float a = (frame->asserv_goto_xy_rel.a)/1000.0;
      htrajectory_gotoXY_R(&trajectory, x, y);
      htrajectory_gotoA_R(&trajectory, a);
      break;
    }
    case ROME_MID_ASSERV_ADVANCE: {
      int16_t d = frame->asserv_advance.d;
      break;
    }
    case ROME_MID_ASSERV_TURN: {
      int16_t a = frame->asserv_turn.a;
      break;
    }
    case ROME_MID_ASSERV_TURN_REL: {
      int16_t a = frame->asserv_turn_rel.a;
      break;
    }
    case ROME_MID_ASSERV_SET_XYA: {
      int16_t x = frame->asserv_set_xya.x;
      int16_t y = frame->asserv_set_xya.y;
      int16_t a = (frame->asserv_set_xya.a)/1000.0;
      hposition_set(&position,x,y,a);
      break;
    }
    case ROME_MID_ASSERV_SET_XY: {
      double a;
      hposition_get_a(&position, &a);
      int16_t x = frame->asserv_set_xya.x;
      int16_t y = frame->asserv_set_xya.y;
      hposition_set(&position,x,y,a);
      break;
    }
    case ROME_MID_ASSERV_SET_A: {
      vect_xy_t xy;
      hposition_get_xy(&position,&xy);
      int16_t a = (frame->asserv_set_xya.a)/1000.0;
      hposition_set(&position, xy.x, xy.y, a);
      break;
    }
    case ROME_MID_ASSERV_SET_X_PID: {
      break;
    }
    case ROME_MID_ASSERV_SET_Y_PID: {
      break;
    }
    case ROME_MID_ASSERV_SET_A_PID: {
      break;
    }
    case ROME_MID_ASSERV_SET_A_QRAMP: {
      break;
    }
    case ROME_MID_ASSERV_SET_HTRAJ_XY_CRUISE: {
      float speed = frame->asserv_set_htraj_xy_cruise.speed;
      float acc   = frame->asserv_set_htraj_xy_cruise.acc;
      htrajectory_setXYCruiseSpeed(&trajectory, speed, acc);
      ROME_SEND_ASSERV_SET_HTRAJ_XY_CRUISE(intf, speed, acc);
      break;
    }
    case ROME_MID_ASSERV_SET_HTRAJ_XY_STEERING: {
      float speed = frame->asserv_set_htraj_xy_steering.speed;
      float acc   = frame->asserv_set_htraj_xy_steering.acc;
      htrajectory_setXYSteeringSpeed(&trajectory, speed, acc);
      ROME_SEND_ASSERV_SET_HTRAJ_XY_STEERING(intf, speed, acc);
      break;
    }
    case ROME_MID_ASSERV_SET_HTRAJ_XY_STOP: {
      float speed = frame->asserv_set_htraj_xy_stop.speed;
      float acc   = frame->asserv_set_htraj_xy_stop.acc;
      htrajectory_setXYStopSpeed(&trajectory, speed, acc);
      ROME_SEND_ASSERV_SET_HTRAJ_XY_STOP(intf, speed, acc);
      break;
    }
    case ROME_MID_ASSERV_SET_HTRAJ_XYSTEERING_WINDOW: {
      break;
    }
    case ROME_MID_ASSERV_SET_HTRAJ_XYSTOP_WINDOW: {
      break;
    }
    case ROME_MID_ASSERV_SET_HTRAJ_ASTOP_WINDOW: {
      break;
    }
    default:
      break;
  }
}

// CSs cpu usage in percent (0-100)
extern uint8_t cs_cpuUsage;

int up_cnt = 0;
void vcs_update(void)
{
  up_cnt++;
  cs_update(NULL);
}


#if defined(BUILD_GALIPEUR)
#define ZGYRO_SCALE 2*1.1214e-6
#elif defined(BUILD_GALIPETTE)
#define ZGYRO_SCALE 2.1964e-6//1.335e-6//1.0*BASE_ZGYRO_SCALE
#else
# error "Please define either BUILD_GALIPEUR or BUILD_GALIPETTE"
#endif

void _adxrs_update(void) {
  adxrs_capture_manual(ZGYRO_SCALE);
}

int main(void)
{
  init0 = init1;
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
  
  //--------------------------------------------------------
  // CS
  //--------------------------------------------------------
  cs_initialize();

  // setup ADXRS update task
  timer_set_callback(timerE0, 'B', TIMER_US_TO_TICKS(E0, ADXRS_PERIOD_US), ADXRS_INTLVL, _adxrs_update);

  // remove break
  hrobot_break(0);

  //adxrs_calibration_mode(true);
  //_delay_ms(2000);
  //adxrs_calibration_mode(false);
  printf("-- reboot --\n");
  //----------------------------------------------------------------------
  for(;;) {
    _delay_ms(10);
    rome_handle_input(&rome);
  }
}


