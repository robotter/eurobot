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
#include "avoidance.h"
#include "cli.h"
#include "motor_encoders.h"

#include "settings.h"

#include "adxrs/adxrs.h"
#include "katioucha.h"


#include "telemetry.h"

volatile uint8_t init0, init1;

portpin_t leds[4];

// accessed through ROME handler
#include "pid.h"
extern struct pid_filter pid_x;
extern struct pid_filter pid_y;
extern struct pid_filter pid_angle;

#include "quadramp.h"
extern struct quadramp_filter qramp_angle;

// avoidance system
extern avoidance_t avoidance;

// match timer
bool match_timer_counting = false;
int32_t match_timer_ms = -1;

// ROME interfaces
rome_intf_t rome;
#if defined(BUILD_GALIPEUR)
rome_intf_t rome_r3d2;
#endif

// katioucha wrapper for ROME
static void _katioucha_fire(uint8_t n) {
  static uint8_t rounds_fired = 0;

  const uint8_t low = KATIOUCHA_LINE_LOW;
  const uint8_t high = KATIOUCHA_LINE_HIGH;
  const uint8_t tube1 = KATIOUCHA_FIRE_TUBE_1;
  const uint8_t tube2 = KATIOUCHA_FIRE_TUBE_2;
  const uint8_t tube3 = KATIOUCHA_FIRE_ALL;

  uint8_t i;
  for(i=rounds_fired; i<rounds_fired+n; i++) {
    switch(i) {
      case 0: katioucha_set_position(low,tube1); break;
      case 1: katioucha_set_position(high,tube1); break;
      case 2: katioucha_set_position(low,tube2); break;
      case 3: katioucha_set_position(high,tube2); break;
      case 4: katioucha_set_position(low,tube3); break;
      case 5: katioucha_set_position(high,tube3); break;
      default:
        // *click*, "ton flingue est vide john"
        break;
    }
  }
  rounds_fired += n;
  // downlink ammo qty
  ROME_SEND_KATIOUCHA_TM_ROUNDS_FIRED(&rome, rounds_fired);
}

// ROME messages handler
void rome_handler(rome_intf_t *intf, const rome_frame_t *frame) {
  switch((uint8_t)frame->mid) {

    case ROME_MID_ASSERV_ACTIVATE: {
      uint8_t fid = frame->asserv_autoset.fid;
      uint8_t b = frame->asserv_activate.activate;
      robot_cs_activate(&robot_cs, b);
      ROME_SEND_ACK(intf, fid);
      break;
    }
    case ROME_MID_ASSERV_CALIBRATE: { 
      uint8_t fid = frame->asserv_calibrate.fid;
      uint8_t b = frame->asserv_calibrate.b;
      adxrs_calibration_mode(b);
      ROME_SEND_ACK(intf, fid);
      break;
    }
    case ROME_MID_START_TIMER: {
      uint8_t fid = frame->start_timer.fid;
      // start match
      match_timer_counting = true;
      // XXX turn avoidance on at the same time XXX
      //avoidance_inhibit(&avoidance,false);
      ROME_SEND_ACK(intf,fid);
      break;
    }
    case ROME_MID_ASSERV_AUTOSET: {
      uint8_t fid = frame->asserv_autoset.fid;
      uint8_t side = frame->asserv_autoset.side;
      float x = frame->asserv_autoset.x;
      float y = frame->asserv_autoset.y;
      htrajectory_autoset(&trajectory, side, x, y);
      ROME_SEND_ACK(intf, fid);
      break;
    }
    case ROME_MID_ASSERV_GOTO_XY: {
      uint8_t fid = frame->asserv_goto_xy.fid;
      float x = frame->asserv_goto_xy.x;
      float y = frame->asserv_goto_xy.y;
      float a = (frame->asserv_goto_xy.a)/1000.0;
      htrajectory_gotoXY(&trajectory, x, y);
      htrajectory_gotoA(&trajectory, a);
      ROME_SEND_ACK(intf, fid);
      break;
    }
    case ROME_MID_ASSERV_GOTO_XY_REL: {
      uint8_t fid = frame->asserv_goto_xy_rel.fid;
      float x = frame->asserv_goto_xy_rel.x;
      float y = frame->asserv_goto_xy_rel.y;
      float a = (frame->asserv_goto_xy_rel.a)/1000.0;
      htrajectory_gotoXY_R(&trajectory, x, y);
      htrajectory_gotoA_R(&trajectory, a);
      ROME_SEND_ACK(intf, fid);
      break;
    }
    case ROME_MID_ASSERV_SET_XYA: {
      uint8_t fid = frame->asserv_set_xya.fid;
      int16_t x = frame->asserv_set_xya.x;
      int16_t y = frame->asserv_set_xya.y;
      int16_t a = (frame->asserv_set_xya.a)/1000.0;
      hposition_set(&position,x,y,a);
      ROME_SEND_ACK(intf, fid);
      break;
    }
    case ROME_MID_ASSERV_SET_XY: {
      uint8_t fid = frame->asserv_set_xy.fid;
      double a;
      hposition_get_a(&position, &a);
      int16_t x = frame->asserv_set_xya.x;
      int16_t y = frame->asserv_set_xya.y;
      hposition_set(&position,x,y,a);
      ROME_SEND_ACK(intf, fid);
      break;
    }
    case ROME_MID_ASSERV_SET_A: {
      uint8_t fid = frame->asserv_set_a.fid;
      vect_xy_t xy;
      hposition_get_xy(&position,&xy);
      int16_t a = (frame->asserv_set_xya.a)/1000.0;
      hposition_set(&position, xy.x, xy.y, a);
      ROME_SEND_ACK(intf, fid);
      break;
    }
    case ROME_MID_ASSERV_SET_X_PID: {
      uint8_t fid = frame->asserv_set_x_pid.fid;
      uint16_t p = frame->asserv_set_x_pid.p;
      uint16_t i = frame->asserv_set_x_pid.i;
      uint16_t d = frame->asserv_set_x_pid.d;
      int32_t max_in = frame->asserv_set_x_pid.max_in;
      int32_t max_I = frame->asserv_set_x_pid.max_I;
      int32_t max_out = frame->asserv_set_x_pid.max_out;
      pid_set_gains(&pid_x, p, i, d);
      pid_set_maximums(&pid_x, max_in, max_I, max_out);
      ROME_SEND_ACK(intf, fid);
      break;
    }
    case ROME_MID_ASSERV_SET_Y_PID: {
      uint8_t fid = frame->asserv_set_y_pid.fid;
      uint16_t p = frame->asserv_set_y_pid.p;
      uint16_t i = frame->asserv_set_y_pid.i;
      uint16_t d = frame->asserv_set_y_pid.d;
      int32_t max_in = frame->asserv_set_y_pid.max_in;
      int32_t max_I = frame->asserv_set_y_pid.max_I;
      int32_t max_out = frame->asserv_set_y_pid.max_out;
      pid_set_gains(&pid_y, p, i, d);
      pid_set_maximums(&pid_y, max_in, max_I, max_out);
      ROME_SEND_ACK(intf, fid);
      break;
    }
    case ROME_MID_ASSERV_SET_A_PID: {
      uint8_t fid = frame->asserv_set_a_pid.fid;
      uint16_t p = frame->asserv_set_a_pid.p;
      uint16_t i = frame->asserv_set_a_pid.i;
      uint16_t d = frame->asserv_set_a_pid.d;
      int32_t max_in = frame->asserv_set_a_pid.max_in;
      int32_t max_I = frame->asserv_set_a_pid.max_I;
      int32_t max_out = frame->asserv_set_a_pid.max_out;
      pid_set_gains(&pid_angle, p, i, d);
      pid_set_maximums(&pid_angle, max_in, max_I, max_out);
      ROME_SEND_ACK(intf, fid);
      break;
    }
    case ROME_MID_ASSERV_SET_A_QRAMP: {
      uint8_t fid = frame->asserv_set_a_qramp.fid;
      uint16_t dot = frame->asserv_set_a_qramp.dot;
      uint16_t dotdot = frame->asserv_set_a_qramp.dotdot;
      quadramp_set_1st_order_vars(&qramp_angle, dot, dot);
      quadramp_set_2nd_order_vars(&qramp_angle, dotdot, dotdot);
      ROME_SEND_ACK(intf, fid);
      break;
    }
    case ROME_MID_ASSERV_SET_HTRAJ_XY_CRUISE: {
      uint8_t fid = frame->asserv_set_htraj_xy_cruise.fid;
      float speed = frame->asserv_set_htraj_xy_cruise.speed;
      float acc   = frame->asserv_set_htraj_xy_cruise.acc;
      htrajectory_setXYCruiseSpeed(&trajectory, speed, acc);
      ROME_SEND_ACK(intf, fid);
      break;
    }
    case ROME_MID_ASSERV_SET_HTRAJ_XY_STEERING: {
      uint8_t fid = frame->asserv_set_htraj_xy_steering.fid;
      float speed = frame->asserv_set_htraj_xy_steering.speed;
      float acc   = frame->asserv_set_htraj_xy_steering.acc;
      htrajectory_setXYSteeringSpeed(&trajectory, speed, acc);
      ROME_SEND_ACK(intf, fid);
      break;
    }
    case ROME_MID_ASSERV_SET_HTRAJ_XY_STOP: {
      uint8_t fid = frame->asserv_set_htraj_xy_stop.fid;
      float speed = frame->asserv_set_htraj_xy_stop.speed;
      float acc   = frame->asserv_set_htraj_xy_stop.acc;
      htrajectory_setXYStopSpeed(&trajectory, speed, acc);
      ROME_SEND_ACK(intf, fid);
      break;
    }
    case ROME_MID_ASSERV_SET_HTRAJ_XYSTEERING_WINDOW: {
      uint8_t fid = frame->asserv_set_htraj_xysteering_window.fid;
      float r = frame->asserv_set_htraj_xysteering_window.r;
      htrajectory_setSteeringWindow(&trajectory, r);
      ROME_SEND_ACK(intf, fid);
      break;
    }
    case ROME_MID_ASSERV_SET_HTRAJ_STOP_WINDOWS: {
      uint8_t fid = frame->asserv_set_htraj_stop_windows.fid;
      float xy = frame->asserv_set_htraj_stop_windows.xy;
      float angle = frame->asserv_set_htraj_stop_windows.angle;
      htrajectory_setStopWindows(&trajectory, xy, angle);
      ROME_SEND_ACK(intf, fid);
      break;
    }

#if defined(BUILD_GALIPEUR)
    // forward orders to R3D2 board
    case ROME_MID_R3D2_CALIBRATE_ANGLE:
    case ROME_MID_R3D2_CALIBRATE_DIST:
    case ROME_MID_R3D2_CONF_LOAD:
    case ROME_MID_R3D2_CONF_SAVE:
    case ROME_MID_R3D2_SET_MOTOR_SPEED:
      rome_send(&rome_r3d2, frame);
      break;
#endif

#if defined(BUILD_GALIPETTE)
    // fire some katioucha tubes
    case ROME_MID_KATIOUCHA_FIRE: {
      uint8_t fid = frame->katioucha_fire.fid;
      uint8_t n = frame->katioucha_fire.n;
      _katioucha_fire(n);
      ROME_SEND_ACK(intf,fid);
      break;
    }
#endif

    default:
      break;
  }
}

// ROME message handler for R3D2
void rome_r3d2_handler(rome_intf_t *intf, const rome_frame_t *frame) {
  // forward to strat
  rome_send(&rome, frame);
}


// CSs cpu usage in percent (0-100)
extern uint8_t cs_cpuUsage;

int up_cnt = 0;
void vcs_update(void)
{
  up_cnt++;
  cs_update(NULL);

  // check match timer
  if(match_timer_ms > 1000*(int32_t)SETTING_MATCH_DURATION_SECS) {
    // out of time
    hrobot_break(1);
  }
  else {
    // update match timer
    if(match_timer_counting) {
      match_timer_ms += CONTROL_SYSTEM_PERIOD_US/1000;
    }
  }

  // downlink match timer telemetry
  TM_DL_MATCH_TIMER(match_timer_ms/1000);
}


#if defined(BUILD_GALIPEUR)
#define ZGYRO_SCALE 2*1.1214e-6
#elif defined(BUILD_GALIPETTE)
#define ZGYRO_SCALE -2.1964e-6//1.335e-6//1.0*BASE_ZGYRO_SCALE
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

#if defined(BUILD_GALIPEUR)
  rome_intf_init(&rome_r3d2);
  rome_r3d2.uart = uartE0;
  rome_r3d2.handler = rome_r3d2_handler;
#endif
  
  // initialize katioucha
  katioucha_init();

  //--------------------------------------------------------
  // CS
  //--------------------------------------------------------
  cs_initialize();

  // setup ADXRS update task
  timer_set_callback(timerE0, 'B', TIMER_US_TO_TICKS(E0, ADXRS_PERIOD_US), ADXRS_INTLVL, _adxrs_update);

  // remove break
  hrobot_break(0);

  adxrs_calibration_mode(true);
  _delay_ms(2000);
  adxrs_calibration_mode(false);

  printf("-- reboot --\n");
  //----------------------------------------------------------------------
  PORTQ.OUT = 0;
  for(;;) {
    PORTQ.OUT++;
    _delay_ms(10);
    rome_handle_input(&rome);
#if defined(BUILD_GALIPEUR)
    rome_handle_input(&rome_r3d2);
#endif
  }
}


