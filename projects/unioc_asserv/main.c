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
#include <avarix/intlvl.h>
#include <clock/clock.h>
#include <util/delay.h>

#include <stdio.h>
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
#include "servos.h"

#include "settings.h"

#include "adxrs/adxrs.h"


#include "telemetry.h"

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

#if defined(GALIPEUR)
#define ZGYRO_SCALE 2.185e-6
#elif defined(GALIPETTE)
#define ZGYRO_SCALE -2.1964e-6//1.335e-6//1.0*BASE_ZGYRO_SCALE
#else
# error "Please define either GALIPEUR or GALIPETTE"
#endif

// zgyro scale
float zgyro_scale = ZGYRO_SCALE;

// ROME interfaces
rome_intf_t rome;
rome_intf_t rome_r3d2;

// HACK HACK HACK
extern double hrobot_motors_invmatrix_correct[9];

// ROME messages handler
void rome_handler(rome_intf_t *intf, const rome_frame_t *frame)
{
  switch(frame->mid) {

    case ROME_MID_ASSERV_ACTIVATE: {
      if(adxrs_get_calibration_mode()) {
        ROME_LOG(&rome, DEBUG, "ASSERV_ACTIVATE refuse, gyro is in calibration");
      }
      else {
        robot_cs_activate(&robot_cs, frame->asserv_activate.activate);
      }
      rome_reply_ack(intf, frame);
    } break;
    case ROME_MID_ASSERV_CALIBRATE: { 
      adxrs_calibration_mode(frame->asserv_calibrate.b);
      rome_reply_ack(intf, frame);
    } break;
    case ROME_MID_ASSERV_AVOIDANCE: {
      avoidance_inhibit(&avoidance, !frame->asserv_avoidance.b);
      rome_reply_ack(intf, frame);
    } break;
    case ROME_MID_START_TIMER: {
      // start match
      match_timer_counting = true;
#if defined(GALIPEUR)
      // XXX turn avoidance on at the same time XXX
      //avoidance_inhibit(&avoidance,false);
#elif defined(GALIPETTE)
      //avoidance_inhibit(&avoidance,false);
#endif
      rome_reply_ack(intf, frame);
    } break;
    case ROME_MID_ASSERV_AUTOSET: {
      uint8_t table_side = frame->asserv_autoset.table_side;
      uint8_t robot_side = frame->asserv_autoset.robot_side;
      double x = frame->asserv_autoset.x;
      double y = frame->asserv_autoset.y;
      ROME_LOGF(&rome, DEBUG, "ASSERV AUTOSET %1.1f %1.1f %d %d", x,y,table_side,robot_side);
      htrajectory_autoset(&trajectory, robot_side, table_side, x, y);
      rome_reply_ack(intf, frame);
    } break;
    case ROME_MID_ASSERV_GOTO_XY: {
      float x = frame->asserv_goto_xy.x;
      float y = frame->asserv_goto_xy.y;
      float a = (frame->asserv_goto_xy.a)/1000.0;
      htrajectory_gotoXY(&trajectory, x, y);
      htrajectory_gotoA(&trajectory, a);
      rome_reply_ack(intf, frame);
    } break;
    case ROME_MID_ASSERV_GOTO_XY_REL: {
      float x = frame->asserv_goto_xy_rel.x;
      float y = frame->asserv_goto_xy_rel.y;
      float a = (frame->asserv_goto_xy_rel.a)/1000.0;
      htrajectory_gotoXY_R(&trajectory, x, y);
      htrajectory_gotoA_R(&trajectory, a);
      rome_reply_ack(intf, frame);
    } break;
    case ROME_MID_ASSERV_GOTO_XYA_SYNCED: {
      float x = frame->asserv_goto_xya_synced.x;
      float y = frame->asserv_goto_xya_synced.y;
      float a = (frame->asserv_goto_xya_synced.a)/1000.0;
      htrajectory_gotoXY(&trajectory, x, y);
      htrajectory_gotoXY_synced(&trajectory, x, y, a);
      rome_reply_ack(intf, frame);
    } break;
    case ROME_MID_ASSERV_GOTO_XYA_PANNING: {
      float x = frame->asserv_goto_xya_panning.x;
      float y = frame->asserv_goto_xya_panning.y;
      float pan_angle = (frame->asserv_goto_xya_panning.pan_angle)/1000.0;
      htrajectory_gotoXY_panning( &trajectory, x,y,pan_angle, NULL);
      rome_reply_ack(intf, frame);
    } break;
    case ROME_MID_ASSERV_RUN_TRAJ: {
      uint8_t nxy = ROME_FRAME_VARARRAY_SIZE(frame, asserv_run_traj, xy);
      if(nxy % 2 != 0) {
        ROME_LOGF(&rome, ERROR, "asserv_run_traj order ignored: odd number of coordinates (%u)", nxy);
      } else {
        vect_xy_t xy[nxy/2];
        for(uint8_t i=0; i<nxy; i+=2) {
          xy[i/2].x = frame->asserv_run_traj.xy[i];
          xy[i/2].y = frame->asserv_run_traj.xy[i+1];
          ROME_LOGF(&rome, DEBUG, "run_traj[%i] %f %f", i, xy[i/2].x, xy[i/2].y);
        }
        htrajectory_run(&trajectory, xy, sizeof(xy)/sizeof(*xy));
        float a = frame->asserv_run_traj.a/1000.0;
        htrajectory_gotoA(&trajectory, a);
      }
      rome_reply_ack(intf, frame);
    } break;
    case ROME_MID_ASSERV_SET_XYA: {
      int16_t x = frame->asserv_set_xya.x;
      int16_t y = frame->asserv_set_xya.y;
      int16_t a = (frame->asserv_set_xya.a)/1000.0;
      hposition_set(&position,x,y,a);
      htrajectory_reset_carrot(&trajectory);
      rome_reply_ack(intf, frame);
    } break;
    case ROME_MID_ASSERV_SET_XY: {
      double a;
      hposition_get_a(&position, &a);
      int16_t x = frame->asserv_set_xya.x;
      int16_t y = frame->asserv_set_xya.y;
      hposition_set(&position,x,y,a);
      htrajectory_reset_carrot(&trajectory);
      rome_reply_ack(intf, frame);
    } break;
    case ROME_MID_ASSERV_SET_A: {
      vect_xy_t xy;
      hposition_get_xy(&position,&xy);
      int16_t a = (frame->asserv_set_xya.a)/1000.0;
      hposition_set(&position, xy.x, xy.y, a);
      htrajectory_reset_carrot(&trajectory);
      rome_reply_ack(intf, frame);
    } break;
    case ROME_MID_ASSERV_SET_X_PID: {
      uint16_t p = frame->asserv_set_x_pid.p;
      uint16_t i = frame->asserv_set_x_pid.i;
      uint16_t d = frame->asserv_set_x_pid.d;
      int32_t max_in = frame->asserv_set_x_pid.max_in;
      int32_t max_I = frame->asserv_set_x_pid.max_I;
      int32_t max_out = frame->asserv_set_x_pid.max_out;
      pid_set_gains(&pid_x, p, i, d);
      pid_set_maximums(&pid_x, max_in, max_I, max_out);
      rome_reply_ack(intf, frame);
    } break;
    case ROME_MID_ASSERV_SET_Y_PID: {
      uint16_t p = frame->asserv_set_y_pid.p;
      uint16_t i = frame->asserv_set_y_pid.i;
      uint16_t d = frame->asserv_set_y_pid.d;
      int32_t max_in = frame->asserv_set_y_pid.max_in;
      int32_t max_I = frame->asserv_set_y_pid.max_I;
      int32_t max_out = frame->asserv_set_y_pid.max_out;
      pid_set_gains(&pid_y, p, i, d);
      pid_set_maximums(&pid_y, max_in, max_I, max_out);
      rome_reply_ack(intf, frame);
    } break;
    case ROME_MID_ASSERV_SET_A_PID: {
      uint16_t p = frame->asserv_set_a_pid.p;
      uint16_t i = frame->asserv_set_a_pid.i;
      uint16_t d = frame->asserv_set_a_pid.d;
      int32_t max_in = frame->asserv_set_a_pid.max_in;
      int32_t max_I = frame->asserv_set_a_pid.max_I;
      int32_t max_out = frame->asserv_set_a_pid.max_out;
      pid_set_gains(&pid_angle, p, i, d);
      pid_set_maximums(&pid_angle, max_in, max_I, max_out);
      rome_reply_ack(intf, frame);
    } break;
    case ROME_MID_ASSERV_SET_A_QRAMP: {
      uint16_t dot = frame->asserv_set_a_qramp.dot;
      uint16_t dotdot = frame->asserv_set_a_qramp.dotdot;
      quadramp_set_1st_order_vars(&qramp_angle, dot, dot);
      quadramp_set_2nd_order_vars(&qramp_angle, dotdot, dotdot);
      rome_reply_ack(intf, frame);
    } break;
    case ROME_MID_ASSERV_SET_HTRAJ_A_SPEED: {
      float speed = frame->asserv_set_htraj_a_speed.speed;
      float acc   = frame->asserv_set_htraj_a_speed.acc;
      htrajectory_setASpeed(&trajectory, speed, acc);
      rome_reply_ack(intf, frame);
    } break;
    case ROME_MID_ASSERV_SET_HTRAJ_XY_CRUISE: {
      float speed = frame->asserv_set_htraj_xy_cruise.speed;
      float acc   = frame->asserv_set_htraj_xy_cruise.acc;
      htrajectory_setXYCruiseSpeed(&trajectory, speed, acc);
      rome_reply_ack(intf, frame);
    } break;
    case ROME_MID_ASSERV_SET_HTRAJ_XY_STEERING: {
      float speed = frame->asserv_set_htraj_xy_steering.speed;
      float acc   = frame->asserv_set_htraj_xy_steering.acc;
      htrajectory_setXYSteeringSpeed(&trajectory, speed, acc);
      rome_reply_ack(intf, frame);
    } break;
    case ROME_MID_ASSERV_SET_HTRAJ_XY_STOP: {
      float speed = frame->asserv_set_htraj_xy_stop.speed;
      float acc   = frame->asserv_set_htraj_xy_stop.acc;
      htrajectory_setXYStopSpeed(&trajectory, speed, acc);
      rome_reply_ack(intf, frame);
    } break;
    case ROME_MID_ASSERV_SET_HTRAJ_XYSTEERING_WINDOW: {
      float r = frame->asserv_set_htraj_xysteering_window.r;
      htrajectory_setSteeringWindow(&trajectory, r);
      rome_reply_ack(intf, frame);
    } break;
    case ROME_MID_ASSERV_SET_HTRAJ_STOP_WINDOWS: {
      float xy = frame->asserv_set_htraj_stop_windows.xy;
      float angle = frame->asserv_set_htraj_stop_windows.angle;
      htrajectory_setStopWindows(&trajectory, xy, angle);
      rome_reply_ack(intf, frame);
    } break;
    case ROME_MID_ASSERV_SET_ZGYRO_SCALE: {
      INTLVL_DISABLE_ALL_BLOCK() {
        zgyro_scale = frame->asserv_set_zgyro_scale.zscale;
        ROME_LOGF(&rome, DEBUG, "new gyro scale is %e", (double)zgyro_scale);
        rome_reply_ack(intf, frame);
      }
    } break;
    case ROME_MID_ASSERV_GYRO_INTEGRATION: {
      INTLVL_DISABLE_ALL_BLOCK() {
        bool b = frame->asserv_gyro_integration.b;
        adxrs_integrate(b);
        ROME_LOGF(&rome, DEBUG, "gyro integration is %s", b ? "active":"inactive");
        rome_reply_ack(intf, frame); 
      }
    } break;

    case ROME_MID_ASSERV_SET_SERVO: {
      INTLVL_DISABLE_ALL_BLOCK(){
#if defined(GALIPETTE)
        ROME_LOGF(&rome, DEBUG, "set servo %u to %u", frame->asserv_set_servo.id, frame->asserv_set_servo.value);
        servo_set(frame->asserv_set_servo.id,frame->asserv_set_servo.value);
#endif
        rome_reply_ack(intf, frame);
      }
    } break;

    case ROME_MID_ASSERV_SET_CMATRIX: {
      INTLVL_DISABLE_ALL_BLOCK() {

        hrobot_motors_invmatrix_correct[0] = frame->asserv_set_cmatrix.m0;
        hrobot_motors_invmatrix_correct[1] = frame->asserv_set_cmatrix.m1;
        hrobot_motors_invmatrix_correct[2] = frame->asserv_set_cmatrix.m2;
        hrobot_motors_invmatrix_correct[3] = frame->asserv_set_cmatrix.m3;
        hrobot_motors_invmatrix_correct[4] = frame->asserv_set_cmatrix.m4;
        hrobot_motors_invmatrix_correct[5] = frame->asserv_set_cmatrix.m5;
        hrobot_motors_invmatrix_correct[6] = frame->asserv_set_cmatrix.m6;
        hrobot_motors_invmatrix_correct[7] = frame->asserv_set_cmatrix.m7;
        hrobot_motors_invmatrix_correct[8] = frame->asserv_set_cmatrix.m8;
      }
    }

    // forward orders to R3D2 board
    case ROME_MID_R3D2_SET_ROTATION:
    case ROME_MID_R3D2_SET_BLIND_SPOT:
      rome_send(&rome_r3d2, frame);
      break;

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

void _adxrs_update(void) {
  adxrs_capture_manual(zgyro_scale);
}

int main(void)
{
  // Booting
  for(int k=0;k<4;k++) {
    leds[k] = PORTPIN(Q,k);
    portpin_dirset(leds+k);
    portpin_outclr(leds+k);
  }
  PORTQ.OUT = 1;
  // Initialize clocks
  clock_init();

  // Initialize UART
  uart_init();
  uart_fopen(CLI_USER_UART);

  // Initialize Timer
  timer_init();
  PORTQ.OUT = 2;
  TIMER_SET_CALLBACK_US(E0, 'A', CONTROL_SYSTEM_PERIOD_US, CONTROL_SYSTEM_INTLVL, vcs_update);

  INTLVL_ENABLE_ALL();
  __asm__("sei");

  PORTQ.OUT = 3;
  // Initialize ROME
  rome_intf_init(&rome);
  rome.uart = uartD0;
  rome.handler = rome_handler;

  rome_intf_init(&rome_r3d2);
  rome_r3d2.uart = uartE0;
  rome_r3d2.handler = rome_r3d2_handler;

#if defined(GALIPETTE)
  servos_init();
#endif

  PORTQ.OUT = 4;

  //--------------------------------------------------------
  // CS
  //--------------------------------------------------------
  cs_initialize();

  // setup ADXRS update task
  TIMER_SET_CALLBACK_US(E0, 'B', ADXRS_PERIOD_US, ADXRS_INTLVL, _adxrs_update);

  PORTQ.OUT = 5;
  adxrs_calibration_mode(true);
  while(1) {
    int16_t offset = adxrs_get_offset();
    double offset_sqsd = adxrs_get_offset_sqsd();

    // exit calibration if sqsd is low enough
#if defined(GALIPETTE)
    if(offset_sqsd < 160.0)
#else
    if(offset_sqsd < 50.0)
#endif
    {
      ROME_LOGF(&rome, DEBUG, "gyro cal done ! off=%d sqsd=%f", offset, offset_sqsd);
      break;
    }

    // downlink debug infos
    ROME_LOGF(&rome, DEBUG, "gyro cal off=%d sqsd=%f", offset, offset_sqsd);
    rome_handle_input(&rome);
    _delay_ms(500);
  }
  adxrs_calibration_mode(false);

  // remove break
  hrobot_break(0);


  printf("-- reboot --\n");
  //----------------------------------------------------------------------
  PORTQ.OUT = 6;
  for(;;) {

    PORTQ.OUT++;
   
    _delay_ms(10);
    rome_handle_input(&rome);
    rome_handle_input(&rome_r3d2);
  }
}

