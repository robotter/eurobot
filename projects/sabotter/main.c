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
#include <avarix/register.h>
#include <clock/clock.h>
#include <util/delay.h>

#include <stdio.h>
#include <stdlib.h>
#include <uart/uart.h>
#include <timer/timer.h>
#include <timer/uptime.h>
#include <idle/idle.h>
#include <rome/rome.h>
#include "strat.h"
#include "common.h"
#include "config.h"
#include "battery_monitor.h"

// battery monitoring
BATTMON_t battery;

// ROME interfaces
rome_intf_t rome_asserv;
rome_intf_t rome_meca;
rome_intf_t rome_paddock;


// message handlers

static void rome_asserv_handler(rome_intf_t *intf, const rome_frame_t *frame)
{
  switch(frame->mid) {
    case ROME_MID_ACK: {
      uint8_t ack = frame->ack.ack;
      if(rome_ack_in_range(ack)) {
        rome_free_ack(ack);
        return; // don't forward
      }
    } break;
    case ROME_MID_ASSERV_TM_HTRAJ_DONE:
      robot_state.asserv.xy = frame->asserv_tm_htraj_done.xy;
      robot_state.asserv.a = frame->asserv_tm_htraj_done.a;
      break;
    case ROME_MID_ASSERV_TM_HTRAJ_AUTOSET_DONE:
      robot_state.asserv.autoset = frame->asserv_tm_htraj_autoset_done.b;
      break;
    case ROME_MID_ASSERV_TM_GYRO_CALIBRATION:
      robot_state.gyro_calibration = frame->asserv_tm_gyro_calibration.active;
      break;
    case ROME_MID_R3D2_TM_DETECTION: {
      uint8_t i = frame->r3d2_tm_detection.i;
      if(i < R3D2_OBJECTS_MAX) {
        r3d2_object_t *object = robot_state.r3d2.objects+i;
        object->detected = frame->r3d2_tm_detection.detected;
        object->a = frame->r3d2_tm_detection.a/1000.;
        object->r = frame->r3d2_tm_detection.r;
      }
    } break;
    case ROME_MID_ASSERV_TM_XYA:{
      robot_state.current_pos.x = frame->asserv_tm_xya.x;
      robot_state.current_pos.y = frame->asserv_tm_xya.y;
      robot_state.current_pos.a = frame->asserv_tm_xya.a/1000.;
    } break;
    case ROME_MID_ASSERV_TM_HTRAJ_CARROT_XY:{
      robot_state.carrot.x = frame->asserv_tm_htraj_carrot_xy.x;
      robot_state.carrot.y = frame->asserv_tm_htraj_carrot_xy.y;
    } break;
    case ROME_MID_ASSERV_TM_HTRAJ_PATH_INDEX:{
      robot_state.asserv.path_i = frame->asserv_tm_htraj_path_index.i;
      robot_state.asserv.path_n = frame->asserv_tm_htraj_path_index.size;
    } break;
    case ROME_MID_ASSERV_TM_BUMPERS_STATE:{
      robot_state.bumpers.left  = frame->asserv_tm_bumpers_state.b0;
      robot_state.bumpers.right = frame->asserv_tm_bumpers_state.b1;
    } break;

    default:
      break;
  }

  // forward
  rome_send(&rome_paddock, frame);
}

static void rome_meca_handler(rome_intf_t *intf, const rome_frame_t *frame)
{
  switch(frame->mid) {
    case ROME_MID_ACK: {
      uint8_t ack = frame->ack.ack;
      if(rome_ack_in_range(ack)) {
        rome_free_ack(ack);
        return; // don't forward
      }
    } break;
    case ROME_MID_MECA_TM_STATE:
      robot_state.meca_state = frame->meca_tm_state.state;
      break;
    case ROME_MID_MECA_TM_CYLINDER_STATE:
      robot_state.cylinder_nb_slots = frame->meca_tm_cylinder_state.nb_slots;
      robot_state.cylinder_nb_empty = frame->meca_tm_cylinder_state.nb_empty;
      robot_state.cylinder_nb_good = frame->meca_tm_cylinder_state.nb_good;
      robot_state.cylinder_nb_bad = frame->meca_tm_cylinder_state.nb_bad;
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
      rome_free_ack(frame->ack.ack);
      return;
    case ROME_MID_RESET: {
      software_reset();
    } break;
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


static void update_battery(void)
{
  // function called every 50ms

  // call battery mgmt and downlink every 500ms
  static uint8_t it=0; it++;
  if(it > 10) {
    it=0;
    // get battery voltage
    BATTMON_monitor(&battery);
    uint16_t voltage = battery.FilterMemory;
    // send telemetry message
    ROME_SEND_STRAT_TM_BATTERY(&rome_paddock, voltage);
    PORTA.DIRSET = 0x0E;
    PORTA.OUTTGL = 0x0E;
  }

}

static void send_messages(void)
{
  ROME_SEND_STRAT_TM_SCORE(&rome_paddock, robot_state.points);
}

/// Scheduler function called at match end
static void match_end(void)
{
  // count time second by second
  static unsigned int match_time = 0;
  if(++match_time >= 99) {
    ROME_LOG(&rome_paddock,INFO,"End of match");
    // end of match
    ROME_SENDWAIT_ASSERV_ACTIVATE(&rome_asserv, 0);
    ROME_SENDWAIT_MECA_SET_POWER(&rome_meca, 0);
    portpin_outset(&LED_R_PP);
    portpin_outset(&LED_G_PP);
    portpin_outset(&LED_B_PP);
    for(;;) {
      idle();
    }
  }
}


int main(void)
{
  // Booting

  // Initialize clocks
  clock_init();

  #ifdef GALIPETTE
  // HACK : in unioc, Pin D0  and D1 are outputs
  PORTD.DIRCLR = _BV(0) | _BV(1);
  #endif

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

  // starting cord and color selector
  portpin_dirclr(&STARTING_CORD_PP);
  portpin_dirclr(&COLOR_SELECTOR_PP);

  // leds
  portpin_dirset(&LED_R_PP);
  portpin_dirset(&LED_G_PP);
  portpin_dirset(&LED_B_PP);

  portpin_dirset(&LED0_PP);
  portpin_dirset(&LED1_PP);
  portpin_dirset(&LED2_PP);
  portpin_dirset(&LED3_PP);

  // Initialize battery monitoring
  BATTMON_Init(&battery);
  BATTMON_monitor(&battery);
  uint16_t voltage = battery.FilterMemory;

  // Initialize timers
  timer_init();
  uptime_init();
  TIMER_SET_CALLBACK_US(E0, 'B', 50e3, INTLVL_HI, update_battery);
  idle_set_callback(rome_update, update_rome_interfaces);
  idle_set_callback(rome_telemetry, send_messages);

  // if voltage is low, blink red led and don't start anything
  if(voltage < BATTERY_ALERT_LIMIT) {
    for(;;) {
      if((uptime_us() / 500000) % 2 == 0) {
        portpin_outset(&LED_R_PP);
      } else {
        portpin_outclr(&LED_R_PP);
      }
      idle();
    }
  }

  // initialize asserv and meca, fold arms, ...
  portpin_outset(&LED_R_PP);
  robot_state.points = 0;
  strat_init();
  portpin_outclr(&LED_R_PP);
  robot_state.team = strat_select_team();

  strat_test();

  strat_prepare();

  strat_wait_start();

  TIMER_SET_CALLBACK_US(F0, 'A', 1e6, ROME_SEND_INTLVL, match_end);

  strat_run();

  for(;;) {
    idle();
  }
}


