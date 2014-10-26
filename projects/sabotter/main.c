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
    case ROME_MID_ACK:
      if(frame->ack.fid <= ROME_FRAME_ID_MAX) {
        rome_acks_free_frame_id(frame->ack.fid);
        return; // don't forward
      }
      break;
    case ROME_MID_ASSERV_TM_HTRAJ_DONE:
      robot_state.asserv.xy = frame->asserv_tm_htraj_done.xy;
      robot_state.asserv.a = frame->asserv_tm_htraj_done.a;
      break;
    case ROME_MID_ASSERV_TM_HTRAJ_AUTOSET_DONE:
      robot_state.asserv.autoset = frame->asserv_tm_htraj_autoset_done.b;
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
    case ROME_MID_MECA_TM_ARM:
      robot_state.arm.shoulder = frame->meca_tm_arm.upper;
      robot_state.arm.elbow = frame->meca_tm_arm.elbow;
      robot_state.arm.wrist = frame->meca_tm_arm.wrist;
      break;
    case ROME_MID_MECA_TM_SUCKERS:
      robot_state.suckers.a = frame->meca_tm_suckers.a;
      robot_state.suckers.b = frame->meca_tm_suckers.b;
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
  }
}

/// Scheduler function called at match end
static void match_end(void)
{
  // count time second by second
  static unsigned int match_time = 0;
  if(++match_time >= 89) {
    // end of match
    ROME_SEND_AND_WAIT(ASSERV_ACTIVATE, &rome_asserv, 0);
    ROME_SEND_AND_WAIT(MECA_SET_POWER, &rome_meca, 0);
    portpin_outset(&LED_R_PP);
    portpin_outset(&LED_G_PP);
    portpin_outset(&LED_B_PP);
    for(;;) {
      update_rome_interfaces();
    }
  }
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
  timer_set_callback(timerE0, 'A', TIMER_US_TO_TICKS(E0,UPDATE_TICK_US),
                     UPTIME_INTLVL, update_uptime);
  timer_set_callback(timerE0, 'B', TIMER_US_TO_TICKS(E0,50e3),
                     UPTIME_INTLVL, update_battery);

  // if voltage is low, blink purple led and don't start anything
  if(voltage < BATTERY_ALERT_LIMIT) {
    for(;;) {
      if((get_uptime_us() / 500000) % 2 == 0) {
        portpin_outset(&LED_R_PP);
      } else {
        portpin_outclr(&LED_R_PP);
      }
      update_rome_interfaces();
    }
  }

  // initialize asserv and meca, fold arms, ...
  strat_init();
  team_t team = strat_select_team();
  strat_prepare(team);
  strat_wait_start(team);

  timer_set_callback(timerF0, 'A', TIMER_US_TO_TICKS(F0,1e6),
                     ROME_SEND_INTLVL, match_end);

  //strat_test(team);
  strat_run(team);

  for(;;) {
    update_rome_interfaces();
  }
}

