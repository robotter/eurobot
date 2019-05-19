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
#include <xbee/xbee.h>
#include "strat.h"
#include "common.h"
#include "config.h"
#include "battery_monitor.h"
#include <pathfinding/pathfinding.h>
#include "servos.h"

// battery monitoring
BATTMON_t battery;

// ROME and XBee interfaces
rome_reader_t rome_asserv_reader;
#ifdef UART_MECA
rome_reader_t rome_meca_reader;
#endif
xbee_intf_t xbee_paddock;

#if (defined GALIPEUR)
# define ROME_DEVICE  ROME_ENUM_DEVICE_GALIPEUR_STRAT
#elif (defined GALIPETTE)
# define ROME_DEVICE  ROME_ENUM_DEVICE_GALIPETTE_STRAT
#endif

//pathfinding structure
pathfinding_t pathfinder;

// message handlers

static void rome_asserv_handler(const rome_frame_t *frame)
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
  rome_send(ROME_DST_BROADCAST, frame); //TODO don't broadcast everything
}

static void rome_meca_handler(const rome_frame_t *frame)
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
    #if 0
    case ROME_MID_MECA_TM_CYLINDER_STATE:
      robot_state.cylinder_nb_slots = frame->meca_tm_cylinder_state.nb_slots;
      robot_state.cylinder_nb_empty = frame->meca_tm_cylinder_state.nb_empty;
      robot_state.cylinder_nb_good = frame->meca_tm_cylinder_state.nb_good;
      robot_state.cylinder_nb_bad = frame->meca_tm_cylinder_state.nb_bad;
      break;
    #endif
    default:
      break;
  }

  // forward
  rome_send(ROME_DST_BROADCAST, frame); //TODO don't broadcast everything
}

static void rome_xbee_handler(uint16_t addr, const rome_frame_t *frame)
{
  portpin_outtgl(&LED1_PP);
  switch(frame->mid) {
    case ROME_MID_ACK:
      // should not happen
      rome_free_ack(frame->ack.ack);
      return;
    case ROME_MID_RESET: {
      software_reset();
    } break;
    case ROME_MID_STRAT_TEST:  {
      strat_test();
    } break;
    case ROME_MID_STRAT_SET_SERVO: {
      ROME_LOGF(ROME_DST_PADDOCK, DEBUG, "strat: set servo %u to %u", frame->strat_set_servo.id, frame->strat_set_servo.value);
#if defined(GALIPETTE)
      servo_set(frame->strat_set_servo.id, frame->strat_set_servo.value);
#endif
      rome_reply_ack(ROME_DST_XBEE(addr), frame);
      return;
    } break;
    case ROME_MID_TM_ROBOT_POSITION: {
      if (frame->tm_robot_position.device != ROME_DEVICE){
        robot_state.partner_pos.x = frame->tm_robot_position.x;
        robot_state.partner_pos.y = frame->tm_robot_position.y;
        robot_state.partner_pos.a = frame->tm_robot_position.a/1000.;
      }
      rome_reply_ack(ROME_DST_XBEE(addr), frame);
      return;
    } break;
    case ROME_MID_TM_BATTERY:{
      if(frame->tm_battery.device == ROME_ENUM_DEVICE_BOOMOTTER){
        robot_state.boom_age = uptime_us();
      }
      return;
    } break;
    default:
      break;
  }

  // forward to other interfaces
  rome_send(ROME_DST_ASSERV, frame);
#ifdef UART_MECA
  rome_send(ROME_DST_MECA, frame);
#endif
}

static void xbee_paddock_handler(xbee_intf_t *intf, const xbee_frame_t *frame)
{
  switch(frame->api_id) {
    case XBEE_ID_RX16: {
      const rome_frame_t *rome_frame = rome_parse_frame(frame->rx16.data, frame->length - 5);
      if(rome_frame) {
        const uint16_t addr = (frame->rx16.addr_be << 8) | (frame->rx16.addr_be & 0xff);
        rome_xbee_handler(addr, rome_frame);
      }
    } break;
    default:
      break;  // ignore
  }
}


// Handle input from all ROME interfaces
void update_rome_interfaces(void)
{
  const rome_frame_t *frame;
  while((frame = rome_reader_read(&rome_asserv_reader))) rome_asserv_handler(frame);
#ifdef UART_MECA
  while((frame = rome_reader_read(&rome_meca_reader))) rome_meca_handler(frame);
#else
  (void)rome_meca_handler;
#endif
  xbee_handle_input(&xbee_paddock);
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
    ROME_SEND_TM_BATTERY(ROME_DST_BROADCAST, ROME_DEVICE, voltage);
    portpin_outtgl(&LED0_PP);
  }

}

static void send_messages(void)
{
  ROME_SEND_TM_SCORE(ROME_DST_BROADCAST, ROME_DEVICE, robot_state.points);
}

/// Scheduler function called at match end
static void match_end(void)
{
  // count time second by second
  if(++robot_state.match_time >= 99) {
    ROME_LOG(ROME_DST_PADDOCK,INFO,"End of match");
    // end of match
    ROME_SENDWAIT_ASSERV_ACTIVATE(ROME_DST_ASSERV, 0);
#ifdef UART_MECA
    ROME_SENDWAIT_MECA_SET_POWER(ROME_DST_MECA, 0);
#endif
    portpin_outset(&LED_R_PP);
    portpin_outset(&LED_G_PP);
    portpin_outset(&LED_B_PP);
    for(;;) {
      //send timer for boomotter stuff
      ROME_SEND_TM_MATCH_TIMER(ROME_DST_BROADCAST, ROME_DEVICE, robot_state.match_time);
      idle_delay_ms(500);
    }
  }
}

void rome_send_pathfinding_graph(const pathfinding_t *finder)
{
  for(uint8_t i=0; i<finder->nodes_size; i++) {
    const pathfinding_node_t *node = &finder->nodes[i];
    ROME_SEND_PATHFINDING_NODE(ROME_DST_PADDOCK, i, node->x, node->y, node->neighbors, node->neighbors_size);
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

  // leds
  portpin_dirset(&LED_R_PP);
  portpin_dirset(&LED_G_PP);
  portpin_dirset(&LED_B_PP);

  portpin_dirset(&LED0_PP);
  portpin_dirset(&LED1_PP);
  portpin_dirset(&LED2_PP);
  portpin_dirset(&LED3_PP);

  // Initialize UART
  uart_init();
#ifdef GALIPEUR
  uart_fopen(uartC0);
#endif

  INTLVL_ENABLE_ALL();
  __asm__("sei");


  // Initialize XBee and ROME
  xbee_intf_init(&xbee_paddock, XBEE_PADDOCK_UART);
  xbee_paddock.handler = xbee_paddock_handler;
  rome_reader_init(&rome_asserv_reader, ROME_ASSERV_UART);
#ifdef UART_MECA
  rome_reader_init(&rome_meca_reader, ROME_MECA_UART);
#endif

#if defined(GALIPEUR)
  ROME_LOG(ROME_DST_PADDOCK, INFO, "strat galipeur booting");
#elif defined(GALIPETTE)
  ROME_LOG(ROME_DST_PADDOCK, INFO, "strat galipette booting");
#endif

  // starting cord and color selector
  portpin_dirclr(&STARTING_CORD_PP);
  portpin_dirclr(&COLOR_SELECTOR_PP);

  // Initialize battery monitoring
  BATTMON_Init(&battery);
  BATTMON_monitor(&battery);
  uint16_t voltage = battery.FilterMemory;

#if defined(GALIPETTE)
  servos_init();
#endif

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

  //initialize pathfinding
  pathfinding_set_nodes(&pathfinder, pathfinding_graph);
  rome_send_pathfinding_graph(&pathfinder);

  //initialize partner position
  robot_state.partner_pos.x = 200;
  robot_state.partner_pos.y = -1000;
  robot_state.partner_pos.a = 0;

  robot_state.boom_age = uptime_us();

  // initialize asserv and meca, fold arms, ...
  portpin_outset(&LED_R_PP);
  robot_state.points = 0;
  strat_init();
  portpin_outclr(&LED_R_PP);
  robot_state.team = strat_select_team();

  strat_prepare();

  strat_wait_start();

  robot_state.match_time = 0;
  TIMER_SET_CALLBACK_US(F0, 'A', 1e6, ROME_SEND_INTLVL, match_end);

  strat_run();

  for(;;) {
    idle();
  }
}


