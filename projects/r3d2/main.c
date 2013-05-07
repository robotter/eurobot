#include <math.h>
#include <avarix/intlvl.h>
#include <avarix/portpin.h>
#include <clock/clock.h>
#include <uart/uart.h>
#include <timer/timer.h>
#include <perlimpinpin/perlimpinpin.h>
#include <perlimpinpin/payload/system.h>
#include <perlimpinpin/payload/room.h>
#include <perlimpinpin/payload/log.h>
#include "r3d2.h"
#include "config.h"


static r3d2_data_t r3d2_data;
ppp_intf_t pppintf;


ppp_payload_handler_t *ppp_filter(ppp_intf_t *intf)
{
  portpin_outset(&LED_COM_PP);
  if(intf->rstate.header.dst != 0xFF && intf->rstate.header.dst != intf->addr) {
    return NULL;
  }
  switch(intf->rstate.header.pltype) {
    case PPP_TYPE_SYSTEM:
      return ppp_payload_handler_system;
    case PPP_TYPE_ROOM:
      return ppp_payload_handler_room;
    default:
      return NULL;
  }
}


void room_message_handler(ppp_intf_t *intf, room_payload_t *pl)
{
  switch(pl->mid) {
    case ROOM_MID_R3D2_SET_STATE:
      if(pl->r3d2_set_state.state) {
        r3d2_start();
      } else {
        r3d2_stop();
      }
      ROOM_REPLY_R3D2_SET_STATE(intf, pl);
      break;
    case ROOM_MID_R3D2_SET_MOTOR_SPEED:
      r3d2_set_motor_speed(pl->r3d2_set_motor_speed.speed);
      ROOM_REPLY_R3D2_SET_MOTOR_SPEED(intf, pl);
      break;

    case ROOM_MID_R3D2_GET_CONF: {
      const r3d2_conf_t *conf = r3d2_get_conf();
      ROOM_REPLY_R3D2_GET_CONF(
          intf, pl,
          conf->motor_speed, conf->motor_timeout,
          conf->angle_offset*1000, conf->dist_coef*1000);
      } break;
    case ROOM_MID_R3D2_SET_CONF: {
      r3d2_conf_t conf = {
        .motor_speed = pl->r3d2_set_conf.motor_speed,
        .motor_timeout = pl->r3d2_set_conf.motor_timeout,
        .angle_offset = pl->r3d2_set_conf.angle_offset/1000.0,
        .dist_coef = pl->r3d2_set_conf.dist_coef/1000.0,
      };
      r3d2_set_conf(&conf);
      ROOM_REPLY_R3D2_SET_CONF(intf, pl);
      } break;
    case ROOM_MID_R3D2_LOAD_CONF:
      r3d2_conf_load();
      ROOM_REPLY_R3D2_LOAD_CONF(intf, pl);
      break;
    case ROOM_MID_R3D2_SAVE_CONF:
      r3d2_conf_save();
      ROOM_REPLY_R3D2_SAVE_CONF(intf, pl);
      break;

    case ROOM_MID_R3D2_CALIBRATE_ANGLE:
      INTLVL_DISABLE_BLOCK(INTLVL_MED) {
        r3d2_calibrate_angle(pl->r3d2_calibrate_angle.a/1000.);
      }
      ROOM_REPLY_R3D2_CALIBRATE_ANGLE(intf, pl);
      break;

    case ROOM_MID_R3D2_CALIBRATE_DIST:
      INTLVL_DISABLE_BLOCK(INTLVL_MED) {
        r3d2_calibrate_dist(pl->r3d2_calibrate_dist.r/1000.);
      }
      ROOM_REPLY_R3D2_CALIBRATE_DIST(intf, pl);
      break;

    default:
      PPP_LOGF(intf, INFO, "unexpected ROOM message: %u", pl->mid);
      break;
  }
}


void update_data_cb(void)
{
  r3d2_update(&r3d2_data);
}

void send_ppp_events_cb(void)
{
  static uint8_t last_count = 0;
  // north east south west
  bool leds[4] = {false, false, false, false};

  uint8_t i = 0;
  for(i=0; i<r3d2_data.count; i++) {
    const r3d2_object_t *object = r3d2_data.objects+i;
    ROOM_SEND_R3D2_DETECTED(&pppintf, 0xFF, i, object->angle*1000, object->dist*1000);
    int8_t iangle = object->angle/M_PI_4;
    leds[((iangle+1)/2) % 4] = true;
  }
  for(; i<last_count; i++) {
    ROOM_SEND_R3D2_DISAPPEARED(&pppintf, 0xFF, i);
  }

  if(leds[0]) portpin_outset(&LED_WEST_PP); else portpin_outclr(&LED_WEST_PP);
  if(leds[1]) portpin_outset(&LED_SOUTH_PP); else portpin_outclr(&LED_SOUTH_PP);
  if(leds[2]) portpin_outset(&LED_NORTH_PP); else portpin_outclr(&LED_NORTH_PP);
  if(leds[3]) portpin_outset(&LED_EAST_PP); else portpin_outclr(&LED_EAST_PP);

  last_count = r3d2_data.count;
}


int main(void)
{
  clock_init();
  timer_init();
  uart_init();
  uart_fopen(UART_PPP);
  CPU_SREG |= CPU_I_bm;
  INTLVL_ENABLE_ALL();

  portpin_dirset(&LED_RUN_PP);
  portpin_dirset(&LED_ERROR_PP);
  portpin_dirset(&LED_COM_PP);
  portpin_dirset(&LED_WEST_PP);
  portpin_dirset(&LED_EAST_PP);
  portpin_dirset(&LED_NORTH_PP);
  portpin_dirset(&LED_SOUTH_PP);

  // init PPP
  pppintf.filter = ppp_filter;
  pppintf.uart = UART_PPP;
  pppintf.addr = PPP_ADDR;

  ppp_intf_init(&pppintf);
  room_set_message_handler(room_message_handler);
  // send a system RESET to signal that we have booted
  ppp_send_system_reset(&pppintf);

  // init R3D2
  r3d2_init();
  r3d2_conf_load();
  r3d2_start();

  timer_set_callback(timerE0, 'A', TIMER_US_TO_TICKS(E0,20000), INTLVL_LO, update_data_cb);
  timer_set_callback(timerE0, 'B', TIMER_US_TO_TICKS(E0,200000), INTLVL_LO, send_ppp_events_cb);

  // main loop
  for(;;) {
    ppp_intf_update(&pppintf);
  }
}

