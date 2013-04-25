#include <perlimpinpin/payload/system.h>
#include <perlimpinpin/payload/room.h>
#include <perlimpinpin/payload/log.h>
#include "position.h"
#include "trajectory.h"
#include "ramp.h"
#include "pid.h"
#include "config.h"

extern position_t pos_man;
extern traj_t traj_man;
extern ramp_t ramp_dist;
extern ramp_t ramp_angle;
extern pid_t pid_dist;
extern pid_t pid_angle;


ppp_payload_handler_t *ppp_filter(ppp_intf_t *intf)
{
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

    case ROOM_MID_ASSERV_GET_POSITION: {
      uint16_t x, y, a;
      INTLVL_DISABLE_BLOCK(CONTROL_SYSTEM_INTLVL) {
        x = pos_tick_to_mm(&pos_man, pos_get_x(&pos_man)),
        y = pos_tick_to_mm(&pos_man, pos_get_y(&pos_man)),
        a = 1000*pos_tick_to_rad(&pos_man, pos_get_a(&pos_man));
      }
      ROOM_REPLY_ASSERV_GET_POSITION(intf, pl, x, y, a);
    } break;

    case ROOM_MID_ASSERV_SET_POSITION: {
      double x = pos_mm_to_tick(&pos_man, pl->asserv_set_position.x);
      double y = pos_mm_to_tick(&pos_man, pl->asserv_set_position.y);
      double a = pos_rad_to_tick(&pos_man, pl->asserv_set_position.a/1000.);
      INTLVL_DISABLE_BLOCK(CONTROL_SYSTEM_INTLVL) {
        pos_set_x(&pos_man, x);
        pos_set_y(&pos_man, y);
        pos_set_a(&pos_man, a);
        pos_set_d(&pos_man, 0); // any d value is fine, just use 0
        ramp_reset(&ramp_dist, 0);
        ramp_reset(&ramp_angle, a);
        pid_reset(&pid_dist);
        pid_reset(&pid_angle);
      }
      ROOM_REPLY_ASSERV_SET_POSITION(intf, pl);
    } break;

    case ROOM_MID_ASSERV_GOTO_XY: {
      double x = pos_mm_to_tick(&pos_man, pl->asserv_goto_xy.x);
      double y = pos_mm_to_tick(&pos_man, pl->asserv_goto_xy.y);
      traj_goto_xy(&traj_man, x, y);
      ROOM_REPLY_ASSERV_GOTO_XY(intf, pl);
    } break;

    case ROOM_MID_ASSERV_GOTO_A: {
      double a = pos_rad_to_tick(&pos_man, pl->asserv_goto_a.a/1000.);
      traj_goto_a(&traj_man, a);
      ROOM_REPLY_ASSERV_GOTO_A(intf, pl);
    } break;

    default:
      PPP_LOGF(intf, INFO, "unexpected ROOM message: %u", pl->mid);
      break;
  }
}


