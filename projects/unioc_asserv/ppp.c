#include <avr/eeprom.h>
#include <perlimpinpin/payload/system.h>
#include <perlimpinpin/payload/room.h>
#include <perlimpinpin/payload/log.h>

#include "htrajectory.h"
#include "hposition_manager.h"
#include "robot_cs.h"

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
      double a_rad;
      vect_xy_t xy;
      INTLVL_DISABLE_BLOCK(INTLVL_LO) {
        hposition_get_xy(&position, &xy);
        hposition_get_a(&position, &a_rad);
      }
      ROOM_REPLY_ASSERV_GET_POSITION(intf, pl, xy.x, xy.y, a_rad*1000);
    } break;

    case ROOM_MID_ASSERV_SET_POSITION: {
      double x,y,a;
      x = pl->asserv_set_position.x;
      y = pl->asserv_set_position.y;
      a = pl->asserv_set_position.a/1000.0;
      INTLVL_DISABLE_BLOCK(INTLVL_LO) { 
        hposition_set(&position, x, y, a);
      }
      ROOM_REPLY_ASSERV_SET_POSITION(intf, pl);
    } break;

    case ROOM_MID_ASSERV_GOTO_XY: {
      double x = pl->asserv_goto_xy.x;
      double y = pl->asserv_goto_xy.y;
      htrajectory_gotoXY(&trajectory, x, y);
      ROOM_REPLY_ASSERV_GOTO_XY(intf, pl);
    } break;

    case ROOM_MID_ASSERV_GOTO_XY_REL: {
      double x = pl->asserv_goto_xy.x;
      double y = pl->asserv_goto_xy.y;
      htrajectory_gotoXY_R(&trajectory, x, y);
      ROOM_REPLY_ASSERV_GOTO_XY_REL(intf, pl);
    } break;

    case ROOM_MID_ASSERV_GOTO_A: {
      double a = pl->asserv_goto_a.a/1000.0;
      htrajectory_gotoA(&trajectory, a);
      ROOM_REPLY_ASSERV_GOTO_A(intf, pl);
    } break;

    case ROOM_MID_ASSERV_GOTO_A_REL: {
      double a = pl->asserv_goto_a.a/1000.0;
      htrajectory_gotoA_R(&trajectory, a);
      ROOM_REPLY_ASSERV_GOTO_A_REL(intf, pl);
    } break;

    case ROOM_MID_ASSERV_STATUS: {
      bool arrived_a = htrajectory_doneA(&trajectory);
      bool arrived_xy = htrajectory_doneXY(&trajectory);
      ROOM_REPLY_ASSERV_STATUS(intf, pl, arrived_a, arrived_xy);
    } break;

    case ROOM_MID_ASSERV_ACTIVATE: {
      bool activate = pl->asserv_activate.activate;
      robot_cs_activate(&robot_cs, activate);
      ROOM_REPLY_ASSERV_ACTIVATE(intf, pl);
    }break;

    case ROOM_MID_GALIPEUR_FORCE_THRUST: {
      int32_t vx,vy,omegaz;
      vx = pl->galipeur_force_thrust.vx;
      vy = pl->galipeur_force_thrust.vy;
      omegaz = pl->galipeur_force_thrust.omegaz;
      hrobot_set_motors(vx,vy,omegaz);
      ROOM_REPLY_GALIPEUR_FORCE_THRUST(intf, pl);
    } break;

    case ROOM_MID_GALIPEUR_SET_A_SPEED: {
      double speed = pl->galipeur_set_a_speed.speed;
      double acc = pl->galipeur_set_a_speed.acc;
      htrajectory_setASpeed(&trajectory, speed, acc);
      ROOM_REPLY_GALIPEUR_SET_A_SPEED(intf, pl);
    }break;

    case ROOM_MID_GALIPEUR_SET_XY_CRUISE_SPEED: {
      double speed = pl->galipeur_set_xy_cruise_speed.speed;
      double acc = pl->galipeur_set_xy_cruise_speed.acc;
      htrajectory_setXYCruiseSpeed(&trajectory, speed, acc);
      ROOM_REPLY_GALIPEUR_SET_XY_CRUISE_SPEED(intf, pl);
    }break;

    case ROOM_MID_GALIPEUR_SET_XY_STEERING_SPEED: {
      double speed = pl->galipeur_set_xy_steering_speed.speed;
      double acc = pl->galipeur_set_xy_steering_speed.acc;
      htrajectory_setXYSteeringSpeed(&trajectory, speed, acc);
      ROOM_REPLY_GALIPEUR_SET_XY_STEERING_SPEED(intf, pl);
    }break;

    case ROOM_MID_GALIPEUR_SET_XY_STOP_SPEED: {
      double speed = pl->galipeur_set_xy_stop_speed.speed;
      double acc = pl->galipeur_set_xy_stop_speed.acc;
      htrajectory_setXYStopSpeed(&trajectory, speed, acc);
      ROOM_REPLY_GALIPEUR_SET_XY_STOP_SPEED(intf, pl);
    }break;
  
    case ROOM_MID_GALIPEUR_SET_WINDOWS: {
      double steering_xywin = pl->galipeur_set_windows.steering_xy_win;
      double stop_xywin = pl->galipeur_set_windows.stop_xy_win;
      double awin = pl->galipeur_set_windows.stop_a_win;
      htrajectory_setSteeringWindow(&trajectory, steering_xywin);
      htrajectory_setStopWindows(&trajectory, stop_xywin, awin);
      ROOM_REPLY_GALIPEUR_SET_WINDOWS(intf, pl);
    }break;

    default:
      PPP_LOGF(intf, INFO, "unexpected ROOM message: %u", pl->mid);
      break;
  }
}


