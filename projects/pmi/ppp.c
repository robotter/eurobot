#include <avr/eeprom.h>
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

extern EEMEM position_conf_t pos_man_conf;
extern EEMEM traj_conf_t traj_man_conf;
extern EEMEM ramp_conf_t ramp_dist_conf;
extern EEMEM ramp_conf_t ramp_angle_conf;
extern EEMEM pid_conf_t pid_dist_conf;
extern EEMEM pid_conf_t pid_angle_conf;


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

    case ROOM_MID_ASSERV_GOTO_D: {
      double d = pos_mm_to_tick(&pos_man, pl->asserv_goto_d.d);
      traj_goto_d(&traj_man, d);
      ROOM_REPLY_ASSERV_GOTO_D(intf, pl);
    } break;

    case ROOM_MID_ASSERV_STATUS: {
      bool arrived_a = traj_man.flags & TRAJ_FLAG_ENDED;
      bool arrived_xy = traj_man.flags & TRAJ_FLAG_ENDED;
      ROOM_REPLY_ASSERV_STATUS(intf, pl, arrived_a, arrived_xy);
    } break;

    case ROOM_MID_PMI_POSITION_SET_CONF: {
      INTLVL_DISABLE_BLOCK(CONTROL_SYSTEM_INTLVL) {
        pos_man.conf.left_wheel_ratio = pl->pmi_position_set_conf.left_wheel_ratio;
        pos_man.conf.right_wheel_ratio = pl->pmi_position_set_conf.right_wheel_ratio;
        pos_man.conf.tick_p_mm = pl->pmi_position_set_conf.tick_p_mm;
        pos_man.conf.tick_p_180deg = pl->pmi_position_set_conf.tick_p_180deg;
      }
      ROOM_REPLY_PMI_POSITION_SET_CONF(intf, pl);
    } break;
    case ROOM_MID_PMI_POSITION_SAVE_CONF: {
      pos_conf_save(&pos_man, &pos_man_conf);
      ROOM_REPLY_PMI_POSITION_SAVE_CONF(intf, pl);
    } break;

    case ROOM_MID_PMI_RAMP_DIST_SET_CONF: {
      INTLVL_DISABLE_BLOCK(CONTROL_SYSTEM_INTLVL) {
        ramp_dist.conf.a_max = pl->pmi_ramp_dist_set_conf.a_max;
        ramp_dist.conf.v_max = pl->pmi_ramp_dist_set_conf.v_max;
      }
      ROOM_REPLY_PMI_RAMP_DIST_SET_CONF(intf, pl);
    } break;
    case ROOM_MID_PMI_RAMP_DIST_SAVE_CONF: {
      ramp_conf_save(&ramp_dist, &ramp_dist_conf);
      ROOM_REPLY_PMI_RAMP_DIST_SAVE_CONF(intf, pl);
    } break;

    case ROOM_MID_PMI_RAMP_ANGLE_SET_CONF: {
      INTLVL_DISABLE_BLOCK(CONTROL_SYSTEM_INTLVL) {
        ramp_angle.conf.a_max = pl->pmi_ramp_angle_set_conf.a_max;
        ramp_angle.conf.v_max = pl->pmi_ramp_angle_set_conf.v_max;
      }
      ROOM_REPLY_PMI_RAMP_ANGLE_SET_CONF(intf, pl);
    } break;
    case ROOM_MID_PMI_RAMP_ANGLE_SAVE_CONF: {
      ramp_conf_save(&ramp_angle, &ramp_angle_conf);
      ROOM_REPLY_PMI_RAMP_ANGLE_SAVE_CONF(intf, pl);
    } break;

    case ROOM_MID_PMI_PID_DIST_SET_CONF: {
      INTLVL_DISABLE_BLOCK(CONTROL_SYSTEM_INTLVL) {
        pid_dist.conf.kd = pl->pmi_pid_dist_set_conf.kd;
        pid_dist.conf.ki = pl->pmi_pid_dist_set_conf.ki;
        pid_dist.conf.kp = pl->pmi_pid_dist_set_conf.kp;
        pid_dist.conf.d_alpha = pl->pmi_pid_dist_set_conf.d_alpha;
        pid_dist.conf.max_integral = pl->pmi_pid_dist_set_conf.max_integral;
        pid_dist.conf.max_output = pl->pmi_pid_dist_set_conf.max_output;
      }
      ROOM_REPLY_PMI_PID_DIST_SET_CONF(intf, pl);
    } break;
    case ROOM_MID_PMI_PID_DIST_SAVE_CONF: {
      pid_conf_save(&pid_dist, &pid_dist_conf);
      ROOM_REPLY_PMI_PID_DIST_SAVE_CONF(intf, pl);
    } break;

    case ROOM_MID_PMI_PID_ANGLE_SET_CONF: {
      INTLVL_DISABLE_BLOCK(CONTROL_SYSTEM_INTLVL) {
        pid_angle.conf.kd = pl->pmi_pid_angle_set_conf.kd;
        pid_angle.conf.ki = pl->pmi_pid_angle_set_conf.ki;
        pid_angle.conf.kp = pl->pmi_pid_angle_set_conf.kp;
        pid_angle.conf.d_alpha = pl->pmi_pid_angle_set_conf.d_alpha;
        pid_angle.conf.max_integral = pl->pmi_pid_angle_set_conf.max_integral;
        pid_angle.conf.max_output = pl->pmi_pid_angle_set_conf.max_output;
      }
      ROOM_REPLY_PMI_PID_ANGLE_SET_CONF(intf, pl);
    } break;
    case ROOM_MID_PMI_PID_ANGLE_SAVE_CONF: {
      pid_conf_save(&pid_angle, &pid_angle_conf);
      ROOM_REPLY_PMI_PID_ANGLE_SAVE_CONF(intf, pl);
    } break;

    case ROOM_MID_PMI_SAVE_ALL_CONF: {
      pos_conf_save(&pos_man, &pos_man_conf);
      ramp_conf_save(&ramp_dist, &ramp_dist_conf);
      ramp_conf_save(&ramp_angle, &ramp_angle_conf);
      pid_conf_save(&pid_dist, &pid_dist_conf);
      pid_conf_save(&pid_angle, &pid_angle_conf);
      ROOM_REPLY_PMI_SAVE_ALL_CONF(intf, pl);
    } break;

    case ROOM_MID_PMI_GET_BATTERY_INFORMATION: {

      ROOM_REPLY_PMI_GET_BATTERY_INFORMATION(intf, pl, 42);
    } break;

    default:
      PPP_LOGF(intf, INFO, "unexpected ROOM message: %u", pl->mid);
      break;
  }
}


