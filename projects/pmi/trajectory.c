#include <avr/eeprom.h>
#include <avarix/intlvl.h>
#include <math.h>
#include "trajectory.h"
#include "config.h"

#include <perlimpinpin/payload/log.h>
extern ppp_intf_t pppintf;

static double _fmod_pipi(double x) {
  return fmod(x+M_PI, 2*M_PI)-M_PI;
}

static double _compute_angle_consign(traj_t *t, double current, double target)
{
  double consign;
  current = pos_tick_to_rad(t->pos, current);
  target = pos_tick_to_rad(t->pos, target);

  consign = current - _fmod_pipi(current);
  target = _fmod_pipi(target);

  double c1 = consign + target;
  double c2 = c1 + 2 * M_PI;
  double c3 = c1 - 2 * M_PI;

  if(fabs(c1 - current) < fabs(c2 - current)) {
    if(fabs(c1 - current) < fabs(c3 - current)) {
      return pos_rad_to_tick(t->pos, c1);
    }
    else {
      return pos_rad_to_tick(t->pos, c3);
    }
  }
  else {
    if(fabs(c2 - current) < fabs(c3 - current)) {
      return pos_rad_to_tick(t->pos, c2);
    }
    else {
      return pos_rad_to_tick(t->pos, c3);
    }
  }
}

void traj_init(traj_t *t, position_t *p)
{
  t->pos = p;
  t->cons = TRAJ_NO_ASSERV;
  t->flags = 0;
  t->d_cur = 0;
  t->a_cur = 0;
  t->x_cur = 0;
  t->y_cur = 0;
  t->d_target = 0;
  t->a_target = 0;
  t->x_target = 0;
  t->y_target = 0;

  // init with a move
  traj_goto_d(t, 0);
}


void traj_no_asserv(traj_t *t)
{
  INTLVL_DISABLE_BLOCK(CONTROL_SYSTEM_INTLVL) {
    t->cons = TRAJ_NO_ASSERV;
    t->flags &= ~TRAJ_FLAG_ENDED;
  }
}

void traj_goto_d(traj_t *t, double d)
{
  INTLVL_DISABLE_BLOCK(CONTROL_SYSTEM_INTLVL) {
    t->d_target = t->d_cur + d;
    t->cons = TRAJ_D_MOVE;
    t->flags &= ~TRAJ_FLAG_ENDED;
  }
}

void traj_goto_a(traj_t *t, double a)
{
  INTLVL_DISABLE_BLOCK(CONTROL_SYSTEM_INTLVL) {
    t->a_target = a;
    t->cons = TRAJ_A_MOVE;
    t->flags &= ~TRAJ_FLAG_ENDED;
  }
}

void traj_goto_xy(traj_t *t, double x, double y)
{
  INTLVL_DISABLE_BLOCK(CONTROL_SYSTEM_INTLVL) {
    t->x_target = x;
    t->y_target = y;
    t->d_target = NAN;
    t->cons = TRAJ_XY_MOVE;
    t->flags &= ~TRAJ_FLAG_ENDED;
  }
}

void traj_goto_xya(traj_t *t, double x, double y, double a)
{
  INTLVL_DISABLE_BLOCK(CONTROL_SYSTEM_INTLVL) {
    t->x_target = x;
    t->y_target = y;
    t->a_target = a;
    t->d_target = NAN;
    t->cons = TRAJ_XYA_MOVE;
    t->flags &= ~TRAJ_FLAG_ENDED;
  }
}


static void traj_handle_no_asserv(traj_t *t)
{
  t->d_out = t->d_cur;
  t->a_out = t->a_cur;
  t->flags |= TRAJ_FLAG_ENDED;
}

static void traj_handle_d_move(traj_t *t)
{
  t->d_out = t->d_target;
  t->a_out = t->a_target;
  if(fabs(t->d_out - t->d_cur) < pos_mm_to_tick(t->pos, TRAJECTORY_MARGIN_DIST)) {
    t->flags |= TRAJ_FLAG_ENDED;
  }
}

static void traj_handle_a_move(traj_t *t)
{
  t->a_out = t->a_target;
  t->a_out = _compute_angle_consign(t, t->a_cur, t->a_target);
  if(fabs(t->a_out - t->a_cur) < pos_deg_to_tick(t->pos, TRAJECTORY_MARGIN_ANGLE)) {
    t->flags |= TRAJ_FLAG_ENDED;
  }
}

static void traj_handle_xy_move(traj_t *t)
{
  double xy_margin = pos_mm_to_tick(t->pos, TRAJECTORY_MARGIN_DIST);
  double dx = t->x_target - t->x_cur;
  double dy = t->y_target - t->y_cur;
  double dd = dx*dx + dy*dy;

  if(dd < xy_margin*xy_margin) {
    // XY target reached
    t->flags |= TRAJ_FLAG_ENDED;
  } else {
    // check angle to destination
    double target = pos_rad_to_tick(t->pos, atan2(dy, dx));
    t->a_out = _compute_angle_consign(t, t->a_cur, target);

    if(fabs(t->a_out - t->a_cur) < pos_deg_to_tick(t->pos, TRAJECTORY_MARGIN_ANGLE)) {
      // angle is fine, update d target
      t->d_out = t->d_cur + sqrt(dd);
    } else {
      // need to turn
      t->d_out = t->d_cur;
    }
  }
}

static void traj_handle_xya_move(traj_t *t)
{
  double xy_margin = pos_mm_to_tick(t->pos, TRAJECTORY_MARGIN_DIST);
  double dx = t->x_target - t->x_cur;
  double dy = t->y_target - t->y_cur;
  double dd = dx*dx + dy*dy;

  if(dd < xy_margin*xy_margin) {
    // XY target reached, check final angle
    return traj_handle_a_move(t);
  } else {
    // check angle to destination
    t->a_out = atan2(dy, dx);
    if(fabs(t->a_out - t->a_cur) < pos_deg_to_tick(t->pos, TRAJECTORY_MARGIN_ANGLE)) {
      // angle is fine, update d target
      t->d_out = t->d_cur + sqrt(dd);
    } else {
      // need to turn
      t->d_out = t->d_cur;
    }
  }
}


void traj_do_computation(traj_t *t)
{
  // copy current position values
  t->d_cur = pos_get_d(t->pos);
  t->a_cur = pos_get_a(t->pos);
  t->y_cur = pos_get_y(t->pos);
  t->x_cur = pos_get_x(t->pos);

  if(t->flags & TRAJ_FLAG_PAUSE) {
    t->d_out = t->d_pause;
    t->a_out = t->a_pause;
  } else {
    switch(t->cons) {
      case TRAJ_NO_ASSERV:
        traj_handle_no_asserv(t);
        break;
      case TRAJ_D_MOVE:
        traj_handle_d_move(t);
        break;
      case TRAJ_A_MOVE:
        traj_handle_a_move(t);
        break;
      case TRAJ_XY_MOVE:
        traj_handle_xy_move(t);
        break;
      case TRAJ_XYA_MOVE:
        traj_handle_xya_move(t);
        break;
      default:
        traj_no_asserv(t);
        break;
    }
  }
}


void traj_conf_load(traj_t *t, const traj_conf_t *conf)
{
  eeprom_read_block(&t->conf, conf, sizeof(*conf));
}

void traj_conf_save(const traj_t *t, traj_conf_t *conf)
{
  //TODO lock while saving?!
  eeprom_update_block(&t->conf, conf, sizeof(*conf));
}


