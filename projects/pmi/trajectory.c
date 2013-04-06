#include <avr/eeprom.h>
#include <avarix/intlvl.h>
#include "trajectory.h"
#include "config.h"


void traj_init(traj_t *t, position_t *p)
{
  t->pos = p;
  t->cons = TRAJ_NO_ASSERV;
  t->flags = 0;
  t->d_cur = 0;
  t->a_cur = 0;
  t->x_cur = 0;
  t->y_cur = 0;
  t->d_cons = 0;
  t->a_cons = 0;
  t->x_cons = 0;
  t->y_cons = 0;
  t->d_start = 0;
  t->a_start = 0;
  t->x_start = 0;
  t->y_start = 0;

  // init with a move
  traj_goto_d(t, 0);
}


void traj_no_asserv(traj_t *t)
{
  INTLVL_DISABLE_ALL_BLOCK() {
    t->cons = TRAJ_NO_ASSERV;
    t->flags &= ~TRAJ_FLAG_ENDED;
    t->flags |= TRAJ_FLAG_NEW_MOVE;
  }
}

void traj_goto_d(traj_t *t, double d)
{
  INTLVL_DISABLE_ALL_BLOCK() {
    t->d_cons = d;
    t->cons = TRAJ_D_MOVE;
    t->flags &= ~TRAJ_FLAG_ENDED;
    t->flags |= TRAJ_FLAG_NEW_MOVE;
  }
}

void traj_goto_a(traj_t *t, double a)
{
  INTLVL_DISABLE_ALL_BLOCK() {
    t->a_cons = a;
    t->cons = TRAJ_A_MOVE;
    t->flags &= ~TRAJ_FLAG_ENDED;
    t->flags |= TRAJ_FLAG_NEW_MOVE;
  }
}

void traj_goto_a_rel(traj_t *t, double a)
{
  INTLVL_DISABLE_ALL_BLOCK() {
    t->a_cons = a;
    t->cons = TRAJ_A_REL_MOVE;
    t->flags &= ~TRAJ_FLAG_ENDED;
    t->flags |= TRAJ_FLAG_NEW_MOVE;
  }
}


static void traj_handle_no_asserv(traj_t *t)
{
  t->d_out = t->d_cur;
  t->a_out = t->a_cur;
  t->flags |= TRAJ_FLAG_ENDED;
}

static void traj_handle_no_move(traj_t *t)
{
  t->d_out = t->d_start;
  t->a_out = t->a_start;
}

static void traj_handle_d_move(traj_t *t)
{
  t->d_out = t->d_start + t->d_cons;
  t->a_out = t->a_cons;
  if(fabs(t->d_out - t->d_cur) < pos_mm_to_tick(t->pos, TRAJECTORY_MARGIN_DIST)) {
    t->flags |= TRAJ_FLAG_ENDED;
  }
}

static void traj_handle_a_move(traj_t *t)
{
  t->a_out = t->a_cons;
  if(fabs(t->a_out - t->a_cur) < pos_deg_to_tick(t->pos, TRAJECTORY_MARGIN_ANGLE)) {
    t->flags |= TRAJ_FLAG_ENDED;
  }
}

static void traj_handle_a_rel_move(traj_t *t)
{
  t->d_out = t->d_cons;
  t->a_out = t->a_start + t->a_cons;
  //TODO missing check to set TRAJ_FLAG_ENDED flag?
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

    if(t->flags & TRAJ_FLAG_NEW_MOVE) {
      t->d_start = t->d_cur;
      t->a_start = t->a_cur;
      t->x_start = t->x_cur;
      t->y_start = t->y_cur;
      t->flags &= ~TRAJ_FLAG_NEW_MOVE;
    }

    switch(t->cons) {
      case TRAJ_NO_ASSERV:
        traj_handle_no_asserv(t);
        break;
      case TRAJ_NO_MOVE:
        traj_handle_no_move(t);
        break;
      case TRAJ_D_MOVE:
        traj_handle_d_move(t);
        break;
      case TRAJ_A_MOVE:
        traj_handle_a_move(t);
        break;
      case TRAJ_A_REL_MOVE:
        traj_handle_a_rel_move(t);
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


