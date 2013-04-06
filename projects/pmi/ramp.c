#include <avr/eeprom.h>
#include "ramp.h"
#include "config.h"


void ramp_init(ramp_t *r)
{
  r->cons = 0;
  r->v_cur = 0;
  r->c_buf = 0;
}


void ramp_do_computation(ramp_t *r)
{
  // check move direction
  if(r->v_cur >= 0) {
    // check if we should increase or decrease our speed
    if((r->v_cur * r->v_cur) / (2 * r->conf.a_max) + r->c_buf < r->cons) {
      r->v_cur += r->conf.a_max * ((double)CONTROL_SYSTEM_PERIOD_US/1000000.0);
      if(r->v_cur > r->conf.v_max) {
        r->v_cur = r->conf.v_max;
      }
    } else {
      r->v_cur -= r->conf.a_max * ((double)CONTROL_SYSTEM_PERIOD_US/1000000.0);
      if(r->v_cur < -r->conf.v_max) {
        r->v_cur = -r->conf.v_max;
      }
    }
  } else {
    // check if we should increase or decrease our speed
    if(-(r->v_cur * r->v_cur) / (2 * r->conf.a_max) + r->c_buf < r->cons) {
      r->v_cur += r->conf.a_max * ((double)CONTROL_SYSTEM_PERIOD_US/1000000.0);
      if(r->v_cur > r->conf.v_max) {
        r->v_cur = r->conf.v_max;
      }
    } else {
      r->v_cur -= r->conf.a_max * ((double)CONTROL_SYSTEM_PERIOD_US/1000000.0);
      if(r->v_cur < -r->conf.v_max) {
        r->v_cur = -r->conf.v_max;
      }
    }
  }

  r->c_buf += r->v_cur * ((double)CONTROL_SYSTEM_PERIOD_US/1000000.0);
}


void ramp_reset(ramp_t *r, double cons)
{
  r->cons = cons;
  r->c_buf = cons;
  r->v_cur = 0;
}


void ramp_conf_load(ramp_t *r, const ramp_conf_t *conf)
{
  eeprom_read_block(&r->conf, conf, sizeof(*conf));
}

void ramp_conf_save(const ramp_t *r, ramp_conf_t *conf)
{
  //TODO lock while saving?!
  eeprom_update_block(&r->conf, conf, sizeof(*conf));
}


