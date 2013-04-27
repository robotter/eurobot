#include <string.h>
#include <avr/eeprom.h>
#include <avarix/intlvl.h>
#include "ramp.h"
#include "config.h"


// RBR?
#define EEPROM_MAGIC  0x5242523f


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


void ramp_conf_load(ramp_t *r, const ramp_conf_t *conf, const ramp_conf_t *def)
{
  eeprom_read_block(&r->conf, conf, sizeof(*conf));
  if(r->conf.magic != EEPROM_MAGIC) {
    memcpy(&r->conf, &def, sizeof(r->conf));
  }
}

void ramp_conf_save(ramp_t *r, ramp_conf_t *conf)
{
  INTLVL_DISABLE_ALL_BLOCK() {
    r->conf.magic = EEPROM_MAGIC;
    eeprom_update_block(&r->conf, conf, sizeof(*conf));
  }
}


