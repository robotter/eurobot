#include <string.h>
#include <avr/eeprom.h>
#include <avarix/intlvl.h>
#include "position.h"


// RBR!
#define EEPROM_MAGIC  0x52425221

void pos_init(position_t * p)
{
  // Init to NAN, used to detect first computation run.
  // This allows to reset the encoder position to the right value.
  p->llast = NAN;
  p->rlast = NAN;
  
  // Init position to 0. Master will set the real position when ready.
  p->d = 0;
  p->a = 0;
  p->x = 0;
  p->y = 0;
}


void pos_set_encoder_values(position_t *p, int32_t left, int32_t right)
{
  // apply correction ratio
  double lcur = left * p->conf.left_wheel_ratio;
  double rcur = right * p->conf.right_wheel_ratio;

  // init last values if this is the first run
  if(isnan(p->llast)) {
    p->llast = lcur;
  }
  if(isnan(p->rlast)) {
    p->rlast = rcur;
  }

  // compute the delta
  p->ldelta = lcur - p->llast;
  p->rdelta = rcur - p->rlast;

  // save values for the next call
  p->llast = lcur;
  p->rlast = rcur;
}


void pos_conf_load(position_t *p, const position_conf_t *conf, const position_conf_t *def)
{
  eeprom_read_block(&p->conf, conf, sizeof(*conf));
  if(p->conf.magic != EEPROM_MAGIC) {
    memcpy(&p->conf, &def, sizeof(p->conf));
  }
}

void pos_conf_save(position_t *p, position_conf_t *conf)
{
  INTLVL_DISABLE_ALL_BLOCK() {
    p->conf.magic = EEPROM_MAGIC;
    eeprom_update_block(&p->conf, conf, sizeof(*conf));
  }
}


void pos_do_computation(position_t *p)
{
  // compute deltas
  double delta_common = (p->rdelta + p->ldelta)/2.;
  double delta_diff = (p->rdelta - p->ldelta)/2.;

  // compute new position
  p->x += pos_tick_to_mm(p, delta_common) * cos(pos_tick_to_rad(p, p->a + delta_diff/2));
  p->y += pos_tick_to_mm(p, delta_common) * sin(pos_tick_to_rad(p, p->a + delta_diff/2));
  p->d += delta_common;
  p->a += delta_diff;
}

