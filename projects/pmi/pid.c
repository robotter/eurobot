#include <math.h>
#include <string.h>
#include <avr/eeprom.h>
#include <avarix/intlvl.h>
#include "pid.h"


// RBR+
#define EEPROM_MAGIC  0x5242522b


void pid_init(pid_t *p)
{
  // init to NAN, used to detect first computation run
  p->last_error = NAN;

  p->integral = 0;
  p->consign = 0;
  p->feedback = 0;
}


void pid_do_computation(pid_t *p)
{
  // compute error and error filter
  double error = p->consign - p->feedback;
  if(isnan(p->last_error)) {
    p->last_error = error;
  }
  double f_error = p->last_error + p->conf.d_alpha * (error - p->last_error);
  p->last_error = f_error;

  // compute output
  double output = error * p->conf.kp + f_error * p->conf.kd + p->integral * p->conf.ki;
  if(output > p->conf.max_output) {
    output = p->conf.max_output;
  } else if(output < -p->conf.max_output) {
    output = -p->conf.max_output;
  }
  p->output = output;

  // compute integral
  p->integral += error;
  if(p->integral > p->conf.max_integral) {
    p->integral = p->conf.max_integral;
  } else if(p->integral < -p->conf.max_integral) {
    p->integral = -p->conf.max_integral;
  }
}


void pid_reset(pid_t *p)
{
  p->integral = 0;
  p->last_error = NAN;
}


void pid_conf_load(pid_t *p, const pid_conf_t *conf, const pid_conf_t *def)
{
  eeprom_read_block(&p->conf, conf, sizeof(*conf));
  if(p->conf.magic != EEPROM_MAGIC) {
    memcpy(&p->conf, def, sizeof(p->conf));
  }
}

void pid_conf_save(pid_t *p, pid_conf_t *conf)
{
  INTLVL_DISABLE_ALL_BLOCK() {
    p->conf.magic = EEPROM_MAGIC;
    eeprom_update_block(&p->conf, conf, sizeof(*conf));
  }
}


