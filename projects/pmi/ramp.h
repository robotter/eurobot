#ifndef RAMP_H
#define RAMP_H

#include <stdint.h>


/// Ramp configuration
typedef struct {
  uint32_t magic;

  double a_max;
  double v_max;

} ramp_conf_t;


/// Ramp state
typedef struct {
  ramp_conf_t conf;

  double cons;
  double v_cur;
  double c_buf;

} ramp_t;


/// Initialize ramp
void ramp_init(ramp_t *r);

/// Set ramp consign
//TODO lock?!
static inline void ramp_set_consign(ramp_t *r, double cons) { r->cons = cons; }

/// Compute the new consign
void ramp_do_computation(ramp_t *r);

/// Get ramp output
static inline double ramp_get_output(const ramp_t *r) { return r->c_buf; }

/// Reset ramp with a new consign
void ramp_reset(ramp_t *r, double cons);


/// Load ramp configuration from EEPROM
void ramp_conf_load(ramp_t *r, const ramp_conf_t *conf, const ramp_conf_t *def);
/// Save ramp configuration to EEPROM
void ramp_conf_save(ramp_t *r, ramp_conf_t *conf);

#endif
