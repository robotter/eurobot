#ifndef PI_H
#define PI_H

#include <stdint.h>


/// Ramp configuration
typedef struct {
  uint32_t magic;

  double kd;
  double ki;
  double kp;
  double d_alpha;
  double max_integral;
  double max_output;

} pid_conf_t;


/// Ramp state
typedef struct {
  pid_conf_t conf;

  double integral;
  double last_error;
  double consign;
  double feedback;
  double output;

} pid_t;


/// Initialize PID
void pid_init(pid_t *p);

// Setters
//TODO lock?!
static inline void pid_set_consign(pid_t *p, double cons) { p->consign = cons; }
static inline void pid_set_feedback(pid_t *p, double feedback) { p->feedback = feedback; }

// Getters
static inline double pid_get_output(const pid_t *p) { return p->output; }

/// Update values
void pid_do_computation(pid_t *p);

/// Reset PID filter
void pid_reset(pid_t *p);

/// Load PID configuration from EEPROM
void pid_conf_load(pid_t *p, const pid_conf_t *conf, const pid_conf_t *def);
/// Save PID configuration to EEPROM
void pid_conf_save(pid_t *p, pid_conf_t *conf);

#endif
