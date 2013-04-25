#ifndef POSITION_H
#define POSITION_H

#include <math.h>
#include <stdint.h>


/// Position configuration
typedef struct { 
  uint32_t magic;

  double left_wheel_ratio;
  double right_wheel_ratio;
  double tick_p_mm;
  double tick_p_180deg;

} position_conf_t;


/// Position state
typedef struct {
  position_conf_t conf;

  double d;
  double a;
  double x;
  double y;

  double llast;
  double rlast;

  double ldelta;
  double rdelta;

} position_t;


/// Initialize position
void pos_init(position_t * p);

// Set encoder values for the next computation
void pos_set_encoder_values(position_t *p, int32_t left, int32_t right);

// Getters
static inline double pos_get_d(const position_t *p) { return p->d; }
static inline double pos_get_a(const position_t *p) { return p->a; }
static inline double pos_get_x(const position_t *p) { return p->x; }
static inline double pos_get_y(const position_t *p) { return p->y; }

// Setters
static inline void pos_set_d(position_t *p, double d) { p->d = d; }
static inline void pos_set_a(position_t *p, double a) { p->a = a; }
static inline void pos_set_x(position_t *p, double x) { p->x = x; }
static inline void pos_set_y(position_t *p, double y) { p->y = y; }

// Compute the new position
void pos_do_computation(position_t *p);

/// Load position configuration from EEPROM
void pos_conf_load(position_t *p, const position_conf_t *conf, const position_conf_t *def);
/// Save position configuration to EEPROM
void pos_conf_save(position_t *p, position_conf_t *conf);


/// Convert millimeters to position ticks
static inline double pos_mm_to_tick(const position_t *p, double mm)
{
  return mm * p->conf.tick_p_mm;
}

/// Convert degrees to position ticks
static inline double pos_deg_to_tick(const position_t *p, double deg)
{
  return deg * p->conf.tick_p_180deg / 180.0;
}

/// Convert radians to position ticks
static inline double pos_rad_to_tick(const position_t *p, double rad)
{
  return rad * p->conf.tick_p_180deg / M_PI;
}

/// Convert position ticks to millimeters
static inline double pos_tick_to_mm(const position_t *p, double tick)
{
  return tick / p->conf.tick_p_mm;
}

/// Convert position ticks to degrees
static inline double pos_tick_to_deg(const position_t *p, double tick)
{
  return tick * 180.0 / p->conf.tick_p_180deg;
}

/// Convert position ticks to radians
static inline double pos_tick_to_rad(const position_t *p, double tick)
{
  return tick * M_PI / p->conf.tick_p_180deg;
}


#endif
