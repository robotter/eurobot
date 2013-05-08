#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <stdint.h>
#include <stdbool.h>
#include "position.h"

#define TRAJ_FLAG_PAUSE  (1 << 0)
#define TRAJ_FLAG_ENDED  (1 << 1)


/// Trajectory consign type
typedef enum {
  TRAJ_NO_ASSERV = 0,
  TRAJ_D_MOVE,
  TRAJ_A_MOVE,
  TRAJ_XY_MOVE,
  TRAJ_XYA_MOVE,

} traj_cons_t;


/// Trajectory configuration
typedef struct {
} traj_conf_t;


/// Trajectory state
typedef struct {
  traj_conf_t conf;

  position_t *pos;
  traj_cons_t cons;
  volatile uint8_t flags;

  double d_cur;
  double a_cur;
  double x_cur;
  double y_cur;
  double d_target;
  double a_target;
  double x_target;
  double y_target;

  double d_out;
  double a_out;

  double d_pause;
  double a_pause;

} traj_t;


/// Initialize trajectory
void traj_init(traj_t *t, position_t *p);

// Set consigns
void traj_no_asserv(traj_t *t);
void traj_goto_d(traj_t *t, double d);
void traj_goto_a(traj_t *t, double a);
void traj_goto_xy(traj_t *t, double x, double y);
void traj_goto_xya(traj_t *t, double x, double y, double a);

static inline bool traj_done(traj_t *t) { return t->flags & TRAJ_FLAG_ENDED; }

/// Compute outputs
void traj_do_computation(traj_t *t);

// Output accessors
static inline double traj_get_d_output(const traj_t * t) { return t->d_out; }
static inline double traj_get_a_output(const traj_t * t) { return t->a_out; }

/// Load trajectory configuration from EEPROM
void traj_conf_load(traj_t *t, const traj_conf_t *conf);
/// Save trajectory configuration to EEPROM
void traj_conf_save(const traj_t *t, traj_conf_t *conf);


#endif
