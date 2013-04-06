#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <stdint.h>
#include "position.h"

#define TRAJ_FLAG_PAUSE  (1 << 0)
#define TRAJ_FLAG_ENDED  (1 << 1)
#define TRAJ_FLAG_BACKWARD  (1 << 2)
#define TRAJ_FLAG_NEW_MOVE  (1 << 3)


/// Trajectory consign type
typedef enum {
  TRAJ_NO_ASSERV = 0,
  TRAJ_NO_MOVE,
  TRAJ_D_MOVE,
  TRAJ_A_MOVE,
  TRAJ_A_REL_MOVE,
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
  uint8_t flags;

  double d_start;
  double a_start;
  double x_start;
  double y_start;
  double d_cur;
  double a_cur;
  double x_cur;
  double y_cur;
  double d_cons;
  double a_cons;
  double x_cons;
  double y_cons;

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
void traj_goto_a_rel(traj_t *t, double a);

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
