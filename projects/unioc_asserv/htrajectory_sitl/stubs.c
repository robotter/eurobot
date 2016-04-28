#include "../robot_cs.h"
#include "../avoidance.h"

#include <stdio.h>
#include <stdlib.h>

avoidance_t avoidance;

#include "output.h"
output_t output;

void robot_cs_set_a_consign(robot_cs_t* rcs, int32_t angle) {}

void robot_cs_set_xy_consigns(robot_cs_t* rcs, int32_t _x, int32_t _y) {

  output.x_sp = _x;
  output.y_sp = _y;

  double x = 1.0*_x/1000;
  double y = 1.0*_y/1000;

  output.x += MIN(x-output.x,1);
  output.y += MIN(y-output.y,1);

}

void hposition_get_xy( hrobot_position_t *hpos, vect_xy_t *pv ) {

  pv->x = output.x;
  pv->y = output.y;
}

void hposition_get_a( hrobot_position_t *hpos, double *pa ) {
  *pa = output.a;
}

void hposition_set( hrobot_position_t* hpos, double x, double y, double alpha) {

  output.x = x;
  output.y = y;
  output.a = alpha;
}

void hposition_get( hrobot_position_t* hpos, hrobot_vector_t* vec) {
  
  vec->x = output.x;
  vec->y = output.y;
  vec->alpha = output.a;
}

void robot_cs_activate(robot_cs_t* rcs, uint8_t active) {}

void hrobot_set_motors(int32_t vx, int32_t vy, int32_t omega) {}

direction_t avoidance_check(avoidance_t* av) {
  return DIR_NONE;
}
