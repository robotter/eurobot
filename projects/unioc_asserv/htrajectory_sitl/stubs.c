#include "../robot_cs.h"
#include "../avoidance.h"

#include <stdio.h>
#include <stdlib.h>

avoidance_t avoidance;

double robot_x,robot_y,robot_a;

void robot_cs_set_a_consign(robot_cs_t* rcs, int32_t angle) {}

void robot_cs_set_xy_consigns(robot_cs_t* rcs, int32_t x, int32_t y) {

  fprintf(stderr,"%d,%d\n",x,y);

}

void hposition_get_xy( hrobot_position_t *hpos, vect_xy_t *pv ) {

  pv->x = robot_x;
  pv->y = robot_y;
}

void hposition_get_a( hrobot_position_t *hpos, double *pa ) {
  *pa = robot_a;
}

void hposition_set( hrobot_position_t* hpos, double x, double y, double alpha) {

  robot_x = x;
  robot_y = y;
  robot_a = alpha;
}

void hposition_get( hrobot_position_t* hpos, hrobot_vector_t* vec) {
  
  vec->x = robot_x;
  vec->y = robot_y;
  vec->alpha = robot_a;
}

void robot_cs_activate(robot_cs_t* rcs, uint8_t active) {}

void hrobot_set_motors(int32_t vx, int32_t vy, int32_t omega) {}

direction_t avoidance_check(avoidance_t* av) {
  return DIR_NONE;
}
