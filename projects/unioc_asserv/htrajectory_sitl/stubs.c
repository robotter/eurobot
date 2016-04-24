#include "../robot_cs.h"
#include "../avoidance.h"

avoidance_t avoidance;

void robot_cs_set_a_consign(robot_cs_t* rcs, int32_t angle) {}

void robot_cs_set_xy_consigns(robot_cs_t* rcs, int32_t x, int32_t y) {}

void hposition_get_xy( hrobot_position_t *hpos, vect_xy_t *pv ) {}

void hposition_get_a( hrobot_position_t *hpos, double *pa ) {}

void hposition_set( hrobot_position_t* hpos, double x, double y, double alpha) {}

void hposition_get( hrobot_position_t* hpos, hrobot_vector_t* vec) {}

void robot_cs_activate(robot_cs_t* rcs, uint8_t active) {}

void hrobot_set_motors(int32_t vx, int32_t vy, int32_t omega) {}

direction_t avoidance_check(avoidance_t* av) {}
