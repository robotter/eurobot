#ifndef CYLINDER_H
#define CYLINDER_H

#include "jevois_cam.h"

typedef enum{
  CYLINDER_INIT = 0,
  CYLINDER_IDLE,
  CYLINDER_CHECK_EMPTY_PREPARE,
  CYLINDER_CHECK_EMPTY,
  CYLINDER_CHECK_EMPTY_ORDER_MOVING,
  CYLINDER_CHECK_EMPTY_MOVING, //5
  CYLINDER_BALLEATER_PRE_TAKE,
  CYLINDER_BALLEATER_TAKE,
  CYLINDER_BALLEATER_POST_TAKE,
  CYLINDER_EATING_FIND_EMPTY,
  CYLINDER_EATING_FIND_EMPTY_ORDER_MOVING, //10
  CYLINDER_EATING_FIND_EMPTY_MOVING,
  CYLINDER_THROWBALLS_POS,
  CYLINDER_THROWBALLS_PREPARE,
  CYLINDER_THROWBALLS_PREPARE_MOVING,
  CYLINDER_THROWBALLS_POS_THROW, //15
  CYLINDER_THROWBALLS_WAIT_FLYING,
  CYLINDER_THROWBALLS_WAIT_TURBINE_STOP,
  CYLINDER_THROWBALLS_ORDER_MOVING,
  CYLINDER_THROWBALLS_MOVING,
  CYLINDER_THROWBALLS_WHEELMODE, //20
  CYLINDER_THROWBALLS_WHEELMODE_PREPARE_ORDER_MOVE,
  CYLINDER_THROWBALLS_WHEELMODE_PREPARE_MOVING,
  CYLINDER_THROWBALLS_WHEELMODE_ORDER_TURN,
  CYLINDER_THROWBALLS_WHEELMODE_TURNING,
}cylinder_state_t;

typedef enum{
  CYLINDER_THROW_NONE,
  CYLINDER_THROW_WATERTOWER,
  CYLINDER_THROW_TREATMENT,
  CYLINDER_THROW_OFFCUP,
}cylinder_throw_t;

typedef struct {
  jevois_color_t robot_color;
  cylinder_state_t state;

  uint8_t position;

  uint32_t moving_ts;
  uint32_t drop_ts;
  uint32_t throw_ts;
  cylinder_throw_t throw_mode;

  jevois_color_t ball_color[CYLINDER_NB_POS];

  uint8_t tm_state;
}cylinder_t;


extern cylinder_t cylinder;

void cylinder_update(void);
void cylinder_init(void);
void cylinder_shutdown(void);

void cylinder_check_empty(void);
void cylinder_load_water(void);
void cylinder_throw_watertower(void);
void cylinder_thrash_treatment(void);

uint8_t cylinder_count_empty_slots(void);
uint8_t cylinder_count_good_water(void);
uint8_t cylinder_count_bad_water(void);
uint8_t cylinder_get_tm_state(void);

void cylinder_set_robot_color(jevois_color_t color);
#endif//CYLINDER_H