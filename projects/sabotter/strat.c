#include <math.h>
#include <stdlib.h>
#include <clock/clock.h>
#include <util/delay.h>
#include <avarix/portpin.h>
#include <rome/rome.h>
#include <timer/uptime.h>
#include <idle/idle.h>
#include <rome/rome.h>
#include "strat.h"
#include "config.h"

#define ANGLE_TYPE__ float
#include "modulo.inc.h"
#undef ANGLE_TYPE__

#define SPOT_ELEVATOR_LENGTH 83
#define GOTO_TIMEOUT_MS  10000
#define DEFAULT_WAIT_MS  2000

#define AUTOSET_OFFSET 112

extern rome_intf_t rome_asserv;
extern rome_intf_t rome_meca;
extern rome_intf_t rome_paddock;

robot_state_t robot_state;
// we don't put 'kx' on robot_state to allow it to be static
// this avoids optimization errors
static float robot_kx;

typedef enum {
  ROBOT_SIDE_LEFT = 0,
  ROBOT_SIDE_RIGHT,
  ROBOT_SIDE_BACK,
} robot_side_t;

typedef enum {
  TABLE_SIDE_LEFT = 0,
  TABLE_SIDE_RIGHT,
  TABLE_SIDE_UP,
  TABLE_SIDE_DOWN,
} table_side_t;

#define TABLE_SIDE_MAIN (robot_state.team == TEAM_GREEN ? TABLE_SIDE_LEFT : TABLE_SIDE_RIGHT)
#define TABLE_SIDE_AUX (robot_state.team == TEAM_GREEN ? TABLE_SIDE_RIGHT : TABLE_SIDE_LEFT)

#define ROBOT_SIDE_MAIN (robot_state.team == TEAM_GREEN ? ROBOT_SIDE_RIGHT : ROBOT_SIDE_LEFT)
#define ROBOT_SIDE_AUX  (robot_state.team == TEAM_GREEN ? ROBOT_SIDE_LEFT : ROBOT_SIDE_RIGHT)
#define ROBOT_SIDE_BALLEATER (ROBOT_SIDE_LEFT)
#define ROBOT_SIDE_TURBINE (ROBOT_SIDE_RIGHT)
#define ROBOT_SIDE_CUBE_CLAW (ROBOT_SIDE_LEFT)
#define AUTOSET_MAIN (robot_state.team == TEAM_GREEN ? AUTOSET_LEFT : AUTOSET_RIGHT)
#define AUTOSET_AUX  (robot_state.team == TEAM_GREEN ? AUTOSET_RIGHT : AUTOSET_LEFT)
#define KX(x) (robot_kx*(x))
#define KC(green,purple) (robot_state.team == TEAM_GREEN ? (green) : (purple))
#define KA(a) (robot_kx*(a))

// align robot face along table side
float arfast (robot_side_t face, table_side_t side){
  switch(face){
    case ROBOT_SIDE_LEFT:
      switch(side){
        case TABLE_SIDE_LEFT:
          return M_PI/6;
        case TABLE_SIDE_RIGHT:
          return -5*M_PI/6;
        case TABLE_SIDE_UP:
          return -M_PI/3;
        case TABLE_SIDE_DOWN:
          return 2*M_PI/3;
      }
      break;
    case ROBOT_SIDE_RIGHT:
      switch(side){
        case TABLE_SIDE_LEFT:
          return 5*M_PI/6;
        case TABLE_SIDE_RIGHT:
          return -M_PI/6;
        case TABLE_SIDE_UP:
          return M_PI/3;
        case TABLE_SIDE_DOWN:
          return -2*M_PI/3;
      }
      break;
    case ROBOT_SIDE_BACK:
      switch(side){
        case TABLE_SIDE_LEFT:
          return -M_PI/2;
        case TABLE_SIDE_RIGHT:
          return M_PI/2;
        case TABLE_SIDE_UP:
          return M_PI;
        case TABLE_SIDE_DOWN:
          return 0;
      }
      break;
    default:
      break;
  }
  return 0;
}

void update_score(uint16_t points){
  robot_state.points += points;
  ROME_LOGF(&rome_paddock,INFO,"score : %u", robot_state.points);
}

typedef enum {
  ORDER_SUCCESS = 0,
  ORDER_DETECTION,
  ORDER_TIMEOUT,
  ORDER_FAILURE,
} order_result_t;

/// delay for some ms
void idle_delay_ms(uint32_t time_ms){
  uint32_t start_time_us = uptime_us();
  while((uptime_us() - start_time_us) < (time_ms*1000))
    idle();
}


/// Check an order_result_t, return it if not success
#define ORDER_CHECK(expr)  do { \
    order_result_t or = (expr); \
    if(or != ORDER_SUCCESS) { \
      return or; \
    } \
  } while(0)

// Aliases using default waiting time
#define goto_xya(x,y,a)  goto_xya_wait((x),(y),(a),GOTO_TIMEOUT_MS)
#define goto_xya_synced(x,y,a)  goto_xya_synced_wait((x),(y),(a),GOTO_TIMEOUT_MS)
#define goto_xya_rel(x,y,a)  goto_xya_rel_wait((x),(y),(a),GOTO_TIMEOUT_MS)
#define goto_traj(xy,a)  goto_traj_wait((xy),(a),GOTO_TIMEOUT_MS)
#define goto_traj_n(xy,n,a)  goto_traj_n_wait((xy),(n),(a),GOTO_TIMEOUT_MS)
#define goto_xy_rel_align_course(x,y,c)  goto_xy_rel_align_course_wait((x),(y),(c),GOTO_TIMEOUT_MS)
#define go_pick_spot(x,y,s)  go_pick_spot_wait((x),(y),(s),GOTO_TIMEOUT_MS)


// Return true if an opponent is detected
bool opponent_detected(void)
{
  for(uint8_t i=0; i<R3D2_OBJECTS_MAX; i++) {
    r3d2_object_t *object = &robot_state.r3d2.objects[i];
    if(object->detected && object->r < R3D2_AVOID_DISTANCE) {
      return true;
    }
  }
  return false;
}

#define IN_RANGE(x,min,max) ((x) >= (min) && (x) <= (max))
#define R3D2_OFFSET (0)
#define DEFAULT_DETECTION_FOV  (M_PI/2)
#define DEFAULT_DETECTION_WAIT_MS 2000

// Return true if an opponent is detected within an arc
bool opponent_detected_in_arc(float angle, float fov, float distance)
{
  float d_a,min,max;
  min = float_modulo__(angle - fov/2, 0, 2*M_PI);
  max = float_modulo__(angle + fov/2, 0, 2*M_PI);
  if(min > max)
    max += 2*M_PI;
  for(uint8_t i=0; i<R3D2_OBJECTS_MAX; i++) {
    r3d2_object_t *object = &robot_state.r3d2.objects[i];
    d_a = float_modulo__(object->a+robot_state.current_pos.a+R3D2_OFFSET, 0, 2*M_PI);
    bool in_range = IN_RANGE(d_a,min,max)||IN_RANGE(d_a+2*M_PI,min,max);
    // note: object->a is in ]-2*Pi;0]
    if(object->detected && object->r < distance && in_range) {
      return true;
    }
  }
  return false;
}

float angle_from_carrot(int16_t cx, int16_t cy)
{
  int16_t dx = cx - robot_state.current_pos.x;
  int16_t dy = cy - robot_state.current_pos.y;
  float angle = atan2(dy,dx);
  return angle;
}

static void set_xya_wait(double x, double y, double a) {
  ROME_SENDWAIT_ASSERV_SET_XYA(&rome_asserv, x, y, 1000.0*a);
}

typedef enum{
  DETECTION_PATH_FREE = 0,
  DETECTION_PATH_BLOCKED,
  DETECTION_PATH_CLEARED,
} detection_path_t;

// check if oponent is on the way
// returns 0 if path is clear
// stops and wait DEFAULT_DETECTION_WAIT_MS if it is not and returns -1 after a timeout
// if opponent comes too close, disable asserv to prepare for impact !
// if the opponent go away before the end of the timeout, return
detection_path_t opponent_detected_carrot(int16_t x, int16_t y){
  float angle = angle_from_carrot(x,y);
  uint32_t start_time_us = uptime_us();
  uint32_t time;
  if(opponent_detected_in_arc(angle,DEFAULT_DETECTION_FOV, R3D2_AVOID_DISTANCE)) {
    ROME_LOG(&rome_paddock,INFO,"goto : opponent detected");
    ROME_SENDWAIT_ASSERV_GOTO_XY_REL(&rome_asserv, -R3D2_AVOID_MOVEBACK*cos(angle), -R3D2_AVOID_MOVEBACK*sin(angle), 0);
    for(;;){
      if(opponent_detected_in_arc(angle,DEFAULT_DETECTION_FOV, R3D2_STOP_DISTANCE)) {
        ROME_LOG(&rome_paddock,INFO,"goto : opponent too close");
        ROME_SENDWAIT_ASSERV_GOTO_XY_REL(&rome_asserv, 0, 0, 0);
        ROME_SENDWAIT_ASSERV_ACTIVATE(&rome_asserv, 0);
        for(;;){
          time = uptime_us() - start_time_us;
          if((time) > ((uint32_t) 1000*DEFAULT_DETECTION_WAIT_MS)){
            ROME_LOGF(&rome_paddock,INFO,"goto : timeout %ld > %ld", time, (uint32_t) 1000*DEFAULT_DETECTION_WAIT_MS);
            ROME_SENDWAIT_ASSERV_GOTO_XY_REL(&rome_asserv, 0, 0, 0);
            ROME_SENDWAIT_ASSERV_ACTIVATE(&rome_asserv, 1);
            return DETECTION_PATH_BLOCKED;
          }
          if(!opponent_detected_in_arc(angle,DEFAULT_DETECTION_FOV, R3D2_STOP_DISTANCE)){
            ROME_LOG(&rome_paddock,INFO,"goto : close opponent went away, resume initial trajectory");
            ROME_SENDWAIT_ASSERV_GOTO_XY_REL(&rome_asserv, 0, 0, 0);
            ROME_SENDWAIT_ASSERV_ACTIVATE(&rome_asserv, 1);
            return DETECTION_PATH_CLEARED;
          }
          idle();
        }
        ROME_SENDWAIT_ASSERV_ACTIVATE(&rome_asserv, 1);
      }
      else{
        time = uptime_us() - start_time_us;
        if((time) > ((uint32_t) 1000*DEFAULT_DETECTION_WAIT_MS)){
          ROME_LOGF(&rome_paddock,INFO,"goto : timeout %ld > %ld", time, (uint32_t) 1000*DEFAULT_DETECTION_WAIT_MS);
          return DETECTION_PATH_BLOCKED;
        }
        if(!opponent_detected_in_arc(angle,DEFAULT_DETECTION_FOV, R3D2_AVOID_DISTANCE+R3D2_AVOID_MOVEBACK)){
          ROME_LOG(&rome_paddock,INFO,"goto : opponent went away, resume initial trajectory");
          return DETECTION_PATH_CLEARED;
        }
        idle();
      }
    }
  }
  return DETECTION_PATH_FREE;

}

/// Go to given position, avoid opponents
static order_result_t goto_xya_wait(int16_t x, int16_t y, float a, uint16_t timeout_ms)
{
  //int16_t dx = x - robot_state.current_pos.x;
  //int16_t dy = y - robot_state.current_pos.y;
  //float angle = atan2(dy,dx);

  uint32_t start_time_us = uptime_us();
  for(;;) {
    ROME_SENDWAIT_ASSERV_GOTO_XY(&rome_asserv, x, y, 1000*a);
    robot_state.asserv.xy = 0;
    robot_state.asserv.a = 0;
    for(;;) {
      uint32_t time = uptime_us() - start_time_us;
      if((time) > ((uint32_t) timeout_ms*1000)){
        ROME_LOGF(&rome_paddock,INFO,"goto_xya : timeout %ld > %ld", time, (uint32_t)timeout_ms*1000);
        return ORDER_TIMEOUT;
        }
      if(robot_state.asserv.xy && robot_state.asserv.a) {
        return ORDER_SUCCESS;
      }
      if(opponent_detected_carrot(x,y)){
        return ORDER_DETECTION;
      }
      idle();
    }
  }
}

/// Go to given position, avoid opponents
static order_result_t goto_xya_synced_wait(int16_t x, int16_t y, float a, uint16_t timeout_ms)
{
  //int16_t dx = x - robot_state.current_pos.x;
  //int16_t dy = y - robot_state.current_pos.y;
  //float angle = atan2(dy,dx);

  uint32_t start_time_us = uptime_us();
  for(;;) {
    ROME_SENDWAIT_ASSERV_GOTO_XYA_SYNCED(&rome_asserv, x, y, 1000*a);
    robot_state.asserv.xy = 0;
    robot_state.asserv.a = 0;
    for(;;) {
      uint32_t time = uptime_us() - start_time_us;
      if((time) > ((uint32_t) timeout_ms*1000)){
        ROME_LOGF(&rome_paddock,INFO,"goto_xya_synced : timeout %ld > %ld", time, (uint32_t)timeout_ms*1000);
        return ORDER_TIMEOUT;
      }
      if(robot_state.asserv.xy && robot_state.asserv.a) {
        return ORDER_SUCCESS;
      }
      detection_path_t dp = opponent_detected_carrot(x,y);
      if(dp == DETECTION_PATH_BLOCKED) {
        return ORDER_DETECTION;
      }
      if(dp == DETECTION_PATH_CLEARED) {
        //resend order
        break;
      }
      idle();
    }
  }
}

/// Execute trajectory, avoid opponents
#define goto_traj_wait(xy,a,w) goto_traj_n_wait((xy), sizeof(xy)/sizeof(int16_t), (a), (w))
static order_result_t goto_traj_n_wait(int16_t* xy, uint8_t n, float a, uint16_t timeout_ms)
{
  uint8_t path_i = 0;
  uint32_t start_time_us = uptime_us();
  for(;;) {
    uint8_t ln = n - path_i;
    ROME_SENDWAIT_ASSERV_RUN_TRAJ(&rome_asserv,1000*a,xy+path_i*2,ln);
    robot_state.asserv.xy = 0;
    robot_state.asserv.a = 0;
    robot_state.asserv.path_i = path_i;
    robot_state.asserv.path_n = ln/2;
    for(;;) {
      uint32_t time = uptime_us() - start_time_us;
      if((time) > ((uint32_t) timeout_ms*1000)){
        ROME_LOGF(&rome_paddock,INFO,"goto_traj : timeout %ld > %ld", time, (uint32_t)timeout_ms*1000);
        return ORDER_TIMEOUT;
        }
      if(robot_state.asserv.xy && robot_state.asserv.a) {
        return ORDER_SUCCESS;
      }
      path_i = robot_state.asserv.path_i;
      detection_path_t dp = opponent_detected_carrot(xy[2*path_i],xy[2*path_i+1]);
      if(dp == DETECTION_PATH_BLOCKED) {
        return ORDER_DETECTION;
      }
      if(dp == DETECTION_PATH_CLEARED) {
        //resend order
        break;
      }
      idle();
    }
  }
}

/// Go to given position, with robot panning and scanning, avoid opponents
void goto_xya_panning(int16_t x, int16_t y, float pan_angle)
{
  for(;;) {
    ROME_SENDWAIT_ASSERV_GOTO_XYA_PANNING(&rome_asserv, x, y, 1000*pan_angle);
    robot_state.asserv.xy = 0;
    robot_state.asserv.a = 0;
    for(;;) {
      if(robot_state.asserv.xy && robot_state.asserv.a) {
        return;
      }
      detection_path_t dp = opponent_detected_carrot(x,y);
      if(dp == DETECTION_PATH_BLOCKED) {
        return;
      }
      if(dp == DETECTION_PATH_CLEARED) {
        //resend order
        break;
      }
      idle();
    }
  }
}

/// Go to given relative position, avoid opponents
static order_result_t goto_xya_rel_wait(int16_t x, int16_t y, float a, uint16_t timeout_ms)
{
  uint32_t start_time_us = uptime_us();
  for(;;) {
    ROME_SENDWAIT_ASSERV_GOTO_XY_REL(&rome_asserv, x, y, 1000*a);
    robot_state.asserv.xy = 0;
    robot_state.asserv.a = 0;
    for(;;) {
      uint32_t time = uptime_us() - start_time_us;
      if((time) > ((uint32_t) timeout_ms*1000)){
        ROME_LOGF(&rome_paddock,INFO,"goto_traj : timeout %ld > %ld", time, (uint32_t)timeout_ms*1000);
        return ORDER_TIMEOUT;
      }
      if(robot_state.asserv.xy && robot_state.asserv.a) {
        return ORDER_SUCCESS;
      }
      detection_path_t dp = opponent_detected_carrot(x,y);
      if(dp == DETECTION_PATH_BLOCKED) {
        return ORDER_DETECTION;
      }
      if(dp == DETECTION_PATH_CLEARED) {
        //do not resend order, it would be osolete since we already moved
        return ORDER_DETECTION;
      }
      idle();
    }
  }
}

order_result_t goto_xy_rel_align_course_wait(int16_t x, int16_t y, bool claws_first, uint16_t timeout_ms){

  float angle;
  //get angle of movement
  angle = atan2(y,x)-M_PI/2;
  if(claws_first)
    angle += M_PI;

  //compute relative angle consign
  angle -= robot_state.current_pos.a;

  return goto_xya_rel_wait(x,y,angle,timeout_ms);
}

void go_around(int16_t cx,int16_t cy, float a){
  int16_t rx = robot_state.current_pos.x ;
  int16_t ry = robot_state.current_pos.y ;
  float   ra = robot_state.current_pos.a ;
  a += M_PI/6;
  uint8_t nb_step = 6;
  int16_t x = rx - cx;
  int16_t y = ry - cy;

  for(int i=0 ; i < nb_step ; i ++){
    int16_t tmpx = cx+x*cos((i+1)*a/nb_step)-y*sin((i+1)*a/nb_step);
    int16_t tmpy = cy+y*cos((i+1)*a/nb_step)+x*sin((i+1)*a/nb_step);
    float tmpa = ra+i*((a-M_PI/6)/nb_step);

    goto_xya_synced(tmpx,tmpy,tmpa);
  }
}

void _wait_meca_ready(void){
  //force meca state busy
  robot_state.meca_state = ROME_ENUM_MECA_STATE_BUSY;
  ROME_LOG(&rome_paddock,DEBUG,"strat : wait meca ready");
  for (;;){
    idle();
    if((robot_state.meca_state == ROME_ENUM_MECA_STATE_READY)){
      ROME_LOG(&rome_paddock,DEBUG,"strat : meca ready");
      return;
    }
  }
}

void _wait_meca_ground_clear(void){
  //force meca state busy
  robot_state.meca_state = ROME_ENUM_MECA_STATE_BUSY;
  ROME_LOG(&rome_paddock,DEBUG,"strat : wait meca ground clear");
  for (;;){
    idle();
    if((robot_state.meca_state == ROME_ENUM_MECA_STATE_GROUND_CLEAR ||
      robot_state.meca_state == ROME_ENUM_MECA_STATE_READY)){
      ROME_LOG(&rome_paddock,DEBUG,"strat : meca ground clear");
      return;
    }
  }
}

/// Do an autoset
void autoset(robot_side_t robot_side, autoset_side_t table_side, float x, float y)
{
  ROME_SENDWAIT_ASSERV_AUTOSET(&rome_asserv, robot_side, table_side, x, y);
  robot_state.asserv.autoset = 0;
  while(!robot_state.asserv.autoset) {
    //TODO avoid opponent
    idle();
  }
}

bool starting_cord_plugged(void)
{
  for(;;) {
    bool b = !portpin_in(&STARTING_CORD_PP);
    bool ok = true;
    for(uint16_t i=0; i<1000; i++) {
      if(b != !portpin_in(&STARTING_CORD_PP)) {
        ok = false;
        break;
      }
    }
    if(ok) {
      return b;
    }
  }
}

team_t strat_select_team(void)
{
  // wait for starting cord to be unplugged
  ROME_LOG(&rome_paddock,INFO,"Unplug starting cord ...");
  while(starting_cord_plugged()) {
    if((uptime_us() / 500000) % 2 == 0) {
      portpin_outset(&LED_B_PP);
    } else {
      portpin_outclr(&LED_B_PP);
    }
    idle();
  }

  // wait for color to be selected
  // color is selected when starting cord is plugged
  ROME_LOG(&rome_paddock,INFO,"Select color ...");
  team_t team = TEAM_NONE;
  portpin_outclr(&LED_B_PP);
  for(;;) {
    if(portpin_in(&COLOR_SELECTOR_PP)) {
      portpin_outclr(&LED_R_PP);
      portpin_outset(&LED_G_PP);
      portpin_outclr(&LED_B_PP);
      team = TEAM_GREEN;
    } else {
      portpin_outset(&LED_R_PP);
      portpin_outset(&LED_G_PP);
      portpin_outclr(&LED_B_PP);
      team = TEAM_ORANGE;
    }
    if(starting_cord_plugged()) {
      portpin_outclr(&LED_B_PP);
      break;
    }
    idle();
  }

  if(team == TEAM_GREEN)
    ROME_LOG(&rome_paddock,INFO,"Color : GREEN !");
  else
    ROME_LOG(&rome_paddock,INFO,"Color : ORANGE !");

  // wait 2s before next step
  uint32_t tend = uptime_us() + 2e6;
  while(uptime_us() < tend) {
    idle();
  }

  return team;
}

void strat_wait_start(void)
{

  while(starting_cord_plugged()) {
    if((uptime_us() / 500000) % 2 == 0) {
      if(robot_state.team == TEAM_ORANGE) {
        portpin_outset(&LED_R_PP);
        portpin_outset(&LED_G_PP);
        portpin_outclr(&LED_B_PP);
      } else {
        portpin_outclr(&LED_R_PP);
        portpin_outset(&LED_G_PP);
        portpin_outclr(&LED_B_PP);
      }
    } else {
      portpin_outclr(&LED_R_PP);
      portpin_outclr(&LED_G_PP);
      portpin_outset(&LED_B_PP);
    }
    idle();
  }

  portpin_outclr(&LED_R_PP);
  portpin_outclr(&LED_G_PP);
  portpin_outclr(&LED_B_PP);
  ROME_SENDWAIT_START_TIMER(&rome_asserv);
#if (defined GALIPEUR)
  ROME_SENDWAIT_START_TIMER(&rome_meca);
#endif
}

bool galipeur_cylinder_is_empty(void){
  return (robot_state.cylinder_nb_empty == robot_state.cylinder_nb_slots);
}

void strat_init_galipeur(void)
{
  ROME_LOG(&rome_paddock,INFO,"Galipeur : Strat init");
  // disable asserv
  ROME_SENDWAIT_ASSERV_ACTIVATE(&rome_asserv, 0);
  ROME_SENDWAIT_ASSERV_SET_HTRAJ_XY_STEERING(&rome_asserv, 2.5, 0.1);
  ROME_SENDWAIT_ASSERV_SET_HTRAJ_XY_CRUISE(&rome_asserv, 20, 0.1);

  // initialize meca
  ROME_LOG(&rome_paddock,INFO,"Init meca");
  ROME_SENDWAIT_MECA_SET_POWER(&rome_meca, 1);
  // set R3D2 parameters
  ROME_SENDWAIT_R3D2_SET_ROTATION(&rome_asserv,0,25);

  for(;;) {
    update_rome_interfaces();
    if(!robot_state.gyro_calibration)
      break;
  }

  // set R3D2 parameters
  ROME_SENDWAIT_R3D2_SET_ROTATION(&rome_asserv,200,25);
}

void strat_prepare_galipeur(void)
{
  //initalise kx factor
  robot_kx = robot_state.team == TEAM_GREEN ? -1 : 1;

  //send color to meca
  ROME_SENDWAIT_MECA_SET_ROBOT_COLOR(&rome_meca, robot_state.team == TEAM_GREEN);

  ROME_LOG(&rome_paddock,DEBUG,"Strat prepare");
  // initialize asserv
  ROME_SENDWAIT_ASSERV_ACTIVATE(&rome_asserv, 1);
  set_xya_wait(0,0,0);
  ROME_SENDWAIT_ASSERV_GOTO_XY(&rome_asserv, 0, 0, 0);
  ROME_SENDWAIT_ASSERV_SET_HTRAJ_XY_STEERING(&rome_asserv, 1.5, 0.03);
  ROME_SENDWAIT_ASSERV_SET_HTRAJ_XY_CRUISE(&rome_asserv, 15, 0.03);

  // autoset robot
  // x in starting area
  set_xya_wait(KX(0), 0, arfast(ROBOT_SIDE_BACK,TABLE_SIDE_MAIN));
  autoset(ROBOT_SIDE_BACK,AUTOSET_MAIN, KX(1500-AUTOSET_OFFSET), 0);
  goto_xya(KX(1000), 0, arfast(ROBOT_SIDE_BACK,TABLE_SIDE_MAIN));
  // y on building area
  goto_xya(KX(900), 300, arfast(ROBOT_SIDE_BACK,TABLE_SIDE_UP));
  autoset(ROBOT_SIDE_BACK,AUTOSET_UP, 0, 2000-AUTOSET_OFFSET);

  // check the state of the cylinder
  _wait_meca_ready();
  ROME_SENDWAIT_MECA_CMD(&rome_meca,ROME_ENUM_MECA_COMMAND_CHECK_EMPTY);

  //evade the border, leaving space for galipette
  goto_xya(KX(900), 1600, arfast(ROBOT_SIDE_BACK,TABLE_SIDE_UP));

  //go in front of starting area
  goto_xya(KX(1300), 1500, arfast(ROBOT_SIDE_BALLEATER,TABLE_SIDE_DOWN));

  //wait for meca to end checking cylinder
  _wait_meca_ready();
  if (!galipeur_cylinder_is_empty()){
    _wait_meca_ready();
    ROME_SENDWAIT_MECA_CMD(&rome_meca,ROME_ENUM_MECA_COMMAND_TRASH_BEGINMATCH);
  }

  //wait for meca to end all orders before shutting down asserv
  _wait_meca_ready();

  ROME_SENDWAIT_ASSERV_ACTIVATE(&rome_asserv, 0);
  ROME_SENDWAIT_ASSERV_GYRO_INTEGRATION(&rome_asserv, 0);

}

typedef enum{
  RS_VERYSLOW = 0,
  RS_SLOW,
  RS_NORMAL,
  RS_FAST,
} robot_speed_t;

void galipeur_set_speed(robot_speed_t s){
  switch (s){
    case RS_VERYSLOW:
      ROME_SENDWAIT_ASSERV_SET_HTRAJ_XY_STEERING(&rome_asserv, 1.5, 0.03);
      ROME_SENDWAIT_ASSERV_SET_HTRAJ_XY_CRUISE(&rome_asserv, 10, 0.03);
      break;
    case RS_SLOW:
      ROME_SENDWAIT_ASSERV_SET_HTRAJ_XY_STEERING(&rome_asserv, 1.5, 0.03);
      ROME_SENDWAIT_ASSERV_SET_HTRAJ_XY_CRUISE(&rome_asserv, 15, 0.03);
      break;
    case RS_NORMAL:
      ROME_SENDWAIT_ASSERV_SET_HTRAJ_XY_STEERING(&rome_asserv, 2.5, 0.1);
      ROME_SENDWAIT_ASSERV_SET_HTRAJ_XY_CRUISE(&rome_asserv, 20, 0.1);
      break;
    case RS_FAST:
      ROME_SENDWAIT_ASSERV_SET_HTRAJ_XY_STEERING(&rome_asserv, 3, 0.1);
      ROME_SENDWAIT_ASSERV_SET_HTRAJ_XY_CRUISE(&rome_asserv, 80, 0.05);
      break;
    default:
      break;
  }
}

float compute_throw_angle(int16_t x, int16_t y){
  float target = atan2(KX(1320)-KX(x),2200-y);
  float robot = arfast(ROBOT_SIDE_TURBINE,TABLE_SIDE_UP);
  //if (robot_state.team == TEAM_GREEN)
  //  return robot + target;
  //else
    return robot - target;

}

typedef enum{
  DISPENSER_NEAR = 0,
  DISPENSER_FAR,
} dispenser_t;

order_result_t galipeur_take_water(dispenser_t dispenser){
  order_result_t or;

  //save the amount of water we already have to check if we managed to "open" a dispenser
  uint8_t nb_empty = robot_state.cylinder_nb_empty;

  //dispenser positions
  int16_t near_pos = 2000-840;
  int16_t far_pos = -(1500-610);

  //balleater configuration
  int16_t balleater_depth = AUTOSET_OFFSET + 40;
  int16_t approach_depth = balleater_depth + 60;
  int16_t approach_side = 150;

  int16_t traj1[4];
  int16_t traj2[2];
  float angle;

  //prepare meca to take water
  _wait_meca_ready();
  ROME_SENDWAIT_MECA_CMD(&rome_meca,ROME_ENUM_MECA_COMMAND_PREPARE_LOAD_WATER);
  _wait_meca_ground_clear();

  galipeur_set_speed(RS_NORMAL);

  switch(dispenser){
    case DISPENSER_NEAR:{
      ROME_LOG(&rome_paddock,DEBUG,"Going to start area dispenser");
      angle = arfast(ROBOT_SIDE_BALLEATER, TABLE_SIDE_MAIN);
      //send first position order
      or = goto_xya(KX(1200), near_pos + approach_side, angle);
      if (or!= ORDER_SUCCESS)
        return or;
      //prepare next move orders
      //dispenser near is alongside Y axis
      traj1[0] = KX(1500-approach_depth);
      traj1[1] = near_pos + approach_side;
      traj1[2] = KX(1500-balleater_depth);
      traj1[3] = near_pos;

      traj2[0] = KX(1500-300);
      traj2[1] = near_pos;
      break;
    }
    case DISPENSER_FAR:{
      ROME_LOG(&rome_paddock,INFO,"Going to opposite dispenser");
      //send first position order
      int16_t traj[] = {
        KX(-far_pos),1000,
        KX(far_pos),1000,
        KX(-1200), 300};
      or = goto_traj(traj, arfast(ROBOT_SIDE_BACK, TABLE_SIDE_DOWN));
      if (or!= ORDER_SUCCESS){
        ROME_LOG(&rome_paddock,INFO,"Aborting opposite dispenser");
        idle_delay_ms(5000);
        goto_xya(KX(-far_pos), 1000, arfast(ROBOT_SIDE_BACK, TABLE_SIDE_DOWN));
        return or;
        }
      //we did a very long move, so launch an autoset
      //XXX on eirbot's table, we have a 3cm offset on the other side of the table...
      autoset(ROBOT_SIDE_BACK,AUTOSET_DOWN, 0, AUTOSET_OFFSET + 30);
      //XXX
      or = goto_xya(KX(-1200), 300, arfast(ROBOT_SIDE_BACK, TABLE_SIDE_DOWN));
      or = goto_xya(KX(-1200), 300, arfast(ROBOT_SIDE_BALLEATER, TABLE_SIDE_AUX));
      autoset(ROBOT_SIDE_BALLEATER,AUTOSET_AUX, KX(-1500+AUTOSET_OFFSET), 0);
      or = goto_xya(KX(-1200), 300, arfast(ROBOT_SIDE_BALLEATER, TABLE_SIDE_AUX));
      //prepare next move orders
      //dispenser far is alongside X axis
      angle = arfast(ROBOT_SIDE_BALLEATER, TABLE_SIDE_DOWN);
      traj1[0] = KX(far_pos-approach_side);
      traj1[1] = approach_depth;
      traj1[2] = KX(far_pos);
      traj1[3] = balleater_depth;

      traj2[0] = KX(far_pos);
      traj2[1] = 500;
      break;
    }
    default:
      return ORDER_FAILURE;
  }

  galipeur_set_speed(RS_SLOW);
  //go to dispenser
  or = goto_traj(traj1, angle);
  if (or!= ORDER_SUCCESS)
    return or;

  //take the water
  _wait_meca_ready();
  ROME_SENDWAIT_MECA_CMD(&rome_meca,ROME_ENUM_MECA_COMMAND_LOAD_WATER);
  _wait_meca_ground_clear();

  galipeur_set_speed(RS_NORMAL);
  //just go back a little bit
  or = goto_traj(traj2, angle);

  //if the water in cylinder changed, we scored !
  if (nb_empty != robot_state.cylinder_nb_empty)
    update_score(10);

  return or;
}

order_result_t galipeur_throw_water_watertower(void){
  ROME_LOG(&rome_paddock,INFO,"Throwing water in watertower");
  order_result_t or;
  galipeur_set_speed(RS_NORMAL);

  _wait_meca_ready();
  ROME_SENDWAIT_MECA_CMD(&rome_meca,ROME_ENUM_MECA_COMMAND_PREPARE_THROW_WATERTOWER);
  _wait_meca_ground_clear();

  uint8_t balls_loaded = robot_state.cylinder_nb_good;
  or = goto_xya(KX(1100), 1450, compute_throw_angle(1100,1450));
  ROME_SENDWAIT_MECA_SET_THROW_POWER(&rome_meca,1950);
  _wait_meca_ready();
  ROME_SENDWAIT_MECA_CMD(&rome_meca,ROME_ENUM_MECA_COMMAND_THROW_WATERTOWER);
  _wait_meca_ground_clear();
  update_score(5*balls_loaded);

  return or;
}


order_result_t galipeur_trash_water_treatment(void){
  ROME_LOG(&rome_paddock,INFO,"Trashing water in treatment area");
  // warning : order to call when we are near the "far" dispenser !
  order_result_t or;
  galipeur_set_speed(RS_NORMAL);

  _wait_meca_ready();
  ROME_SENDWAIT_MECA_CMD(&rome_meca,ROME_ENUM_MECA_COMMAND_PREPARE_TRASH_TREATMENT);
  _wait_meca_ground_clear();

  //store nb balls for scoring purpose
  uint8_t balls_loaded = robot_state.cylinder_nb_bad;

  //push away the cubes in front of treatment area
  int16_t traj[] = {
    KX(-700), 500,
    KX(-100), 500,
  };
  if (robot_state.team == TEAM_GREEN)
    or = goto_traj(traj, arfast(ROBOT_SIDE_BACK,TABLE_SIDE_MAIN));
  else
    or = goto_traj(traj, arfast(ROBOT_SIDE_BALLEATER,TABLE_SIDE_MAIN));
  //go in position to trash the bad water
  or = goto_xya(KX(-250),250+120, arfast(ROBOT_SIDE_TURBINE,TABLE_SIDE_DOWN));
  _wait_meca_ready();
  ROME_SENDWAIT_MECA_CMD(&rome_meca,ROME_ENUM_MECA_COMMAND_TRASH_TREATMENT);
  _wait_meca_ground_clear();
  update_score(10*balls_loaded);

  or = goto_xya(KX(-250), 500, arfast(ROBOT_SIDE_TURBINE, TABLE_SIDE_DOWN));
  or = goto_xya(KX(-250), 500, arfast(ROBOT_SIDE_BACK, TABLE_SIDE_DOWN));
  autoset(ROBOT_SIDE_BACK, AUTOSET_DOWN, 0, 250+AUTOSET_OFFSET);
  or = goto_xya(KX(-250), 1000, arfast(ROBOT_SIDE_BACK, TABLE_SIDE_DOWN));
  return or;
}

order_result_t galipeur_empty_cylinder(void){
  order_result_t or_throw_water = ORDER_FAILURE;
  order_result_t or_trash_water = ORDER_FAILURE;

  while(!galipeur_cylinder_is_empty()){
    if (robot_state.cylinder_nb_bad !=0){
      or_trash_water = galipeur_trash_water_treatment();
    }
    else
      if (robot_state.cylinder_nb_good !=0)
        or_throw_water = galipeur_throw_water_watertower();
    //wait to be sure that cylinder counts are updated
    idle_delay_ms(200);
  }

  (void) or_throw_water;
  (void) or_trash_water;

  return ORDER_SUCCESS;
}

void strat_run_galipeur(void)
{
  ROME_LOG(&rome_paddock,INFO,"Go !!!");
  ROME_SENDWAIT_ASSERV_GYRO_INTEGRATION(&rome_asserv, 1);
  ROME_SENDWAIT_ASSERV_ACTIVATE(&rome_asserv, 1);

  order_result_t or_take_water_near  = ORDER_FAILURE;
  order_result_t or_empty_water_near = ORDER_FAILURE;
  order_result_t or_take_water_far   = ORDER_FAILURE;
  order_result_t or_empty_water_far  = ORDER_FAILURE;

  bool force_far_dispenser_first = false;

  while (!( or_take_water_near  == ORDER_SUCCESS &&
            or_take_water_far   == ORDER_SUCCESS &&
            or_empty_water_near == ORDER_SUCCESS &&
            or_empty_water_far  == ORDER_SUCCESS )  ){

    if (force_far_dispenser_first &&
        or_take_water_far != ORDER_SUCCESS){

      or_take_water_far = galipeur_take_water(DISPENSER_FAR);

      or_empty_water_far = galipeur_empty_cylinder();
    }

    if (force_far_dispenser_first == true){
      goto_xya(KX(1200), 1500, arfast(ROBOT_SIDE_BACK,TABLE_SIDE_MAIN));
      autoset(ROBOT_SIDE_BACK,AUTOSET_MAIN, KX(1500-AUTOSET_OFFSET), 0);
      goto_xya(KX(1200), 1500, arfast(ROBOT_SIDE_BACK,TABLE_SIDE_MAIN));
    }
    else
      force_far_dispenser_first = true;

    if (or_take_water_near != ORDER_SUCCESS){
      or_take_water_near = galipeur_take_water(DISPENSER_NEAR);

      or_empty_water_far = galipeur_empty_cylinder();
    }

    idle_delay_ms(200);
  }

  ROME_LOG(&rome_paddock,INFO,"That's all folks !");
  _delay_ms(3000);
  ROME_SENDWAIT_ASSERV_ACTIVATE(&rome_asserv, 0);
  ROME_SENDWAIT_MECA_SET_POWER(&rome_meca, 0);
}

void strat_test_galipeur(void)
{
#if 0
  ROME_LOG(&rome_paddock,INFO,"Strat test stuff");
  for(;;) {
    update_rome_interfaces();
    if(!robot_state.gyro_calibration)
      break;
  }

  ROME_SENDWAIT_ASSERV_ACTIVATE(&rome_asserv, 1);
  ROME_SENDWAIT_ASSERV_GYRO_INTEGRATION(&rome_asserv, 1);
  autoset(ROBOT_SIDE_BACK, AUTOSET_DOWN, 0, AUTOSET_OFFSET);
  goto_xya_rel(0,250,0.0);
  autoset(ROBOT_SIDE_BACK, AUTOSET_RIGHT, 0, AUTOSET_OFFSET);
  goto_xya(-250,250,arfast(ROBOT_SIDE_BACK,TABLE_SIDE_RIGHT));

  galipeur_set_speed(RS_NORMAL);

  uint8_t i = 0;
  for(;;){
    goto_xya(-250,1000,0.0);
    goto_xya(-250,200,0.0);
    i++;
    if (i > 5){
      autoset(ROBOT_SIDE_BACK, AUTOSET_DOWN, 0, AUTOSET_OFFSET);
      goto_xya(-250,250,0.0);
      autoset(ROBOT_SIDE_BACK, AUTOSET_RIGHT, 0, AUTOSET_OFFSET);
      goto_xya(-250,250,arfast(ROBOT_SIDE_BACK,TABLE_SIDE_RIGHT));
      i = 0;
    }
    else
      idle_delay_ms(2000);
  }
#endif
}

/****************************************************************************/

// enum for galipette servo number assignation
typedef enum{
  NONE = 0,
  CUBE_CLAW_LEFT = 1,
  CUBE_CLAW_ELEVATOR = 2,
  CUBE_CLAW_RIGHT = 3,
} cube_claw_servo_t;

#define CUBE_CLAW_LEFT_START 1500
#define CUBE_CLAW_LEFT_CLOSED 2100
#define CUBE_CLAW_LEFT_OPENED 3000

#define CUBE_CLAW_RIGHT_START 3000
#define CUBE_CLAW_RIGHT_CLOSED 2400
#define CUBE_CLAW_RIGHT_OPENED 1500

#define CUBE_CLAW_ELEVATOR_DOWN 1300
#define CUBE_CLAW_ELEVATOR_BUTTON 2600
#define CUBE_CLAW_ELEVATOR_BUTTON_WITHCUBE 2800
#define CUBE_CLAW_ELEVATOR_MID 3250
#define CUBE_CLAW_ELEVATOR_UP 4000

#define GALIPETTE_BUMPER_TO_CENTER_DIST 100 // distance from the edge of galipette to the center of the robot (in mm)

void galipette_set_speed(robot_speed_t s){
  switch (s){
    case RS_SLOW:
      ROME_SENDWAIT_ASSERV_SET_HTRAJ_XY_STEERING(&rome_asserv, 1.5, 0.03);
      ROME_SENDWAIT_ASSERV_SET_HTRAJ_XY_CRUISE(&rome_asserv, 15, 0.03);
      ROME_SENDWAIT_ASSERV_SET_HTRAJ_XY_STOP(&rome_asserv, 0.5, 0.05);
      break;
    case RS_NORMAL:
      ROME_SENDWAIT_ASSERV_SET_HTRAJ_XY_STEERING(&rome_asserv, 2.5, 0.05);
      ROME_SENDWAIT_ASSERV_SET_HTRAJ_XY_CRUISE(&rome_asserv, 20, 0.05);
      ROME_SENDWAIT_ASSERV_SET_HTRAJ_XY_STOP(&rome_asserv, 0.5, 0.05);
      break;
    case RS_FAST:
      ROME_SENDWAIT_ASSERV_SET_HTRAJ_XY_STEERING(&rome_asserv, 3, 0.05);
      ROME_SENDWAIT_ASSERV_SET_HTRAJ_XY_CRUISE(&rome_asserv, 80, 0.05);
      ROME_SENDWAIT_ASSERV_SET_HTRAJ_XY_STOP(&rome_asserv, 0.5, 0.05);
      break;
    default:
      break;
  }
}

void galipette_cube_claw_start(void){
  ROME_SENDWAIT_ASSERV_SET_SERVO(&rome_asserv, CUBE_CLAW_LEFT, CUBE_CLAW_LEFT_START);
  ROME_SENDWAIT_ASSERV_SET_SERVO(&rome_asserv, CUBE_CLAW_RIGHT, CUBE_CLAW_RIGHT_START);
}

void galipette_cube_claw_close(void){
  ROME_SENDWAIT_ASSERV_SET_SERVO(&rome_asserv, CUBE_CLAW_LEFT, CUBE_CLAW_LEFT_CLOSED);
  ROME_SENDWAIT_ASSERV_SET_SERVO(&rome_asserv, CUBE_CLAW_RIGHT, CUBE_CLAW_RIGHT_CLOSED);
}

void galipette_cube_claw_open(void){
  ROME_SENDWAIT_ASSERV_SET_SERVO(&rome_asserv, CUBE_CLAW_LEFT, CUBE_CLAW_LEFT_OPENED);
  ROME_SENDWAIT_ASSERV_SET_SERVO(&rome_asserv, CUBE_CLAW_RIGHT, CUBE_CLAW_RIGHT_OPENED);
}

void strat_init_galipette(void)
{
  ROME_LOG(&rome_paddock,INFO,"Galipette : Strat init");
  // disable asserv
  ROME_SENDWAIT_ASSERV_ACTIVATE(&rome_asserv, 0);

  ROME_SENDWAIT_ASSERV_SET_SERVO(&rome_asserv, CUBE_CLAW_ELEVATOR, CUBE_CLAW_ELEVATOR_BUTTON);
  galipette_cube_claw_start();

  // set R3D2 parameters
  ROME_SENDWAIT_R3D2_SET_ROTATION(&rome_asserv,0,25);
  for(;;) {
    idle();
    if(!robot_state.gyro_calibration)
      break;
  }

  // set R3D2 parameters
  ROME_SENDWAIT_R3D2_SET_ROTATION(&rome_asserv,350,25);

}

void strat_prepare_galipette(void)
{
  galipette_set_speed(RS_NORMAL);
  //initalise kx factor
  robot_kx = robot_state.team == TEAM_GREEN ? -1 : 1;

  ROME_LOG(&rome_paddock,DEBUG,"Strat prepare");

  ROME_SENDWAIT_ASSERV_ACTIVATE(&rome_asserv, 1);

  set_xya_wait(KX(0), 0, arfast(ROBOT_SIDE_BACK,TABLE_SIDE_MAIN));
  autoset(ROBOT_SIDE_BACK,AUTOSET_MAIN, KX(1500-GALIPETTE_BUMPER_TO_CENTER_DIST), 0);
  goto_xya(KX(1200), 0, arfast(ROBOT_SIDE_BACK,TABLE_SIDE_MAIN));
  goto_xya(KX(1200), 0, arfast(ROBOT_SIDE_BACK,TABLE_SIDE_UP));
  autoset(ROBOT_SIDE_BACK,AUTOSET_UP, 0, 2000-GALIPETTE_BUMPER_TO_CENTER_DIST);
  goto_xya(KX(1200), 1800, arfast(ROBOT_SIDE_BACK,TABLE_SIDE_UP));
  goto_xya(KX(1350), 1800, arfast(ROBOT_SIDE_CUBE_CLAW,TABLE_SIDE_AUX));

  ROME_SENDWAIT_ASSERV_SET_SERVO(&rome_asserv, CUBE_CLAW_ELEVATOR, CUBE_CLAW_ELEVATOR_DOWN);
  galipette_cube_claw_open();

  ROME_SENDWAIT_ASSERV_ACTIVATE(&rome_asserv, 0);
  ROME_SENDWAIT_ASSERV_GYRO_INTEGRATION(&rome_asserv, 0);
}

void strat_run_galipette(void)
{
  ROME_SENDWAIT_ASSERV_GYRO_INTEGRATION(&rome_asserv, 1);
  ROME_SENDWAIT_ASSERV_ACTIVATE(&rome_asserv, 1);
  galipette_cube_claw_close();

  //wait for galipeur departure
  idle_delay_ms(1000);

  //go switching domotic panel first
  ROME_SENDWAIT_ASSERV_SET_SERVO(&rome_asserv, CUBE_CLAW_ELEVATOR, CUBE_CLAW_ELEVATOR_BUTTON_WITHCUBE);

  int16_t traj[] = {
      KX(1000), 1700,
      KX(370), 1700,
      KX(370), 1820,
      };
  goto_traj(traj, arfast(ROBOT_SIDE_CUBE_CLAW,TABLE_SIDE_UP));
  idle_delay_ms(2000);
  galipette_cube_claw_open();
  goto_xya(KX(370), 1460, arfast(ROBOT_SIDE_CUBE_CLAW,TABLE_SIDE_MAIN));
  ROME_SENDWAIT_ASSERV_SET_SERVO(&rome_asserv, CUBE_CLAW_ELEVATOR, CUBE_CLAW_ELEVATOR_UP);

}

void strat_test_galipette(void)
{
#if 1
  ROME_LOG(&rome_paddock,INFO,"Strat test stuff");
  for(;;) {
    update_rome_interfaces();
    if(!robot_state.gyro_calibration)
      break;
  }

  galipette_set_speed(RS_NORMAL);
  ROME_SENDWAIT_ASSERV_ACTIVATE(&rome_asserv, 1);
  ROME_SENDWAIT_ASSERV_GYRO_INTEGRATION(&rome_asserv, 1);

  for(;;){
    goto_xya(500,-500,0);
    idle_delay_ms(10000);
    goto_xya(0,0,0);
    idle_delay_ms(10000);
  }
#endif
}



#if (defined GALIPEUR)
# define ROBOT_SUFFIX  galipeur
#elif (defined GALIPETTE)
# define ROBOT_SUFFIX  galipette
#else
# error Either GALIPEUR or GALIPETTE must be defined
#endif

#define CONCAT__(a,b)  a ## b
#define CONCAT_(a,b)  CONCAT__(a,b)
#define ROBOT_FUNCTION(f)  CONCAT_(f,ROBOT_SUFFIX)


void strat_init(void)
{
  ROBOT_FUNCTION(strat_init_)();
}

void strat_prepare(void)
{
  ROBOT_FUNCTION(strat_prepare_)();
}

void strat_run(void)
{
  ROBOT_FUNCTION(strat_run_)();
}

void strat_test(void)
{
  ROBOT_FUNCTION(strat_test_)();
}

