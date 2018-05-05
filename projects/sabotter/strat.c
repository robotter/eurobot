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
#include <pathfinding/pathfinding.h>

#define ANGLE_TYPE__ float
#include "modulo.inc.h"
#undef ANGLE_TYPE__

#define GOTO_TIMEOUT_MS  10000
#define DEFAULT_WAIT_MS  2000

extern rome_intf_t rome_asserv;
extern rome_intf_t rome_meca;
extern rome_intf_t rome_paddock;

extern pathfinding_t pathfinder;

void rome_send_pathfinding_path(const pathfinding_t *finder)
{
  ROME_SEND_PATHFINDING_PATH(&rome_paddock, finder->path, finder->path_size);
}

robot_state_t robot_state;
// we don't put 'kx' on robot_state to allow it to be static
// this avoids optimization errors
static float robot_kx;

typedef enum{
  RS_VERYSLOW = 0,
  RS_SLOW,
  RS_NORMAL,
  RS_FAST,
} robot_speed_t;

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
#define AUTOSET_MAIN (robot_state.team == TEAM_GREEN ? AUTOSET_LEFT : AUTOSET_RIGHT)
#define AUTOSET_AUX  (robot_state.team == TEAM_GREEN ? AUTOSET_RIGHT : AUTOSET_LEFT)
#define KX(x) (robot_kx*(x))

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
order_result_t goto_traj_n_wait(int16_t* xy, uint8_t n, float a, uint16_t timeout_ms)
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
      detection_path_t dp = opponent_detected_carrot(xy[2*robot_state.asserv.path_i],xy[2*robot_state.asserv.path_i+1]);
      if(dp == DETECTION_PATH_BLOCKED) {
        return ORDER_DETECTION;
      }
      if(dp == DETECTION_PATH_CLEARED) {
        //resend order
        path_i = robot_state.asserv.path_i;
        break;
      }
      idle();
    }
  }
}

///Execute pathfinding type trajectory
order_result_t goto_pathfinding_node(uint8_t goal, float angle){
  for(;;){
    //update obstacles
    //first is our other robot, the others are the detected robots
    pathfinding_obstacle_t obstacles[R3D2_OBJECTS_MAX+1];

    //TODO : add galipette.eur position to obstacles
    obstacles[0].x = 0;
    obstacles[0].y = 0;
    obstacles[0].r = 0;

    //compute absolute position of other robots
    for(uint8_t i=0; i<R3D2_OBJECTS_MAX; i++) {
      r3d2_object_t *object = &robot_state.r3d2.objects[i];
      if (object->detected) {
        float da = float_modulo__(object->a+robot_state.current_pos.a+R3D2_OFFSET, 0, 2*M_PI);
        obstacles[i+1].x = robot_state.current_pos.x + object->r*cos(da);
        obstacles[i+1].y = robot_state.current_pos.y + object->r*sin(da);
        obstacles[i+1].r = 300;
        ROME_LOGF(&rome_paddock,DEBUG,"obstacle ra %f %f",object->r,da);
        ROME_LOGF(&rome_paddock,DEBUG,"obstacle xy %d %d",obstacles[i].x,obstacles[i].y);
      }
      else {
        obstacles[i+1].x = 0;
        obstacles[i+1].y = 0;
        obstacles[i+1].r = 0;
      }
    }
    pathfinder.obstacles = obstacles;
    pathfinder.obstacles_size = sizeof(obstacles)/sizeof(*obstacles);

    //search for a path
    uint8_t start = pathfinding_nearest_node(&pathfinder,
                                  robot_state.current_pos.x,
                                  robot_state.current_pos.y);
    pathfinding_search(&pathfinder, start, goal);
    ROME_LOGF(&rome_paddock, INFO, "path found (%u)", pathfinder.path_size);

    //send the trajectory to asserv, but do not go to nearest node
    int16_t traj[pathfinder.path_size * 2];
    for(int i=1; i < pathfinder.path_size; i++){
      traj[2*i] = pathfinder.nodes[pathfinder.path[i]].x;
      traj[2*i+1] = pathfinder.nodes[pathfinder.path[i]].y;
    }

    goto_traj(traj, angle);

  }
  return ORDER_SUCCESS;
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

//include robot's strat files
#if (defined GALIPEUR)
# include "strat_galipeur.inc.c"
#elif (defined GALIPETTE)
# include "strat_galipette.inc.c"
#else
# error Either GALIPEUR or GALIPETTE must be defined
#endif
