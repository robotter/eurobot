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
#define DEFAULT_FOV  (M_PI/3)

#define AUTOSET_OFFSET 114

extern rome_intf_t rome_asserv;
extern rome_intf_t rome_meca;
extern rome_intf_t rome_paddock;

robot_state_t robot_state;
// we don't put 'kx' on robot_state to allow it to be static
// this avoids optimization errors
static int16_t robot_kx;


typedef enum {
  ROBOT_SIDE_LEFT = 0,
  ROBOT_SIDE_RIGHT,
  ROBOT_SIDE_BACK,
} robot_side_t;

#define ROBOT_SIDE_MAIN (robot_state.team == TEAM_GREEN ? ROBOT_SIDE_RIGHT : ROBOT_SIDE_LEFT)
#define ROBOT_SIDE_AUX  (robot_state.team == TEAM_GREEN ? ROBOT_SIDE_LEFT : ROBOT_SIDE_RIGHT)
#define MECA_MAIN (robot_state.team == TEAM_GREEN ? MECA_RIGHT : MECA_LEFT)
#define MECA_AUX  (robot_state.team == TEAM_GREEN ? MECA_LEFT : MECA_RIGHT)
#define AUTOSET_MAIN (robot_state.team == TEAM_GREEN ? AUTOSET_RIGHT : AUTOSET_LEFT)
#define AUTOSET_AUX  (robot_state.team == TEAM_GREEN ? AUTOSET_LEFT : AUTOSET_RIGHT)
#define KX(x) (robot_kx*(x))
#define KC(green,purple) (robot_state.team == TEAM_GREEN ? (green) : (purple))
#define KA(a) (robot_kx*(a))

typedef enum {
  MECA_LEFT = 0,
  MECA_RIGHT = 1
}spot_elevator_t;

typedef enum {
  SPOT_ELEV_S_BUSY = 0,
  SPOT_ELEV_S_GROUND_CLEAR,
  SPOT_ELEV_S_READY,
}spot_elevator_state_t;

typedef enum {
  ORDER_SUCCESS = 0,
  ORDER_DETECTION,
  ORDER_TIMEOUT,
  ORDER_FAILURE,
} order_result_t;

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
#define DEFAULT_DETECTION_FOV 2*M_PI/3
#define R3D2_OFFSET (0)

// Return true if an opponent is detected within an arc
bool opponent_detected_in_arc(float angle, float fov)
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
    if(object->detected && object->r < R3D2_AVOID_DISTANCE && in_range) {
      return true;
    }
  }
  return false;
}

// Same as opponent_detected_arc(), based on carrot and using a default fov
bool opponent_detected_carrot(void)
{
  int16_t dx = robot_state.carrot.x - robot_state.current_pos.x;
  int16_t dy = robot_state.carrot.y - robot_state.current_pos.y;
  float angle = atan2(dy,dx);
  return opponent_detected_in_arc(angle, DEFAULT_FOV);
}

// Wait while opponent is detected
// Return true if opponent is not detected anymore, false on timeout
bool wait_detected_opponent(uint16_t ms)
{
  //uint32_t tend = uptime_us() + ms * 1000;
  while(opponent_detected_carrot()) {
    //if(uptime_us() >= tend) {
    //  return false;
    //}
    idle();
  }
  return true;
}


void ext_arm_set(int16_t pos)
{
  ROME_SENDWAIT_MECA_SET_ARM(&rome_meca, pos);
}

/// Set arm position
void ext_arm_raise(void) { ext_arm_set(430); }
void ext_arm_lower(void) { ext_arm_set(100); }
void ext_arm_clap(void)  { ext_arm_set(300); }
void ext_arm_galette_prepare(void)  { ext_arm_set(270); }
void ext_arm_galette_lift(void)  { ext_arm_set(460); }
void ext_arm_galette_release(void)  { ext_arm_set(350); }

void _wait_meca_ready(void){
  ROME_LOG(&rome_paddock,DEBUG,"strat : wait meca ready");
  for (;;){
    if((robot_state.left_elev.state == SPOT_ELEV_S_READY)&&
       (robot_state.right_elev.state == SPOT_ELEV_S_READY)){
      ROME_LOG(&rome_paddock,DEBUG,"strat : meca ready");
      return;
    }
    idle();
  }
}

void _wait_meca_ground_clear(void){
  ROME_LOG(&rome_paddock,DEBUG,"strat : wait meca ground clear");
  for (;;){
    if((robot_state.left_elev.state != SPOT_ELEV_S_BUSY)&&
       (robot_state.right_elev.state != SPOT_ELEV_S_BUSY)){
      ROME_LOG(&rome_paddock,DEBUG,"strat : meca ground clear");
      return;}
    idle();
  }
}

void _meca_discharge_spots(void){
  _wait_meca_ready();
  if(robot_state.left_elev.nb_spots > 0){
    ROME_SENDWAIT_MECA_CMD(&rome_meca, ROME_ENUM_MECA_COMMAND_DISCHARGE_SPOT_STACK, MECA_LEFT);
    robot_state.left_elev.state = SPOT_ELEV_S_BUSY;
  }

  if(robot_state.right_elev.nb_spots > 0){
    ROME_SENDWAIT_MECA_CMD(&rome_meca, ROME_ENUM_MECA_COMMAND_DISCHARGE_SPOT_STACK, MECA_RIGHT);
    robot_state.right_elev.state = SPOT_ELEV_S_BUSY;
  }
  _wait_meca_ground_clear();
}

void _meca_release_spots(void){
  _wait_meca_ready();
  if(robot_state.left_elev.nb_spots > 0){
    ROME_SENDWAIT_MECA_CMD(&rome_meca, ROME_ENUM_MECA_COMMAND_RELEASE_SPOT_STACK, MECA_LEFT);
    robot_state.left_elev.state = SPOT_ELEV_S_BUSY;
  }

  if(robot_state.right_elev.nb_spots > 0){
    ROME_SENDWAIT_MECA_CMD(&rome_meca, ROME_ENUM_MECA_COMMAND_RELEASE_SPOT_STACK, MECA_RIGHT);
    robot_state.right_elev.state = SPOT_ELEV_S_BUSY;
  }
  _wait_meca_ground_clear();
}

static void _meca_order_blocking_left_right(uint8_t cmd_left, uint8_t cmd_right){
  _wait_meca_ready();
  //send orders
  if (cmd_left){
    ROME_SENDWAIT_MECA_CMD(&rome_meca, cmd_left, MECA_LEFT);
    robot_state.left_elev.state = SPOT_ELEV_S_BUSY;
  }
  if (cmd_right){
    ROME_SENDWAIT_MECA_CMD(&rome_meca, cmd_right, MECA_RIGHT);
    robot_state.right_elev.state = SPOT_ELEV_S_BUSY;
  }
  //wait for meca to compute orders...
  _wait_meca_ground_clear();
}

static void _meca_order_blocking_main_aux(uint8_t cmd_main, uint8_t cmd_aux){
  if(robot_state.team == TEAM_GREEN)
    _meca_order_blocking_left_right(cmd_aux,cmd_main);
  else if(robot_state.team == TEAM_PURPLE)
    _meca_order_blocking_left_right(cmd_main,cmd_aux);
}

static void set_xya_wait(double x, double y, double a) {
  ROME_SENDWAIT_ASSERV_SET_XYA(&rome_asserv, x, y, 1000.0*a);
}

#define _meca_order_blocking_ma(m,a) _meca_order_blocking_main_aux(ROME_ENUM_MECA_COMMAND_##m,ROME_ENUM_MECA_COMMAND_##a)
#define _meca_order_blocking_both(cmd) _meca_order_blocking_left_right(ROME_ENUM_MECA_COMMAND_##cmd,ROME_ENUM_MECA_COMMAND_##cmd)

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
        ROME_LOGF(&rome_paddock,INFO,"goto_traj : timeout %ld > %ld", time, (uint32_t)timeout_ms*1000);
        return ORDER_TIMEOUT;
        }
      if(robot_state.asserv.xy && robot_state.asserv.a) {
        return ORDER_SUCCESS;
      }
      if(opponent_detected_carrot()) {
        ROME_LOG(&rome_paddock,INFO,"goto_xya : opponent detected");
        ROME_SENDWAIT_ASSERV_GOTO_XY_REL(&rome_asserv, 0, 0, 0);
        ROME_SENDWAIT_ASSERV_ACTIVATE(&rome_asserv, 0);
        for(;;){
          start_time_us = uptime_us();
          if(!opponent_detected_carrot())
            break;
          idle();
        }
        ROME_SENDWAIT_ASSERV_ACTIVATE(&rome_asserv, 1);
        break; // to resend goto order
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
        ROME_LOGF(&rome_paddock,INFO,"goto_traj : timeout %ld > %ld", time, (uint32_t)timeout_ms*1000);
        return ORDER_TIMEOUT;
        }
      if(robot_state.asserv.xy && robot_state.asserv.a) {
        return ORDER_SUCCESS;
      }
      if(opponent_detected_carrot()) {
        ROME_LOG(&rome_paddock,INFO,"goto_xya_synced : opponent detected");
        ROME_SENDWAIT_ASSERV_GOTO_XY_REL(&rome_asserv, 0, 0, 0);
        ROME_SENDWAIT_ASSERV_ACTIVATE(&rome_asserv, 0);
        for(;;){
          start_time_us = uptime_us();
          if(!opponent_detected_carrot())
            break;
          idle();
        }
        ROME_SENDWAIT_ASSERV_ACTIVATE(&rome_asserv, 1);
        break; // to resend goto order
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
    ROME_SENDWAIT_ASSERV_RUN_TRAJ(&rome_asserv,1000*a,xy+path_i*2,n);
    robot_state.asserv.xy = 0;
    robot_state.asserv.a = 0;
    robot_state.asserv.path_i = path_i;
    robot_state.asserv.path_n = n/2;
    for(;;) {
      uint32_t time = uptime_us() - start_time_us;
      if((time) > ((uint32_t) timeout_ms*1000)){
        ROME_LOGF(&rome_paddock,INFO,"goto_traj : timeout %ld > %ld", time, (uint32_t)timeout_ms*1000);
        return ORDER_TIMEOUT;
        }
      if(robot_state.asserv.xy && robot_state.asserv.a) {
        return ORDER_SUCCESS;
      }
      if(opponent_detected_carrot()) {
        ROME_LOG(&rome_paddock,INFO,"goto_traj : opponent detected");
        path_i = robot_state.asserv.path_i;
        ROME_SENDWAIT_ASSERV_GOTO_XY_REL(&rome_asserv, 0, 0, 0);
        ROME_SENDWAIT_ASSERV_ACTIVATE(&rome_asserv, 0);
        for(;;){
          start_time_us = uptime_us();
          if(!opponent_detected_carrot())
            break;
          idle();
        }
        ROME_SENDWAIT_ASSERV_ACTIVATE(&rome_asserv, 1);
        break; // to resend goto order
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
      if(opponent_detected_carrot()) {
        ROME_LOG(&rome_paddock,INFO,"goto_xya_panning : opponent detected");
        ROME_SENDWAIT_ASSERV_GOTO_XY_REL(&rome_asserv, 0, 0, 0);
        ROME_SENDWAIT_ASSERV_ACTIVATE(&rome_asserv, 0);
        while(opponent_detected_carrot()) {
          idle();
        }
        ROME_SENDWAIT_ASSERV_ACTIVATE(&rome_asserv, 1);
        break; // to resend goto order
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
      if(opponent_detected_carrot()) {
        ROME_LOG(&rome_paddock,INFO,"goto_xya_rel : opponent detected");
        ROME_SENDWAIT_ASSERV_GOTO_XY_REL(&rome_asserv, 0, 0, 0);
        ROME_SENDWAIT_ASSERV_ACTIVATE(&rome_asserv, 0);
        for(;;){
          start_time_us = uptime_us();
          if(!opponent_detected_carrot())
            break;
          idle();
        }
        ROME_SENDWAIT_ASSERV_ACTIVATE(&rome_asserv, 1);
        break; // to resend goto order
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


#define CLAW_X 70
#define CLAW_Y -200
#define CLAW_APPROACH 50
#define CLAW_PUSH_SPOT -100

order_result_t go_pick_spot_wait(int16_t x, int16_t y, spot_elevator_t side, uint16_t timeout_ms){
  ROME_LOGF(&rome_paddock,DEBUG,"spot :%d,%d",x,y);

  int16_t dx; 
  if ((side == MECA_RIGHT && robot_state.team == TEAM_GREEN)
    ||(side == MECA_LEFT && robot_state.team == TEAM_PURPLE)){
    _meca_order_blocking_ma(PREPARE_PICK_SPOT,NONE);
    dx = KX(CLAW_X);
    }
  else{
    _meca_order_blocking_ma(NONE,PREPARE_PICK_SPOT);
    dx = -KX(CLAW_X);
    }
  int16_t rx = robot_state.current_pos.x ;
  int16_t ry = robot_state.current_pos.y ;
  int16_t rs_x = x - rx;
  int16_t rs_y = y - ry;
 
  double beta = atan2(rs_y,rs_x);

  double alpha = beta - 3*M_PI/2;

  // spot position in robot frame
  int16_t dy = (CLAW_Y - CLAW_APPROACH);
  ROME_LOGF(&rome_paddock,DEBUG,"dxdy :%d,%d",dx,dy);
  // spot position in table frame
  int16_t tdx = dx*cos(alpha) - dy*sin(alpha);
  int16_t tdy = dx*sin(alpha) + dy*cos(alpha);
  goto_xya_wait(rx,ry,beta+M_PI/2,timeout_ms);
  goto_xya_wait(x-tdx,y-tdy,beta+M_PI/2,timeout_ms);

  // spot position in robot frame
  dy = CLAW_Y - CLAW_PUSH_SPOT;
  ROME_LOGF(&rome_paddock,DEBUG,"dxdy :%d,%d",dx,dy);
  // spot position in table frame
  tdx = dx*cos(alpha) - dy*sin(alpha);
  tdy = dx*sin(alpha) + dy*cos(alpha);
  _wait_meca_ready();
  goto_xya_wait(x-tdx,y-tdy,beta+M_PI/2,timeout_ms);
  if ((side == MECA_RIGHT && robot_state.team == TEAM_GREEN)
    ||(side == MECA_LEFT && robot_state.team == TEAM_PURPLE)){
    _meca_order_blocking_ma(PICK_SPOT,NONE);
    }
  else{
    _meca_order_blocking_ma(NONE,PICK_SPOT);
    }
  return ORDER_SUCCESS;
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

/// delay for some ms
void strat_delay_ms(uint32_t ms) {
  uint32_t tend = uptime_us() + 1000*ms;
  for(;;) {
    if(uptime_us() >= tend) {
      return;
    }
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
      portpin_outclr(&LED_G_PP);
      portpin_outset(&LED_B_PP);
      team = TEAM_PURPLE;
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
    ROME_LOG(&rome_paddock,INFO,"Color : PURPLE !");
    
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
      if(robot_state.team == TEAM_PURPLE) {
        portpin_outset(&LED_R_PP);
        portpin_outclr(&LED_G_PP);
        portpin_outset(&LED_B_PP);
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
  //ROME_SENDWAIT_R3D2_SET_ROTATION(&rome_asserv,0,25);

  for(;;) {
    update_rome_interfaces();
    if(!robot_state.gyro_calibration)
      break;
  }
  
  strat_delay_ms(5000);
  ROME_SENDWAIT_MECA_SET_SAND_ROLLER(&rome_meca, 0);

  // set R3D2 parameters
  //ROME_SENDWAIT_R3D2_SET_ROTATION(&rome_asserv,350,25);
}

void strat_prepare_galipeur(void)
{
  //initalise kx factor
  robot_kx = robot_state.team == TEAM_GREEN ? 1 : -1;
  
  ROME_LOG(&rome_paddock,DEBUG,"Strat prepare");
  // initialize asserv
  ROME_SENDWAIT_ASSERV_ACTIVATE(&rome_asserv, 1);
  set_xya_wait(0,0,0);
  ROME_SENDWAIT_ASSERV_GOTO_XY(&rome_asserv, 0, 0, 0);
  ROME_SENDWAIT_ASSERV_SET_HTRAJ_XY_STEERING(&rome_asserv, 1.5, 0.03);
  ROME_SENDWAIT_ASSERV_SET_HTRAJ_XY_CRUISE(&rome_asserv, 15, 0.03);

  // autoset robot, Y on pond
  autoset(ROBOT_SIDE_BACK,AUTOSET_DOWN, 0, AUTOSET_OFFSET);
  // move in front of starting area
  goto_xya(KX(200), 1210, KA(0));
  //and autoset X
  autoset(ROBOT_SIDE_BACK,AUTOSET_MAIN, KX(1500-AUTOSET_OFFSET), 0);

  ROME_SENDWAIT_ASSERV_ACTIVATE(&rome_asserv, 0);
  ROME_SENDWAIT_ASSERV_GYRO_INTEGRATION(&rome_asserv, 0);
}

order_result_t galipeur_close_doors(void){
  //go in front of first door
  goto_xya(KX(1250), 1750, KA(M_PI));
  //push it
  goto_xya(KX(1200), 1850, KA(M_PI));
  //go in front of 2nd door
  goto_xya(KX(1100), 1750, KA(M_PI));
  //push it
  goto_xya(KX(1050), 1850, KA(M_PI));
  //evade the doors
  goto_xya(KX(1100), 1700, KA(M_PI));
  return ORDER_SUCCESS;
}

typedef enum{
  RS_SLOW = 0,
  RS_NORMAL,
  RS_FAST,
} robot_speed_t;

void galipeur_set_speed(robot_speed_t s){
  switch (s){
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
      ROME_SENDWAIT_ASSERV_SET_HTRAJ_XY_CRUISE(&rome_asserv, 80, 0.1);
      break;
    default:
      break;
  }
}

void galipeur_reset_sandroller(void){
  ROME_SENDWAIT_MECA_SET_SAND_ROLLER(&rome_meca, 0);
  strat_delay_ms(500);
  ROME_SENDWAIT_MECA_SET_SAND_ROLLER(&rome_meca, 1);
}

#define DESTROY_DUNE_END_POSITION_X KX(950)
#define DESTROY_DUNE_END_POSITION_Y 1400

order_result_t galipeur_destroy_dune_first_row(void){
  order_result_t or;
  //we must be the first to get there !
  galipeur_set_speed(RS_FAST);
  int16_t traj1[] = {
    DESTROY_DUNE_END_POSITION_X,DESTROY_DUNE_END_POSITION_Y,
    KX(-300), 1600,
  };
  //first traj must be a success, if not leave the area
  or = goto_traj(traj1,KA(M_PI/2));
  if (or != ORDER_SUCCESS){
    //go to destroy end position
    galipeur_set_speed(RS_NORMAL);
    goto_xya_synced(DESTROY_DUNE_END_POSITION_X,
                    DESTROY_DUNE_END_POSITION_Y,
                    KA(M_PI/2));
    return ORDER_FAILURE;
  }
  //slow down to push sand
  galipeur_set_speed(RS_SLOW);
  //destroy 1st row
  goto_xya(KX(-300), 2000-240, KA(M_PI));
  ROME_SENDWAIT_MECA_SET_SAND_ROLLER(&rome_meca, 1);
  goto_xya(KX(100),  2000-240, KA(M_PI));
  ROME_SENDWAIT_MECA_SET_SAND_ROLLER(&rome_meca, 0);
  return ORDER_SUCCESS;
}

order_result_t galipeur_destroy_dune_second_row(void){
  order_result_t or;
  //we must be the first to get there !
  galipeur_set_speed(RS_FAST);
  int16_t traj1[] = {
    DESTROY_DUNE_END_POSITION_X,DESTROY_DUNE_END_POSITION_Y,
    KX(-300), 1600,
  };
  //first traj must be a success, if not leave the area
  or = goto_traj(traj1,KA(M_PI/2));
  if (or != ORDER_SUCCESS){
    //go to destroy end position
    galipeur_set_speed(RS_NORMAL);
    goto_xya_synced(DESTROY_DUNE_END_POSITION_X,
                    DESTROY_DUNE_END_POSITION_Y,
                    KA(M_PI/2));
    return ORDER_FAILURE;
  }

  //slow down to push sand
  galipeur_set_speed(RS_SLOW);
  //destroy 2nd row
  goto_xya(KX(-400), 2000-270, KA(-5*M_PI/6));
  ROME_SENDWAIT_MECA_SET_SAND_ROLLER(&rome_meca, 1);
  goto_xya(KX(-400), 2000-220, KA(-5*M_PI/6));
  galipeur_reset_sandroller();
  goto_xya(KX(-250), 2000-220, KA(-5*M_PI/6));
  galipeur_reset_sandroller();
  goto_xya(KX(-230), 2000-230, KA(-5*M_PI/6));
  ROME_SENDWAIT_MECA_SET_SAND_ROLLER(&rome_meca, 0);
  autoset(ROBOT_SIDE_BACK,AUTOSET_UP, 0, 2000-AUTOSET_OFFSET);
  ROME_SENDWAIT_MECA_SET_SAND_ROLLER(&rome_meca, 1);
  goto_xya(KX(250),  2000-120, KA(M_PI));
  ROME_SENDWAIT_MECA_SET_SAND_ROLLER(&rome_meca, 0);
  return ORDER_SUCCESS;
}

void galipeur_bring_back_sand(void){
#if 1
  galipeur_set_speed(RS_NORMAL);
  goto_xya(KX(1500-700), 1400, KA(M_PI));
  //push sand by successive increments
  goto_xya(KX(1500-800), 1500, KA(M_PI));
  goto_xya_synced(KX(1500-600), 1600, KA(2*M_PI/3));
  goto_xya_synced(KX(1500-600), 1150, KA(2*M_PI/3));

  goto_xya(KX(1500-600), 1250, KA(2*M_PI/3));
  goto_xya_synced(KX(1500-300), 950, KA(M_PI/6));
  goto_xya_synced(KX(350), 950, KA(M_PI/6));
#else
  //push sand by a smooth curvy move
  go_around(KX(1500-900),1230,KA(-M_PI));
#endif
  
  int16_t end[] = {
    KX(550),1000,
    KX(1000),1200,
    DESTROY_DUNE_END_POSITION_X,DESTROY_DUNE_END_POSITION_Y};
  goto_traj(end,KA(M_PI/2));
}

void strat_run_galipeur(void)
{
  ROME_LOG(&rome_paddock,DEBUG,"Go !!!");
  ROME_SENDWAIT_ASSERV_GYRO_INTEGRATION(&rome_asserv, 1);
  ROME_SENDWAIT_ASSERV_ACTIVATE(&rome_asserv, 1);

#if 1
  order_result_t or_dune_row1 = ORDER_FAILURE;
  order_result_t or_dune_row2 = ORDER_FAILURE;
  order_result_t or_doors = ORDER_FAILURE;

  do{
    or_dune_row1 = galipeur_destroy_dune_first_row();
    if (or_dune_row1 == ORDER_SUCCESS)
      galipeur_bring_back_sand();
    else
      if (or_doors != ORDER_SUCCESS)
        or_doors = galipeur_close_doors();
  }
  while(or_dune_row1 != ORDER_SUCCESS);
 
  do{
    or_dune_row2 = galipeur_destroy_dune_second_row();
    if (or_dune_row2 == ORDER_SUCCESS)
      galipeur_bring_back_sand();
    else
      if (or_doors != ORDER_SUCCESS)
        or_doors = galipeur_close_doors();
  }
  while(or_dune_row2 != ORDER_SUCCESS);

  if (or_doors != ORDER_SUCCESS)
    or_doors = galipeur_close_doors();


#else
  //2015 1st match code modified to use advanced detection
  //never tested

  //get out of start area
  ext_arm_lower();
  goto_xya(KX(1500-800), 1030, KA(-M_PI/2));

  struct {
    int16_t x;
    int16_t y;
  } spots_mid[] = {
    { KC(1500-880 ,870 -1500), 645 },
    { KC(1500-1110,1100-1500), 230 },
    { KC(1500-1310,1300-1500), 600 },
  };
  bool carpet_done = false;
  uint8_t spot_i = 0;

  while(spot_i < sizeof(spots_mid)/sizeof(*spots_mid)) {
    order_result_t or = go_pick_spot(spots_mid[spot_i].x, spots_mid[spot_i].y, MECA_RIGHT);
    if(or == ORDER_SUCCESS) {
      spot_i++;
      // special case: relative move needed between spots 0 and 1
      if(!carpet_done && spot_i == 1) {
        or = goto_xya_rel(KX(100),50,0);
        if(or == ORDER_SUCCESS) {
          // ready for spot 1
          continue;
        } else if(!carpet_done) {
          // get carpet
        } else {
          //TODO what should we do?
          // relative move incomplete but carpet already put
        }
      } else {
        continue;
      }
    }

    // robot interrupted: put carpet (if not already done)
    if(!carpet_done) {
      //TODO move to a suitable position
      galipeur_put_carpet();
      carpet_done = true;
    }
    // fallback: retry
  }
  galipeur_unload_spots_start_area();

  galipeur_do_claps();
  //goto_xya(KX(1500-600), 1000, KA(M_PI - M_PI/6));
  if(!carpet_done) {
    galipeur_put_carpet();
  }

#endif

  _delay_ms(3000);
  ROME_SENDWAIT_ASSERV_ACTIVATE(&rome_asserv, 0);
  //ROME_SENDWAIT_MECA_SET_POWER(&rome_meca, 0);
}

void strat_test_galipeur(void)
{
}

/****************************************************************************/

// enum for galipette servo number assignation
typedef enum{
  FISHROD_MOTOR_SPEED = 0,    // PWM that controls the MOTOR speed controler
  FISHROD_MAGNET_RELEASE =1,   // SERVO that separates the fishs and the magnets
  FISHROD_ANGLE = 2,          // SERVO that rotates the full fishrod
  FISHROD_MOTOR_ANGLE = 3,    // SERVO that controls MOTOR angle
} fishrod_servo_t;

#define MAGNET_PWM_POS_RELEASE 2400 // magnet servo position to release fish
#define MAGNET_PWM_POS_CAPTURE 1650 // magnet servo position to capture fish

#define FISHROD_POS_HIGH    3400    // fishrod servo closed position
#define FISHROD_POS_MIDDLE  3200    // fishrod servo position to move fish
#define FISHROD_POS_LOW     2700    // fishrod servo position to capture fish

void galipette_set_speed(robot_speed_t s){
  switch (s){
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
      ROME_SENDWAIT_ASSERV_SET_HTRAJ_XY_CRUISE(&rome_asserv, 80, 0.1);
      break;
    default:
      break;
  }
}


void galipette_rod_prepare_for_fishing(void)
{
  ROME_SENDWAIT_ASSERV_SET_SERVO(&rome_asserv, FISHROD_MAGNET_RELEASE, MAGNET_PWM_POS_CAPTURE);
  ROME_SENDWAIT_ASSERV_SET_SERVO(&rome_asserv, FISHROD_ANGLE, FISHROD_POS_LOW);
}

void galipette_rod_close(void)
{
  ROME_SENDWAIT_ASSERV_SET_SERVO(&rome_asserv, FISHROD_MAGNET_RELEASE, MAGNET_PWM_POS_CAPTURE);
  ROME_SENDWAIT_ASSERV_SET_SERVO(&rome_asserv, FISHROD_ANGLE, FISHROD_POS_HIGH);
}

void galipette_rod_prepare_to_move_fish(void)
{
  ROME_SENDWAIT_ASSERV_SET_SERVO(&rome_asserv, FISHROD_MAGNET_RELEASE, MAGNET_PWM_POS_CAPTURE);
  ROME_SENDWAIT_ASSERV_SET_SERVO(&rome_asserv, FISHROD_ANGLE, FISHROD_POS_MIDDLE);
}

void galipette_rod_release_fish(void)
{
  ROME_SENDWAIT_ASSERV_SET_SERVO(&rome_asserv, FISHROD_ANGLE, FISHROD_POS_MIDDLE);
  ROME_SENDWAIT_ASSERV_SET_SERVO(&rome_asserv, FISHROD_MAGNET_RELEASE, MAGNET_PWM_POS_RELEASE);
  idle_delay_ms(1000);
  ROME_SENDWAIT_ASSERV_SET_SERVO(&rome_asserv, FISHROD_MAGNET_RELEASE, MAGNET_PWM_POS_CAPTURE);
}

void strat_init_galipette(void)
{
  ROME_LOG(&rome_paddock,INFO,"Strat init");
  // disable asserv
  ROME_SENDWAIT_ASSERV_ACTIVATE(&rome_asserv, 1);
  for(;;) {
    idle();
    if(!robot_state.gyro_calibration)
      break;
  }
  galipette_rod_close();
}

void strat_prepare_galipette(void)
{
  //initalise kx factor
  robot_kx = robot_state.team == TEAM_GREEN ? 1 : -1;

  set_xya_wait(KX(1320), 1020, KA(-M_PI/2));

  ROME_SENDWAIT_ASSERV_ACTIVATE(&rome_asserv, 0);
  ROME_SENDWAIT_ASSERV_GYRO_INTEGRATION(&rome_asserv, 0);
}

void galipette_push_sand_stack(void)
{
  // fear the galipeur contact and go away from starting area
  goto_xya(KX(1220), 1020, KA(-M_PI/2));
  
  // align with stack,
  goto_xya(KX(1020), 1100, KA(-M_PI/2));
  
  // push stack
  goto_xya(KX(350), 1000, KA(-M_PI/2));

  // go forward  (galipette safe radius estimated to 200 mm)
  // avoid shells 
  // and prepare to push shell in front of fish
  int16_t traj_sand_extraction[] = {
    KX(400) , 1000,
    KX(400) , 900,
    KX(320) , 620,
    KX(550) , 290,
    KX(760) , 190 };
  goto_traj(traj_sand_extraction,KA(-M_PI/2));

  // push shell
  goto_xya_wait(KX(1000), 180, KA(-M_PI/2 + M_PI/9), 5000);
  goto_xya_wait(KX(750),  190, KA(0), 5000);

  // final connection point : x:750, y:190
}

void galipette_return_fish_to_net(void)
{
  // go away from table edge and aquarium
  goto_xya_rel(KX(0), 100,KA(0));

  // go to net and avoid net fixation
  goto_xya(KX(250), 200, KA(0));

  // robot may enconter table edge
  goto_xya_wait(KX(250), 150, KA(0), 2000);

  galipette_rod_release_fish();
  idle_delay_ms(500);
  
  goto_xya_rel(KX(0), 100,KA(0));
}

void galipette_take_fish(void)
{
  // go fishing (half aquarium near net)
  goto_xya_wait(KX(750),  150, KA(0), 2000); // must be aligned with one fish aquarium edge
  goto_xya_rel(KX(0), 10, KA(0)); // robot must be in contact with table border
  galipette_rod_prepare_for_fishing();
  idle_delay_ms(500);

  goto_xya(KX(900),  160, KA(0)); // must be aligned with one fish aquarium edge
  galipette_rod_prepare_to_move_fish();
  idle_delay_ms(500);

  // go to net and release any fish taken
  galipette_return_fish_to_net();

  goto_xya(KX(850),  190, KA(0)); // must be aligned with one fish aquarium edge
  // go fishing (half aquarium far away from the net)
  galipette_rod_prepare_for_fishing();
  goto_xya_wait(KX(1050),  160, KA(0), 3000); 
  goto_xya(KX(850), 160, KA(0));

  galipette_rod_prepare_to_move_fish();
  
  // go to net and release any fish taken
  galipette_return_fish_to_net();


  while(1); /// TODO REMOVE  THIS HORRIBLE LOOP !!!!!!!!!!!!!!!!!!  
}

void strat_run_galipette(void)
{
  ROME_SENDWAIT_ASSERV_GYRO_INTEGRATION(&rome_asserv, 1);
  ROME_SENDWAIT_ASSERV_ACTIVATE(&rome_asserv, 1);
#if 0 // homologation done !!!

  //homologation
  goto_xya_rel(KX(100), -100, KA(-M_PI/2));
  goto_xya_rel(KX(500), -300, 0);
#else
  galipette_push_sand_stack();

  galipette_take_fish();



#endif
}

void strat_test_galipette(void)
{
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

