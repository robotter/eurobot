#include <math.h>
#include <stdlib.h>
#include <clock/clock.h>
#include <util/delay.h>
#include <avarix/portpin.h>
#include <rome/rome.h>
#include <timer/uptime.h>
#include <idle/idle.h>
#include "strat.h"
#include "common.h"
#include "config.h"

#define SPOT_ELEVATOR_LENGTH 83

extern rome_intf_t rome_asserv;
extern rome_intf_t rome_meca;
extern rome_intf_t rome_paddock;

robot_state_t robot_state;

typedef enum {
  ROBOT_SIDE_LEFT = 0,
  ROBOT_SIDE_RIGHT,
  ROBOT_SIDE_BACK,
} robot_side_t;

typedef enum {
  MECA_LEFT = 0,
  MECA_RIGHT = 1
}spot_elevator_t;

typedef enum {
  SPOT_ELEV_S_BUSY = 0,
  SPOT_ELEV_S_GROUND_CLEAR,
  SPOT_ELEV_S_READY,
}spot_elevator_state_t;

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

// Return true if an opponent is detected within an arc
// We assume 0 <= a1 < a2 < 2Pi
bool opponent_detected_arc(float a1, float a2)
{
  for(uint8_t i=0; i<R3D2_OBJECTS_MAX; i++) {
    r3d2_object_t *object = &robot_state.r3d2.objects[i];
    // note: object->a is already in [0;2Pi[
    if(object->detected && object->r < R3D2_AVOID_DISTANCE &&
       object->a >= a1 && object->a >= a2) {
      return true;
    }
  }
  return false;
}

void calibrate_gyro(bool b)
{
  ROME_SENDWAIT_ASSERV_CALIBRATE(&rome_asserv, b);
}

void activate_asserv(bool b)
{
  ROME_SENDWAIT_ASSERV_ACTIVATE(&rome_asserv, b);
}

void ext_arm_set(int16_t pos)
{
  ROME_SENDWAIT_MECA_SET_ARM(&rome_meca, pos);
}

/// Set arm position
void ext_arm_raise(void) { ext_arm_set(430); }
void ext_arm_lower(void) { ext_arm_set(100); }
void ext_arm_clap(void)  { ext_arm_set(300); }
void ext_arm_galipette(void)  { ext_arm_set(180); }

void _wait_meca_ready(void){
  for (;;){
    if((robot_state.left_elev.state == SPOT_ELEV_S_READY)&&
       (robot_state.right_elev.state == SPOT_ELEV_S_READY))
      return;
    idle();
  }
}

void _wait_meca_ground_clear(void){
  for (;;){
    if((robot_state.left_elev.state != SPOT_ELEV_S_BUSY)&&
       (robot_state.right_elev.state != SPOT_ELEV_S_BUSY))
      return;
    idle();
  }
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
  idle();
  _wait_meca_ground_clear();
}

#define _meca_order_blocking_lr(l,r) _meca_order_blocking_left_right(ROME_ENUM_MECA_COMMAND_##l,ROME_ENUM_MECA_COMMAND_##r)
#define _meca_order_blocking_both(cmd) _meca_order_blocking_lr(cmd,cmd)

/// Go to given position, avoid opponents
void goto_xya(int16_t x, int16_t y, float a)
{
  for(;;) {
    ROME_SENDWAIT_ASSERV_GOTO_XY(&rome_asserv, x, y, 1000*a);
    robot_state.asserv.xy = 0;
    robot_state.asserv.a = 0;
    for(;;) {
      if(robot_state.asserv.xy && robot_state.asserv.a) {
        return;
      }
      if(opponent_detected()) {
        ROME_SENDWAIT_ASSERV_GOTO_XY_REL(&rome_asserv, 0, 0, 0);
        ROME_SENDWAIT_ASSERV_ACTIVATE(&rome_asserv, 0);
        //TODO use opponent_detected_arc()
        while(opponent_detected()) {
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
      if(opponent_detected()) {
        ROME_SENDWAIT_ASSERV_GOTO_XY_REL(&rome_asserv, 0, 0, 0);
        ROME_SENDWAIT_ASSERV_ACTIVATE(&rome_asserv, 0);
        //TODO use opponent_detected_arc()
        while(opponent_detected()) {
          idle();
        }
        ROME_SENDWAIT_ASSERV_ACTIVATE(&rome_asserv, 1);
        break; // to resend goto order
      }
      idle();
    }
  }
}


bool starting_cord_plugged_fast(void)
{
  for(;;) {
    bool b = !portpin_in(&STARTING_CORD_PP);
    bool ok = true;
    for(uint16_t i=0; i<100; i++) {
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

/// Go to given position, avoid opponentsa

void goto_xya_painting(int16_t x, int16_t y, float a)
{
  for(;;) {
    ROME_SENDWAIT_ASSERV_GOTO_XY(&rome_asserv, x, y, 1000*a);
    robot_state.asserv.xy = 0;
    robot_state.asserv.a = 0;

    for(;;) {
      if(robot_state.asserv.xy && robot_state.asserv.a) {
        return;
      }
      if(starting_cord_plugged_fast()) {
        // XXX starting cord is connected to painting click
        return;
      }
      if(opponent_detected()) {
        ROME_SENDWAIT_ASSERV_GOTO_XY_REL(&rome_asserv, 0, 0, 0);
        ROME_SENDWAIT_ASSERV_ACTIVATE(&rome_asserv, 0);
        //TODO use opponent_detected_arc()
        while(opponent_detected()) {
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
void goto_xya_rel(int16_t x, int16_t y, float a)
{
  for(;;) {
    ROME_SENDWAIT_ASSERV_GOTO_XY_REL(&rome_asserv, x, y, 1000*a);
    robot_state.asserv.xy = 0;
    robot_state.asserv.a = 0;
    for(;;) {
      if(robot_state.asserv.xy && robot_state.asserv.a) {
        return;
      }
      if(opponent_detected()) {
        ROME_SENDWAIT_ASSERV_GOTO_XY_REL(&rome_asserv, 0, 0, 0);
        ROME_SENDWAIT_ASSERV_ACTIVATE(&rome_asserv, 0);
        //TODO use opponent_detected_arc()
        while(opponent_detected()) {
          idle();
        }
        ROME_SENDWAIT_ASSERV_ACTIVATE(&rome_asserv, 1);
        break; // to resend goto order
      }
      idle();
    }
  }
}

void goto_xy_rel_align_course(int16_t x, int16_t y, bool claws_first){

  float angle;
  //get angle of movement
  angle = atan2(y,x)-M_PI/2;
  if(claws_first)
    angle += M_PI;

  //compute relative angle consign
  angle -= robot_state.current_pos.a;

  goto_xya_rel(x,y,angle);
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
      team = TEAM_YELLOW;
    }
    if(starting_cord_plugged()) {
      portpin_outclr(&LED_B_PP);
      break;
    }
    idle();
  }

  // wait 2s before next step
  uint32_t tend = uptime_us() + 2e6;
  while(uptime_us() < tend) {
    idle();
  }

  return team;
}

void strat_wait_start(team_t team)
{
/*
  // deactivate asserv, turn gyro calibration mode on
  activate_asserv(false);
  calibrate_gyro(true);
*/
  while(starting_cord_plugged()) {
    if((uptime_us() / 500000) % 2 == 0) {
      if(team == TEAM_YELLOW) {
        portpin_outset(&LED_R_PP);
        portpin_outset(&LED_G_PP);
      } else {
        portpin_outset(&LED_G_PP);
      }
      portpin_outclr(&LED_B_PP);
    } else {
      portpin_outclr(&LED_R_PP);
      portpin_outclr(&LED_G_PP);
      portpin_outset(&LED_B_PP);
    }
    idle();
  }

  /*
  // stop gyro calibration, turn asserv on 
  calibrate_gyro(false);
  _delay_ms(100);
  activate_asserv(true);
*/
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
  // disable asserv
  ROME_SENDWAIT_ASSERV_ACTIVATE(&rome_asserv, 0);

  // initialize meca
  ROME_SENDWAIT_MECA_SET_POWER(&rome_meca, 1);

  ext_arm_lower(); 

  ROME_SENDWAIT_MECA_CARPET_UNLOCK(&rome_meca, MECA_RIGHT);
  ROME_SENDWAIT_MECA_CARPET_UNLOCK(&rome_meca, MECA_LEFT);
  _meca_order_blocking_both(PREPARE_BULB);
  for(;;) {
    update_rome_interfaces();
    if(!robot_state.gyro_calibration)
      break;
    }
  _meca_order_blocking_both(PICK_BULB);
  ROME_SENDWAIT_MECA_CARPET_LOCK(&rome_meca, MECA_RIGHT);
  ROME_SENDWAIT_MECA_CARPET_LOCK(&rome_meca, MECA_LEFT);
}

void strat_prepare_galipeur(team_t team)
{
  
#if 1
  // initialize asserv
  ROME_SENDWAIT_ASSERV_SET_XYA(&rome_asserv, 0, 0, 0);
  ROME_SENDWAIT_ASSERV_GOTO_XY(&rome_asserv, 0, 0, 0);
  ROME_SENDWAIT_ASSERV_ACTIVATE(&rome_asserv, 1);

  // autoset robot
  autoset(ROBOT_SIDE_RIGHT,AUTOSET_RIGHT, 1500-100, 0);
  goto_xya_rel(-500, 0, 0);
  goto_xya_rel(0, -350, -M_PI/4);
  autoset(ROBOT_SIDE_RIGHT,AUTOSET_DOWN, 0, 100);

  // move out (in relative coordinates)
  goto_xya_rel(0, 200, 0);
  // approach front of start area
  goto_xya(1500-700, 1040, 0);

  // stack in start area
  goto_xya(1500-400,1040,0);
#else
  ROME_SENDWAIT_ASSERV_SET_XYA(&rome_asserv, 1220, 1040, 0);
  ROME_SENDWAIT_ASSERV_GOTO_XY(&rome_asserv, 1220, 1040, 0);
  ROME_SENDWAIT_ASSERV_ACTIVATE(&rome_asserv, 1);
#endif

  //prepare meca
  _meca_order_blocking_both(RESET_ELEVATOR);
  _meca_order_blocking_both(PICK_BULB);
}

void strat_run_galipeur(team_t team)
{
  // front of start area
  goto_xya(1500-700, 1030, 0);

  goto_xya(1500-700, 650, 0);
  goto_xya(1500-200, 650, 0);
  
  // autoset against right side
  autoset(ROBOT_SIDE_RIGHT,AUTOSET_RIGHT, 1500-100, 0);
  _meca_order_blocking_lr(PREPARE_CUP,NONE);
  goto_xya_rel(-100,0,0);

  // -- SE SPOTS --
  // approach SE spots
  goto_xya(1500-160, 470, 0);
  // pick one spot
  _wait_meca_ready();
  goto_xya(1500-160, 450, 0);
  _meca_order_blocking_lr(PICK_CUP,PICK_SPOT);
  
  goto_xya(1500-160, 450, 0);
  // pick one spot
  _wait_meca_ready();
  goto_xya(1500-160, 340, 0);
  _meca_order_blocking_lr(NONE,PICK_SPOT);

  // -- SE CLAPS -- 
  goto_xya(1500-160, 340, 0);
  // translate along SE bordea
  ext_arm_clap();
  goto_xya(1500-300, 340, 0);
  ext_arm_raise();
  goto_xya(1500-700, 340, 0);
  ext_arm_clap();
  goto_xya(1500-880, 340, 0);

  // -- S SPOTS --
  goto_xya_rel(+100,0,0);
  ext_arm_lower();
  autoset(ROBOT_SIDE_RIGHT,AUTOSET_DOWN, 0, 100);

  goto_xya_rel(0,100,0);

  _wait_meca_ready();
  goto_xya_rel(-200, 0, 0);
  _meca_order_blocking_lr(NONE,PICK_SPOT);

  // -- MIDDLE SPOT --
  goto_xya(1500-1270, 350, -M_PI);
  _wait_meca_ready();
  goto_xya(1500-1270, 550, -M_PI);
  _meca_order_blocking_lr(UNLOAD_CUP,PICK_SPOT);

  // -- DISCHARGE ON THE RED AREA --
  goto_xya(1500-1050, 300, -M_PI/4);
  _meca_order_blocking_lr(NONE,DISCHARGE_SPOT_STACK);
  goto_xy_rel_align_course(-65, -75, true);
  _meca_order_blocking_lr(NONE,RELEASE_SPOT_STACK);
  goto_xy_rel_align_course(65*1.2, 75*1.2, false);
  goto_xya_rel(200,0,0);
  autoset(ROBOT_SIDE_RIGHT,AUTOSET_DOWN, 0, 100);
  goto_xya_rel(0,50,0);

  goto_xya(1500-700, 600, M_PI/2);

  // -- BACK TO START AREA --
  _meca_order_blocking_lr(NONE,PREPARE_BULB);
  goto_xya(1500-700, 1030, M_PI/2);
  // -- TAKE GALIPETTE --
  goto_xya(1500-420, 1040, M_PI/2);
  ext_arm_galipette();
  _delay_ms(500);
  goto_xya(1500-420, 1040, M_PI/2+M_PI/4);
  ext_arm_raise();
  _delay_ms(500);
 
  // -- TAKE BULB --
  goto_xya(1500-350, 1000, M_PI/2);
  autoset(ROBOT_SIDE_BACK,AUTOSET_RIGHT, 1500-227, 0);
  _meca_order_blocking_lr(NONE,PICK_BULB);
  goto_xya(1500-700, 1030, M_PI/2);

  // -- GO TO STAIRS ! --
  goto_xya(1500-800, 1600, M_PI);
  goto_xya(1500-830, 1700, M_PI);
  _meca_order_blocking_lr(NONE,PICK_SPOT);
  _wait_meca_ready();
  goto_xya(1500-830, 1800, M_PI);
  _meca_order_blocking_lr(NONE,PICK_SPOT);
  
  goto_xya_rel(0,-160,0);
  autoset(ROBOT_SIDE_RIGHT,AUTOSET_LEFT, 1500-967-100, 0);
  goto_xya_rel(100, 0,0);
  goto_xya(1500-950, 1600, M_PI - M_PI/6);
  ROME_SENDWAIT_MECA_CARPET_UNLOCK(&rome_meca, MECA_RIGHT);
  ROME_SENDWAIT_MECA_CARPET_UNLOCK(&rome_meca, MECA_LEFT);

  goto_xya(1500-880, 1800, -M_PI/2);
  ext_arm_clap();
  _delay_ms(500);
  goto_xya_rel(200, 0, 0);
  //we can make only 4 spots pile ... so sad ...
  //goto_xya(1500-830, 350, -M_PI); 
  //goto_xya_rel(0, 200, 0);
  //_meca_order_blocking_lr(NONE,PICK_SPOT);

  //goto_xya(1500-600, 1030, M_PI/2);  
  //goto_xya_rel(200, 0, 0);
  //_meca_order_blocking_lr(NONE,PREPARE_CUP);

  //goto_xya(1500-600, 1030, M_PI/2);  
  //goto_xya(1500-920, 900,  M_PI);
  //goto_xya(1500-920, 1070, M_PI);
  //_meca_order_blocking_lr(NONE,PICK_CUP);

  //goto_xya(1500-600, 1030, M_PI/2);  
  //_meca_order_blocking_lr(NONE,UNLOAD_CUP);


  //back to prog !
  _delay_ms(10000);
  goto_xya(1500-900,1500,M_PI/2);

  _delay_ms(3000);
  ROME_SENDWAIT_ASSERV_ACTIVATE(&rome_asserv, 0);
  ROME_SENDWAIT_MECA_SET_POWER(&rome_meca, 0);
}

void strat_test_galipeur(team_t team)
{
  int8_t kx = team == TEAM_YELLOW ? 1 : -1;
  (void)kx;
}

void strat_homologation_galipeur(team_t team)
{
  int8_t kx = team == TEAM_YELLOW ? 1 : -1;

  // exit starting area
  goto_xya(kx*1160, 534, 0);
  // turn
  goto_xya_rel(0, 0, M_PI/3);
  // push fire
  goto_xya_rel(0, 600, 0);

  //end
}

/****************************************************************************/

void strat_init_galipette(void)
{
}

void strat_prepare_galipette(team_t team)
{
  (void) team;
}

void strat_run_galipette(team_t team)
{
  (void) team;
}

void strat_test_galipette(team_t team)
{
  (void) team;
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

void strat_prepare(team_t team)
{
  ROBOT_FUNCTION(strat_prepare_)(team);
}

void strat_run(team_t team)
{
  ROBOT_FUNCTION(strat_run_)(team);
}

void strat_test(team_t team)
{
  ROBOT_FUNCTION(strat_test_)(team);
}

