#include <math.h>
#include <stdlib.h>
#include <clock/clock.h>
#include <util/delay.h>
#include <avarix/portpin.h>
#include <rome/rome.h>
#include <timer/uptime.h>
#include "strat.h"
#include "common.h"
#include "config.h"

#define SPOT_ELEVATOR_LENGTH 83

extern rome_intf_t rome_asserv;
extern rome_intf_t rome_meca;
extern rome_intf_t rome_paddock;

robot_state_t robot_state;

typedef enum {
  SPOT_ELV_LEFT = 0,
  SPOT_ELV_RIGHT = 1
}spot_elevator_t;

typedef enum {
  EXTARM_LEFT = 0,
  EXTARM_FRONT = 1,
  EXTARM_RIGHT = 2,
}external_arm_t;

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

void ext_arm_set(external_arm_t n, int16_t pos)
{
  ROME_SENDWAIT_MECA_SET_SERVO(&rome_meca, n, pos);
}

void katioucha_fire(uint8_t n)
{
  ROME_SENDWAIT_KATIOUCHA_FIRE(&rome_asserv, n);
}

/// Set side arm position
void ext_arm_raise(external_arm_t n)
{
  switch(n) {
    case EXTARM_LEFT:  ext_arm_set(n, 120); break;
    case EXTARM_RIGHT: ext_arm_set(n, -60); break;
    default: return;
  }
}

void ext_arm_lower(external_arm_t n)
{
  switch(n) {
    case EXTARM_LEFT:  ext_arm_set(n, -100); break;
    case EXTARM_RIGHT: ext_arm_set(n, 150); break;
    default: return;
  }
}

void ext_arm_over_border(external_arm_t n)
{
  switch(n) {
    case EXTARM_LEFT:  ext_arm_set(n, -240); break;
    case EXTARM_RIGHT: ext_arm_set(n, 300); break;
    default: return;
  }
}


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
          update_rome_interfaces();
        }
        ROME_SENDWAIT_ASSERV_ACTIVATE(&rome_asserv, 1);
        break; // to resend goto order
      }
      update_rome_interfaces();
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
          update_rome_interfaces();
        }
        ROME_SENDWAIT_ASSERV_ACTIVATE(&rome_asserv, 1);
        break; // to resend goto order
      }
      update_rome_interfaces();
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

/// Go to given position, avoid opponents
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
          update_rome_interfaces();
        }
        ROME_SENDWAIT_ASSERV_ACTIVATE(&rome_asserv, 1);
        break; // to resend goto order
      }
      update_rome_interfaces();
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
          update_rome_interfaces();
        }
        ROME_SENDWAIT_ASSERV_ACTIVATE(&rome_asserv, 1);
        break; // to resend goto order
      }
      update_rome_interfaces();
    }
  }
}

/// Do an autoset
void autoset(autoset_side_t side, float x, float y)
{
  ROME_SENDWAIT_ASSERV_AUTOSET(&rome_asserv, side, x, y);
  robot_state.asserv.autoset = 0;
  while(!robot_state.asserv.autoset) {
    //TODO avoid opponent
    update_rome_interfaces();
  }
}

/// Move arm and wait for it to be in position
void arm_set(uint16_t shoulder, uint16_t elbow, uint16_t wrist)
{
  for(;;) {
    uint32_t tend = uptime_us() + STRAT_TIMEOUT_US;
    ROME_SENDWAIT_MECA_SET_ARM(&rome_meca, shoulder, elbow, wrist);
    do {
      update_rome_interfaces();
      if(abs(shoulder-robot_state.arm.shoulder) < ARM_UPPER_MARGIN &&
         abs(elbow-robot_state.arm.elbow) < ARM_LOWER_MARGIN &&
         abs(wrist-robot_state.arm.wrist) < ARM_LOWER_MARGIN) {
        return;
      }
    } while(uptime_us() < tend);
  }
}

/// Move arm but don't wait
void arm_set_nowait(uint16_t shoulder, uint16_t elbow, uint16_t wrist)
{
  ROME_SENDWAIT_MECA_SET_ARM(&rome_meca, shoulder, elbow, wrist);
}

/// delay for some ms
void strat_delay_ms(uint32_t ms) {
  uint32_t tend = uptime_us() + 1000*ms;
  for(;;) {
    if(uptime_us() >= tend) {
      return;
    }
    update_rome_interfaces();
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
    update_rome_interfaces();
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
    update_rome_interfaces();
  }

  // wait 2s before next step
  uint32_t tend = uptime_us() + 2e6;
  while(uptime_us() < tend) {
    update_rome_interfaces();
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
    update_rome_interfaces();
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
  // initialize asserv variables
  //ROME_SENDWAIT_ASSERV_SET_X_PID(&rome_asserv, 3000, 0, 0, 50000, 100000, 0);
  //ROME_SENDWAIT_ASSERV_SET_Y_PID(&rome_asserv, 3000, 0, 0, 50000, 100000, 0);
  //ROME_SENDWAIT_ASSERV_SET_A_PID(&rome_asserv, 2500, 0, 0, 50000, 1000, 0);
  //ROME_SENDWAIT_ASSERV_SET_A_QRAMP(&rome_asserv, 200.0, 100.0);
  //ROME_SENDWAIT_ASSERV_SET_HTRAJ_XY_CRUISE(&rome_asserv, 20.0, 100.0);
  //ROME_SENDWAIT_ASSERV_SET_HTRAJ_XY_STEERING(&rome_asserv, 5.0, 0.1);
  //ROME_SENDWAIT_ASSERV_SET_HTRAJ_XY_STOP(&rome_asserv, 1.0, 0.1);
  //ROME_SENDWAIT_ASSERV_SET_HTRAJ_XYSTEERING_WINDOW(&rome_asserv, 50.0);
  //ROME_SENDWAIT_ASSERV_SET_HTRAJ_STOP_WINDOWS(&rome_asserv, 5.0, 0.03);


  // disable asserv
  ROME_SENDWAIT_ASSERV_ACTIVATE(&rome_asserv, 0);

  // initialize meca
  ROME_SENDWAIT_MECA_SET_POWER(&rome_meca, 1);
}

void strat_prepare_galipeur(team_t team)
{
  // initialize asserv
  ROME_SENDWAIT_ASSERV_SET_XYA(&rome_asserv, 0, 0, -M_PI/2);
  ROME_SENDWAIT_ASSERV_GOTO_XY(&rome_asserv, 0, 0, -M_PI/2);
  ROME_SENDWAIT_ASSERV_ACTIVATE(&rome_asserv, 1);

  // raise both arms
  ext_arm_raise(EXTARM_LEFT);
  ext_arm_raise(EXTARM_RIGHT);

  // autoset robot
  int8_t kx = team == TEAM_YELLOW ? -1 : 1;
  autoset(AUTOSET_DOWN, 0, 100+SPOT_ELEVATOR_LENGTH);
  goto_xya_rel(0, 100, 0);
  goto_xya(0, 550, -M_PI/2);
  goto_xya(-350, 550, -M_PI/2);
  autoset_side_t side = team == TEAM_YELLOW ? AUTOSET_LEFT : AUTOSET_RIGHT;
  autoset(side, kx*(1500-100-SPOT_ELEVATOR_LENGTH), 450);

  goto_xya_rel(500, 0, 0);
  goto_xya_rel(0, 420, 0);
  goto_xya(kx*(1500-230-SPOT_ELEVATOR_LENGTH), 1000, -M_PI/2);

  // prepare meca
  ROME_SENDWAIT_MECA_PREPARE_FOR_ONBOARD_BULB(&rome_meca, SPOT_ELV_LEFT);
  //ROME_SENDWAIT_MECA_PICK_ONE_SPOT(&rome_meca, SPOT_ELV_LEFT);
}

static void _pick_spot_corner_0_0(team_t team)
{

  int8_t kx = team == TEAM_YELLOW ? -1 : +1;
  goto_xya(kx*(1500-450),310,-M_PI/2);
  strat_delay_ms(1000);

//  approach spot
  //goto_xya_rel(-200,0,0);
  goto_xya(kx*(1500-280),310,-M_PI/2);
  strat_delay_ms(1000);

  //ext_arm_lower(EXTARM_RIGHT);
  ROME_SENDWAIT_MECA_PICK_ONE_SPOT(&rome_meca, SPOT_ELV_LEFT);
  strat_delay_ms(5000);

  goto_xya(kx*(1500-370),310,-M_PI/2);
  strat_delay_ms(1000);

  goto_xya(kx*(1500-370),220,-M_PI/2);
  strat_delay_ms(1000);

  goto_xya(kx*(1500-280),220,-M_PI/2);
  strat_delay_ms(1000);

  ROME_SENDWAIT_MECA_PICK_ONE_SPOT(&rome_meca, SPOT_ELV_LEFT);
  strat_delay_ms(5000);
 
}

void strat_run_galipeur(team_t team)
{
  int8_t kx = team == TEAM_YELLOW ? -1 : +1;

  #if 0
  // autoset before starting, to avoid gyro's drift
  autoset_side_t side = team == TEAM_YELLOW ? AUTOSET_LEFT : AUTOSET_RIGHT;
  ROME_SENDWAIT_MECA_PICK_ONE_SPOT(&rome_meca, SPOT_ELV_LEFT);
  strat_delay_ms(500);
  autoset(side, kx*(1500-100-70), 1000);
#endif

  goto_xya_rel(400,0,0);
  strat_delay_ms(1000);
  goto_xya_rel(0,-400,0);
  strat_delay_ms(1000);
  
  _pick_spot_corner_0_0(team);

while(1);
  goto_xya(kx*(1500-300),200,-M_PI/2 + M_PI/3);

  ext_arm_raise(EXTARM_RIGHT);
  strat_delay_ms(500);

  goto_xya(kx*(1500-600),200,-M_PI/2 + M_PI/3);
  ext_arm_lower(EXTARM_RIGHT);
  strat_delay_ms(500);

  goto_xya(kx*(1500-900),200,-M_PI/2 + M_PI/3);
  ext_arm_raise(EXTARM_RIGHT);
  goto_xya(kx*(1500-980),370,-M_PI/2 - M_PI/4 + M_PI/3);
  goto_xya_rel(150*0.7,-150*0.7,0);
  goto_xya(kx*(1500-1000),500,-M_PI/2);
  goto_xya(kx*(1500-1000),500,M_PI/2 - M_PI/4 + M_PI/3);
  goto_xya(kx*(1500-550),900,M_PI/2 - M_PI/4 + M_PI/3);

  //go push oponent side clap !
  //goto_xya(kx*(-1500+1100),900,-M_PI/2 + M_PI/3);
  //goto_xya(kx*(-1500+300),200,-M_PI/2  + M_PI/3);
  //ext_arm_lower(EXTARM_RIGHT);
  //strat_delay_ms(500);
  //goto_xya(kx*(-1500+500),200,-M_PI/2  + M_PI/3);
  //ext_arm_raise(EXTARM_RIGHT);
  //strat_delay_ms(500);
  //ext_arm_lower(EXTARM_RIGHT);
  //strat_delay_ms(500);
  //ext_arm_raise(EXTARM_RIGHT);

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
  // initialize asserv variables
  //ROME_SENDWAIT_ASSERV_SET_X_PID(&rome_asserv, 3000, 0, 0, 50000, 100000, 0);
  //ROME_SENDWAIT_ASSERV_SET_Y_PID(&rome_asserv, 3000, 0, 0, 50000, 100000, 0);
  //ROME_SENDWAIT_ASSERV_SET_A_PID(&rome_asserv, 2500, 0, 0, 50000, 1000, 0);
  //ROME_SENDWAIT_ASSERV_SET_A_QRAMP(&rome_asserv, 200.0, 100.0);
  //ROME_SENDWAIT_ASSERV_SET_HTRAJ_XY_CRUISE(&rome_asserv, 10.0, 0.05);
  //ROME_SENDWAIT_ASSERV_SET_HTRAJ_XY_STEERING(&rome_asserv, 5.0, 0.1);
  
  ROME_SENDWAIT_ASSERV_SET_HTRAJ_XY_STOP(&rome_asserv, 1.0, 0.05);
  //ROME_SENDWAIT_ASSERV_SET_HTRAJ_XYSTEERING_WINDOW(&rome_asserv, 50.0);
  //ROME_SENDWAIT_ASSERV_SET_HTRAJ_STOP_WINDOWS(&rome_asserv, 5.0, 0.03);
  // set position
  //ROME_SENDWAIT_ASSERV_SET_XYA(&rome_asserv, 820,-1300,-M_PI_4);

  // disable asserv
  ROME_SENDWAIT_ASSERV_ACTIVATE(&rome_asserv, 0);
}

void strat_prepare_galipette(team_t team)
{
  // initialize asserv
  ROME_SENDWAIT_ASSERV_SET_XYA(&rome_asserv, -1300,1100,0);
  ROME_SENDWAIT_ASSERV_GOTO_XY(&rome_asserv, -1300,1100,0);
  ROME_SENDWAIT_ASSERV_ACTIVATE(&rome_asserv, 1);
  // inhibit avoidance
  ROME_SENDWAIT_ASSERV_AVOIDANCE(&rome_asserv, 0);
}

void strat_run_galipette(team_t team)
{
  //strat_delay_ms(9000);

  int8_t kx = team == TEAM_YELLOW ? 1 : -1;

  kx = 1;
  
  goto_xya(-1300,1100, 0);
  goto_xya(-1200,1000, 0);

  // go around pop corn glass
  goto_xya(-400,1000,0);
  goto_xya(-400,1300,M_PI/3);
  goto_xya(-460,1300,M_PI/3);
  strat_delay_ms(500);
  goto_xya(-1000,1000,M_PI/3);

  strat_delay_ms(2000);
  ROME_SENDWAIT_ASSERV_SET_A_QRAMP(&rome_asserv, 300.0, 300.0);
  //ROME_SENDWAIT_ASSERV_SET_HTRAJ_XY_CRUISE(&rome_asserv, 10.0, 50.0);
  goto_xya_panning(-400, 1000, M_PI/3);
  goto_xya(-400,1000,M_PI/3);
  goto_xya_panning(-400,1300,M_PI/3);
  goto_xya(-400,1300,M_PI/3);
  ROME_SENDWAIT_ASSERV_SET_A_QRAMP(&rome_asserv, 200.0, 100.0);
  //ROME_SENDWAIT_ASSERV_SET_HTRAJ_XY_CRUISE(&rome_asserv, 20.0, 100.0);
  goto_xya(-460,1300,M_PI/3);
  goto_xya(-1000,1000,M_PI/3);
  while(1) 
          update_rome_interfaces();
  strat_delay_ms(5000);
  goto_xya(-300, 900,0);
  strat_delay_ms(5000);
  goto_xya(-400,980,0);
  strat_delay_ms(2000);

  //push it to cinema
  goto_xya(-1300,820,0);
  strat_delay_ms(1000);

  // then go to stares
  goto_xya(-500,1100,0);
  goto_xya(-50,1100,0);

  strat_delay_ms(5000);
  while(1);
  // go out of table for at least 1m and pray 
  // XXX tirette is connected to painting click XXX
  goto_xya_painting(kx*1000,1000,M_PI);
  goto_xya_rel(0,0,0);
  strat_delay_ms(500);

  goto_xya_rel(0,-200,0);
}

void strat_test_galipette(team_t team)
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

