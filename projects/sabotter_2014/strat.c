#include <math.h>
#include <stdlib.h>
#include <clock/clock.h>
#include <util/delay.h>
#include <avarix/portpin.h>
#include <rome/rome.h>
#include "rome_acks.h"
#include "strat.h"
#include "common.h"
#include "config.h"

extern rome_intf_t rome_asserv;
extern rome_intf_t rome_meca;
extern rome_intf_t rome_paddock;

robot_state_t robot_state;

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

void ext_arm_set(external_arm_t n, int16_t pos)
{
  ROME_SEND_AND_WAIT(MECA_SET_SERVO, &rome_meca, n, pos);
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
    case EXTARM_LEFT:  ext_arm_set(n, -290); break;
    case EXTARM_RIGHT: ext_arm_set(n, 350); break;
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
    ROME_SEND_AND_WAIT(ASSERV_GOTO_XY, &rome_asserv, x, y, 1000*a);
    robot_state.asserv.xy = 0;
    robot_state.asserv.a = 0;
    for(;;) {
      if(robot_state.asserv.xy && robot_state.asserv.a) {
        return;
      }
      if(opponent_detected()) {
        ROME_SEND_AND_WAIT(ASSERV_GOTO_XY_REL, &rome_asserv, 0, 0, 0);
        ROME_SEND_AND_WAIT(ASSERV_ACTIVATE, &rome_asserv, 0);
        //TODO use opponent_detected_arc()
        while(opponent_detected()) {
          update_rome_interfaces();
        }
        ROME_SEND_AND_WAIT(ASSERV_ACTIVATE, &rome_asserv, 0);
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
    ROME_SEND_AND_WAIT(ASSERV_GOTO_XY_REL, &rome_asserv, x, y, 1000*a);
    robot_state.asserv.xy = 0;
    robot_state.asserv.a = 0;
    for(;;) {
      if(robot_state.asserv.xy && robot_state.asserv.a) {
        return;
      }
      if(opponent_detected()) {
        ROME_SEND_AND_WAIT(ASSERV_GOTO_XY_REL, &rome_asserv, 0, 0, 0);
        ROME_SEND_AND_WAIT(ASSERV_ACTIVATE, &rome_asserv, 0);
        //TODO use opponent_detected_arc()
        while(opponent_detected()) {
          update_rome_interfaces();
        }
        ROME_SEND_AND_WAIT(ASSERV_ACTIVATE, &rome_asserv, 0);
        break; // to resend goto order
      }
      update_rome_interfaces();
    }
  }
}

/// Do an autoset
void autoset(autoset_side_t side, float x, float y)
{
  ROME_SEND_AND_WAIT(ASSERV_AUTOSET, &rome_asserv, side, x, y);
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
    uint32_t tend = get_uptime_us() + STRAT_TIMEOUT_US;
    ROME_SEND_AND_WAIT(MECA_SET_ARM, &rome_meca, shoulder, elbow, wrist);
    do {
      update_rome_interfaces();
      if(abs(shoulder-robot_state.arm.shoulder) < ARM_UPPER_MARGIN &&
         abs(elbow-robot_state.arm.elbow) < ARM_LOWER_MARGIN &&
         abs(wrist-robot_state.arm.wrist) < ARM_LOWER_MARGIN) {
        return;
      }
    } while(get_uptime_us() < tend);
  }
}

/// Move arm but don't wait
void arm_set_nowait(uint16_t shoulder, uint16_t elbow, uint16_t wrist)
{
  ROME_SEND_AND_WAIT(MECA_SET_ARM, &rome_meca, shoulder, elbow, wrist);
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
    if((get_uptime_us() / 500000) % 2 == 0) {
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
      portpin_outset(&LED_R_PP);
      portpin_outclr(&LED_G_PP);
      team = TEAM_RED;
    } else {
      portpin_outset(&LED_R_PP);
      portpin_outset(&LED_G_PP);
      team = TEAM_YELLOW;
    }
    if(starting_cord_plugged()) {
      portpin_outclr(&LED_B_PP);
      break;
    }
    update_rome_interfaces();
  }

  // wait 2s before next step
  uint32_t tend = get_uptime_us() + 2e6;
  while(get_uptime_us() < tend) {
    update_rome_interfaces();
  }

  return team;
}

void strat_wait_start(team_t team)
{
  while(starting_cord_plugged()) {
    if((get_uptime_us() / 500000) % 2 == 0) {
      if(team == TEAM_RED) {
        portpin_outset(&LED_R_PP);
        portpin_outclr(&LED_G_PP);
      } else {
        portpin_outset(&LED_R_PP);
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
  portpin_outclr(&LED_R_PP);
  portpin_outclr(&LED_G_PP);
  portpin_outclr(&LED_B_PP);
  ROME_SEND_AND_WAIT(START_TIMER, &rome_asserv);
  ROME_SEND_AND_WAIT(START_TIMER, &rome_meca);
}


void strat_init_galipeur(void)
{
  // initialize asserv variables
  //ROME_SEND_AND_WAIT(ASSERV_SET_X_PID, &rome_asserv, 3000, 0, 0, 50000, 100000, 0);
  //ROME_SEND_AND_WAIT(ASSERV_SET_Y_PID, &rome_asserv, 3000, 0, 0, 50000, 100000, 0);
  //ROME_SEND_AND_WAIT(ASSERV_SET_A_PID, &rome_asserv, 2500, 0, 0, 50000, 1000, 0);
  //ROME_SEND_AND_WAIT(ASSERV_SET_A_QRAMP, &rome_asserv, 200.0, 100.0);
  //ROME_SEND_AND_WAIT(ASSERV_SET_HTRAJ_XY_CRUISE, &rome_asserv, 20.0, 100.0);
  //ROME_SEND_AND_WAIT(ASSERV_SET_HTRAJ_XY_STEERING, &rome_asserv, 5.0, 0.1);
  //ROME_SEND_AND_WAIT(ASSERV_SET_HTRAJ_XY_STOP, &rome_asserv, 1.0, 0.1);
  //ROME_SEND_AND_WAIT(ASSERV_SET_HTRAJ_XYSTEERING_WINDOW, &rome_asserv, 50.0);
  //ROME_SEND_AND_WAIT(ASSERV_SET_HTRAJ_STOP_WINDOWS, &rome_asserv, 5.0, 0.03);

  // disable asserv
  ROME_SEND_AND_WAIT(ASSERV_ACTIVATE, &rome_asserv, 0);

  // initialize meca
  ROME_SEND_AND_WAIT(MECA_SET_POWER, &rome_meca, 1);
}

void strat_prepare_galipeur(team_t team)
{
  // initialize asserv
  ROME_SEND_AND_WAIT(ASSERV_GOTO_XY, &rome_asserv, 0, 0, 0);
  ROME_SEND_AND_WAIT(ASSERV_ACTIVATE, &rome_asserv, 1);

  // raise both arms
  ext_arm_raise(EXTARM_LEFT);
  ext_arm_raise(EXTARM_RIGHT);

  // autoset robot
  int8_t kx = team == TEAM_RED ? 1 : -1;
  int16_t xoffset = TEAM_RED ? 120 : 100;
  autoset_side_t side = team == TEAM_RED ? AUTOSET_RIGHT : AUTOSET_LEFT;
  autoset(side, kx*(1500-100), 0);
  goto_xya_rel(kx*-xoffset, 0, 0);
  autoset(AUTOSET_DOWN, kx*(1500-100-xoffset), 100);
  goto_xya_rel(+20, 280, 0);

  // prepare meca
  ROME_SEND_AND_WAIT(MECA_SET_PUMP, &rome_meca, 0, 1);
  ROME_SEND_AND_WAIT(MECA_SET_PUMP, &rome_meca, 1, 1);
  ROME_SEND_AND_WAIT(MECA_SET_SUCKER, &rome_meca, 0, 1);
  ROME_SEND_AND_WAIT(MECA_SET_SUCKER, &rome_meca, 1, 1);
}

void strat_run_galipeur(team_t team)
{
  int8_t kx = team == TEAM_RED ? 1 : -1;

  // autoset before starting, to avoid gyro's drift
  //XXX values copy-pasted from above
  // Y value must be the same
  autoset_side_t side = team == TEAM_RED ? AUTOSET_RIGHT : AUTOSET_LEFT;
  autoset(side, kx*(1500-100), 380);
  goto_xya_rel(kx*(-100), 0, 0);

  // go for first fire
  external_arm_t arm = (team == TEAM_RED) ? EXTARM_LEFT : EXTARM_RIGHT;
  goto_xya(kx*(1500-650), 600, -2.0*M_PI/3.0);
  ext_arm_lower(arm);
  goto_xya(kx*(1500-650), 1400, -2.0*M_PI/3.0);
  ext_arm_raise(arm);

  // go for second fire
  ext_arm_lower(EXTARM_RIGHT);
  goto_xya(kx*(1500-700), 800, -M_PI/6.0);
  goto_xya(kx*(1500-1300), 800, -M_PI/6.0);
  ext_arm_raise(EXTARM_RIGHT);

  // raise arm to avoid border
  arm_set_nowait(0,400,200);
  goto_xya(kx*(1500-700), 800, -M_PI/6.0);
  goto_xya(kx*(1500-300), 500, -M_PI/6.0);
  autoset(side, kx*(1500-100), 500);

  // push fire from border
  arm = (team == TEAM_RED) ? EXTARM_LEFT : EXTARM_RIGHT;
  ext_arm_over_border(arm);
  _delay_ms(1000);
  goto_xya_rel(-200,0,0);

}

void strat_test_galipeur(team_t team)
{
  int8_t kx = team == TEAM_RED ? 1 : -1;
  (void)kx;
}

void strat_homologation_galipeur(team_t team)
{
  int8_t kx = team == TEAM_RED ? 1 : -1;

  // exit starting area
  goto_xya(kx*1160, 534, 0);
  // turn
  goto_xya_rel(0, 0, M_PI/3);
  // push fire
  goto_xya_rel(0, 600, 0);

  //end
}


void strat_init_galipette(void)
{
}

void strat_prepare_galipette(team_t team)
{
}

void strat_run_galipette(team_t team)
{
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

