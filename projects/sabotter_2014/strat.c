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

/// Go to given position, avoid opponents
void goto_xya(int16_t x, int16_t y, float a)
{
  ROME_SEND_AND_WAIT(ASSERV_GOTO_XY, &rome_asserv, x, y, 1000*a);
  robot_state.asserv.xy = 0;
  robot_state.asserv.a = 0;
  while(!robot_state.asserv.xy || !robot_state.asserv.a) {
    //TODO avoid opponent
    update_rome_interfaces();
  }
}

/// Go to given relative position, avoid opponents
void goto_xya_rel(int16_t x, int16_t y, float a)
{
  ROME_SEND_AND_WAIT(ASSERV_GOTO_XY_REL, &rome_asserv, x, y, 1000*a);
  robot_state.asserv.xy = 0;
  robot_state.asserv.a = 0;
  while(!robot_state.asserv.xy || !robot_state.asserv.a) {
    //TODO avoid opponent
    update_rome_interfaces();
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

  // prepare meca
  ROME_SEND_AND_WAIT(MECA_SET_PUMP, &rome_meca, 0, 1);
  ROME_SEND_AND_WAIT(MECA_SET_PUMP, &rome_meca, 1, 1);
  ROME_SEND_AND_WAIT(MECA_SET_SUCKER, &rome_meca, 0, 1);
  ROME_SEND_AND_WAIT(MECA_SET_SUCKER, &rome_meca, 1, 1);

  // autoset robot
  int8_t kx = team == TEAM_RED ? 1 : -1;
  autoset_side_t side = team == TEAM_RED ? AUTOSET_RIGHT : AUTOSET_LEFT;
  autoset(side, kx*(1500-100), 0);
  goto_xya_rel(kx*-120, 0, 0);
  autoset(AUTOSET_DOWN, kx*(1500-100-120), 100);
  goto_xya_rel(0, 280, 0);
}

void strat_run_galipeur(team_t team)
{
  int8_t kx = team == TEAM_RED ? 1 : -1;
  (void)kx;
}

void strat_test_galipeur(team_t team)
{
  int8_t kx = team == TEAM_RED ? 1 : -1;

  // exit starting area
  goto_xya(kx*1050, 534, 0);
  // deploy arm
  arm_set(-9000, 400, -300);
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

