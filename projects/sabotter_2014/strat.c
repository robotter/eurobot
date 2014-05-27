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
void goto_xya(int16_t x, int16_t y, int16_t a)
{
  ROME_SEND_AND_WAIT(ASSERV_GOTO_XY, &rome_asserv, x, y, a);
  robot_state.asserv.xy = 0;
  robot_state.asserv.a = 0;
  while(!robot_state.asserv.xy || !robot_state.asserv.a) {
    //TODO avoid opponent
    update_rome_interfaces();
  }
}

/// Go to given relative position, avoid opponents
void goto_xya_rel(int16_t x, int16_t y, int16_t a)
{
  ROME_SEND_AND_WAIT(ASSERV_GOTO_XY_REL, &rome_asserv, x, y, a);
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

  // initialize meca
  ROME_SEND_AND_WAIT(MECA_SET_ARM, &rome_meca, 0, 0, 0);
  //ROME_SEND_AND_WAIT(MECA_SET_PUMP, &rome_meca, 0, 1);
  //ROME_SEND_AND_WAIT(MECA_SET_PUMP, &rome_meca, 1, 1);
  //ROME_SEND_AND_WAIT(MECA_SET_SUCKER, &rome_meca, 0, 1);
  //ROME_SEND_AND_WAIT(MECA_SET_SUCKER, &rome_meca, 1, 1);
  //_delay_ms(1000);
  //ROME_SEND_AND_WAIT(MECA_SET_POWER, &rome_meca, 0);
}

void strat_run_galipeur(team_t team)
{
  int8_t kx = team == TEAM_RED ? 1 : -1;
  // set start position
  //TODO adjust
  ROME_SEND_AND_WAIT(ASSERV_SET_XYA, &rome_asserv,
                     kx*(1500-120), 150, team == TEAM_RED ? M_PI/2 : M_PI/6);
  ROME_SEND_AND_WAIT(ASSERV_ACTIVATE, &rome_asserv, 1);

  // exit start zone
  goto_xya_rel(kx*150, -500, 0);
  //TODO deploy arm

  // go in front of first fire and push it
  goto_xya(kx*110, 800, 0);
  goto_xya_rel(kx*0, 100, 0);
}

void strat_test_galipeur(void)
{
}


void strat_init_galipette(void)
{
}

void strat_test_galipette(void)
{
}

void strat_run_galipette(team_t team)
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

void strat_run(team_t team)
{
  ROBOT_FUNCTION(strat_run_)(team);
}

void strat_test(void)
{
  ROBOT_FUNCTION(strat_test_)();
}

