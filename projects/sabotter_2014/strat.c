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
  while(!robot_state.asserv.xy || !robot_state.asserv.a) {
    //TODO avoid opponent
    update_rome_interfaces();
  }
}

/// Go to given relative position, avoid opponents
void goto_xya_rel(int16_t x, int16_t y, int16_t a)
{
  ROME_SEND_AND_WAIT(ASSERV_GOTO_XY_REL, &rome_asserv, x, y, a);
  while(!robot_state.asserv.xy || !robot_state.asserv.a) {
    //TODO avoid opponent
    update_rome_interfaces();
  }
}

/// Do an autoset
void autoset(uint8_t side, float x, float y)
{
  ROME_SEND_AND_WAIT(ASSERV_AUTOSET, &rome_asserv, side, x, y);
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


void strat_run(team_t team)
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

