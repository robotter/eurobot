#include <math.h>
#include <clock/clock.h>
#include <rome/rome.h>
#include "rome_acks.h"
#include "strat.h"

extern rome_intf_t rome_asserv;
extern rome_intf_t rome_meca;
extern rome_intf_t rome_paddock;


void strat_run(team_t team)
{
  int8_t kx = team == TEAM_RED ? 1 : -1;
  // set start position
  //TODO adjust
  ROME_SEND_AND_WAIT(ASSERV_SET_XYA, &rome_asserv,
                     kx*(1500-120), 150, team == TEAM_RED ? M_PI/2 : M_PI/6);
  ROME_SEND_AND_WAIT(ASSERV_ACTIVATE, &rome_asserv, 1);

  // exit start zone
  ROME_SEND_AND_WAIT(ASSERV_GOTO_XY_REL, &rome_asserv, kx*150, -500, 0);
  //TODO deploy arm

  // go in front of first fire and push it
  ROME_SEND_AND_WAIT(ASSERV_GOTO_XY, &rome_asserv, kx*110, 800, 0);
  ROME_SEND_AND_WAIT(ASSERV_GOTO_XY_REL, &rome_asserv, 0, 100, 0);
}

