#include <stdio.h>
#include <stdint.h>

#include "../hrobot_manager.h"
#include "../robot_cs.h"
#include "../quadramp.h"

#include "../htrajectory.h"
#include "../settings.h"

#include "output.h"
extern output_t output;

int main(int argc, char **argv) {

  htrajectory_t htj;
  hrobot_position_t hrp;
  robot_cs_t rcs;
  struct quadramp_filter qramp_angle;

  htrajectory_init(&htj, &hrp, &rcs, &qramp_angle);
  htrajectory_setASpeed(&htj, SETTING_TRAJECTORY_A_SPEED,
                                     SETTING_TRAJECTORY_A_ACC);
  htrajectory_setXYCruiseSpeed(&htj, SETTING_TRAJECTORY_XYCRUISE_SPEED,
                                            SETTING_TRAJECTORY_XYCRUISE_ACC);
  htrajectory_setXYSteeringSpeed(&htj, SETTING_TRAJECTORY_XYSTEERING_SPEED,
                                              SETTING_TRAJECTORY_XYSTEERING_ACC);
  htrajectory_setXYStopSpeed(&htj, SETTING_TRAJECTORY_XYSTOP_SPEED,
                                          SETTING_TRAJECTORY_XYSTOP_ACC);

  htrajectory_setSteeringWindow(&htj, SETTING_TRAJECTORY_STEERING_XYWIN);
  htrajectory_setStopWindows(&htj, SETTING_TRAJECTORY_STOP_XYWIN,
                                          SETTING_TRAJECTORY_STOP_AWIN);

  htrajectory_gotoXY(&htj, 1000,1000);

  int step;
  for(step=0;step<1000;step++) {
    htrajectory_update(&htj);

    fprintf(stdout, "%f,%f,%f,%f,%f\n",
      output.x, output.y, output.a,
      output.x_sp/1000.0, output.y_sp/1000.0);

  }

  return 0;
}
