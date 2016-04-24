#include <stdio.h>
#include <stdint.h>

#include "../hrobot_manager.h"
#include "../robot_cs.h"
#include "../quadramp.h"

#include "../htrajectory.h"

int main(int argc, char **argv) {

  htrajectory_t htj;
  hrobot_position_t hrp;
  robot_cs_t rcs;
  struct quadramp_filter qramp_angle;

  htrajectory_init(&htj, &hrp, &rcs, &qramp_angle);

  int steps;
  for(steps=0;steps<1000;steps++) {

    htrajectory_update(&htj);
  }

  return 0;
}
