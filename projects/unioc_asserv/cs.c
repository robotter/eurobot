/*  
 *  Copyright RobOtter (2009) 
 * 
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

/** \file cs.c
  * \author JD
  *
  * Manage control systems
  *
  */

#include <avarix.h>
#include <math.h>
#include <perlimpinpin/payload/log.h>

#include "cs.h"
#include "robot_cs.h"
#include "htrajectory.h"
#include "motor_encoders.h"
#include "hrobot_manager.h"
// XXX NDJD : bring me back me ADC module is done
//#include "avoidance.h"

#include "settings.h"

extern ppp_intf_t pppintf;

// Robot position
hrobot_position_t position;
// Robot control systems
robot_cs_t robot_cs;
// Trajectory management
htrajectory_t trajectory;
// robot_cs quadramps
extern struct quadramp_filter qramp_angle;

// Avoidance system
// XXX NDJD : bring me back me ADC module is done
//avoidance_t avoidance;

void cs_initialize(void)
{
  // Initialize robot manager
  PPP_LOG(&pppintf, NOTICE, "Initializing robot manager");
  hrobot_init();

  // Initilialize motor encoders
  PPP_LOG(&pppintf, NOTICE, "Initialize motor encoders");
  motor_encoders_init();

  // Initialize position manager
  PPP_LOG(&pppintf, NOTICE, "Initializing position manager");
  hposition_init( &position );

  // Initialize control systems for robot
  PPP_LOG(&pppintf, NOTICE, "Initializing robot control systems");
  robot_cs_init(&robot_cs);
  robot_cs_set_hrobot_manager(&robot_cs,&system);
  robot_cs_set_hposition_manager(&robot_cs,&position);
  
  // Initialize trajectory management
  PPP_LOG(&pppintf, NOTICE, "Initializing trajectory management");
  htrajectory_init(&trajectory,&position,&robot_cs,&qramp_angle);
  htrajectory_setASpeed(&trajectory, SETTING_TRAJECTORY_A_SPEED,
                                     SETTING_TRAJECTORY_A_ACC);
  htrajectory_setXYCruiseSpeed(&trajectory, SETTING_TRAJECTORY_XYCRUISE_SPEED,
                                            SETTING_TRAJECTORY_XYCRUISE_ACC);
  htrajectory_setXYSteeringSpeed(&trajectory, SETTING_TRAJECTORY_XYSTEERING_SPEED,
                                              SETTING_TRAJECTORY_XYSTEERING_ACC);
  htrajectory_setXYStopSpeed(&trajectory, SETTING_TRAJECTORY_XYSTOP_SPEED,
                                          SETTING_TRAJECTORY_XYSTOP_ACC);

  htrajectory_setSteeringWindow(&trajectory, SETTING_TRAJECTORY_STEERING_XYWIN);
  htrajectory_setStopWindows(&trajectory, SETTING_TRAJECTORY_STOP_XYWIN,
                                          SETTING_TRAJECTORY_STOP_AWIN);

  hposition_set( &position, SETTING_POSITION_INIT_X,
                            SETTING_POSITION_INIT_Y,
                            SETTING_POSITION_INIT_A);

  // Initialize avoidance system
  // XXX NDJD : bring me back me ADC module is done
  //PPP_LOG(&pppintf, NOTICE, "Initializing avoidance system");
  //avoidance_init(&avoidance);
  // XXX
}

void cs_update(void* dummy)
{
  // update avoidance system
  // XXX NDJD : to be included when ADC module is avaible
  //avoidance_update(&avoidance);
  // update trajectory management
  htrajectory_update(&trajectory);
	// update robot position 
	hposition_update(&position);
	// update control systems
	robot_cs_update(&robot_cs);
}
