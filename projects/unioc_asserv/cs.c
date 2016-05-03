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
#include <stdio.h>

#include "cs.h"
#include "robot_cs.h"
#include "htrajectory.h"
#include "motor_encoders.h"
#include "hrobot_manager.h"
#include "adxrs/adxrs.h"
#include "avoidance.h"
#include "bumpers.h"

#include "settings.h"

// Robot position
hrobot_position_t position;
// Robot control systems
robot_cs_t robot_cs;
// Trajectory management
htrajectory_t trajectory;
// robot_cs quadramps
extern struct quadramp_filter qramp_angle;

// Avoidance system
avoidance_t avoidance;

void cs_initialize(void)
{
  // Initialize robot manager
  hrobot_init();

  // Initilialize motor encoders
  motor_encoders_init();

  // Initialize z gyro
  adxrs_init(PORTPIN(K,0));
  adxrs_startup();
  adxrs_capture_manual(0);

  // Initialize position manager
  hposition_init( &position );

  // Initialize control systems for robot
  robot_cs_init(&robot_cs);
  robot_cs_set_hrobot_manager(&robot_cs,&system);
  robot_cs_set_hposition_manager(&robot_cs,&position);
  
  // Initialize trajectory management
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
  avoidance_init(&avoidance);

#if defined(GALIPETTE)
  bumpers_init();
#endif
}

void cs_update(void* dummy)
{
  // update avoidance system
  avoidance_update(&avoidance);
  // update trajectory management
  htrajectory_update(&trajectory);
	// update robot position 
	hposition_update(&position);
	// update control systems
	robot_cs_update(&robot_cs);
#if defined(GALIPETTE)
  // update bumpers
  bumpers_update();
#endif
}
