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

/** \file robot_cs.c
  * \author JD
  *
  * Manage control systems for robot
  *
  */

#include "robot_cs.h"

#include <math.h>
#include "control_system_manager.h"
#include "pid.h"
#include "pid_config.h"
#include <quadramp.h>
#include "settings.h"

// control system managers
struct cs csm_x;
struct cs csm_y;
struct cs csm_angle;

// quadramp
struct quadramp_filter qramp_angle;

// pids
struct pid_filter pid_x;
struct pid_filter pid_y;
struct pid_filter pid_angle;

void robot_cs_init(robot_cs_t* rcs)
{
  // zeroing structures
  rcs->hrs = NULL;
  rcs->hpm = NULL;

  // set CS on
  rcs->active = 1;

  // CS not reactivated since last tick
  rcs->reactivated = 0;

	// setup PIDs
	pid_init(&pid_x);
	pid_init(&pid_y);
	pid_init(&pid_angle);

	pid_set_gains(&pid_x, SETTING_PID_X_GAIN_P,
                        SETTING_PID_X_GAIN_I,
                        SETTING_PID_X_GAIN_D);
  pid_set_maximums(&pid_x, SETTING_PID_X_MAX_IN,
                           SETTING_PID_X_MAX_I,
                           SETTING_PID_X_MAX_OUT);
  pid_set_out_shift(&pid_x, SETTING_PID_X_SHIFT);
 
  pid_set_gains(&pid_y, SETTING_PID_Y_GAIN_P,
                        SETTING_PID_Y_GAIN_I,
                        SETTING_PID_Y_GAIN_D);
  pid_set_maximums(&pid_y, SETTING_PID_Y_MAX_IN,
                           SETTING_PID_Y_MAX_I,
                           SETTING_PID_Y_MAX_OUT);
  pid_set_out_shift(&pid_y, SETTING_PID_Y_SHIFT);
 
  pid_set_gains(&pid_angle, SETTING_PID_A_GAIN_P,
                            SETTING_PID_A_GAIN_I,
                            SETTING_PID_A_GAIN_D);
  pid_set_maximums(&pid_angle, SETTING_PID_A_MAX_IN,
                           SETTING_PID_A_MAX_I,
                           SETTING_PID_A_MAX_OUT);
  pid_set_out_shift(&pid_angle, SETTING_PID_A_SHIFT);
  
  // quadramp
  quadramp_set_1st_order_vars(&qramp_angle,
                                SETTING_QRAMP_A_SPEED, SETTING_QRAMP_A_SPEED);
  quadramp_set_2nd_order_vars(&qramp_angle,
                                SETTING_QRAMP_A_ACC, SETTING_QRAMP_A_ACC);
  
	// setup CSMs
	cs_init(&csm_x);
	cs_init(&csm_y);
	cs_init(&csm_angle);

	cs_set_consign_filter(&csm_x,     NULL, NULL); 
	cs_set_consign_filter(&csm_y,     NULL, NULL); 
  cs_set_consign_filter(&csm_angle, &quadramp_do_filter, &qramp_angle);

	cs_set_correct_filter(&csm_x,     &pid_do_filter, &pid_x);
	cs_set_correct_filter(&csm_y,     &pid_do_filter, &pid_y);
	cs_set_correct_filter(&csm_angle, &pid_do_filter, &pid_angle);

	cs_set_feedback_filter(&csm_x,     NULL, NULL);
	cs_set_feedback_filter(&csm_y,     NULL, NULL);
	cs_set_feedback_filter(&csm_angle, NULL, NULL);

	cs_set_process_out(&csm_x, &get_robot_x, rcs);
	cs_set_process_out(&csm_y, &get_robot_y, rcs);
	cs_set_process_out(&csm_angle, &get_robot_a, rcs);

	cs_set_process_in(&csm_x, NULL, NULL);
	cs_set_process_in(&csm_y, NULL, NULL);
	cs_set_process_in(&csm_angle, NULL, NULL);
}

void robot_cs_activate(robot_cs_t* rcs, uint8_t active)
{
  // must be performed on a interruption free environnement
  INTLVL_DISABLE_BLOCK(INTLVL_LO) {
    if(!active) { 
      // clear previous motors consign
      hrobot_set_motors(0, 0, 0);
    }
    else
      rcs->reactivated = 1;
    rcs->active = active;
  }
}

void robot_cs_set_hposition_manager(robot_cs_t* rcs,
                                     hrobot_position_t* hpm)
{
  rcs->hpm = hpm;
}


void robot_cs_set_hrobot_manager(robot_cs_t* rcs,
                                  hrobot_system_t* hrs)
{
  rcs->hrs = hrs;
}

void robot_cs_update(void* dummy)
{
  double vx_t,vy_t,omegaz_t;
  double vx_r,vy_r;
  double alpha;
  double _ca,_sa;

  hrobot_vector_t hvec;
	robot_cs_t *rcs = dummy;

  // if CS inactivated
  if(!rcs->active)
    return;

  // if CS was previously inactive, we need a little hack for quadramps
  if(rcs->reactivated)
  {
    int32_t consign;

    pid_reset(&pid_x);
    pid_reset(&pid_y);
    pid_reset(&pid_angle);

    consign = cs_get_consign(&csm_angle);
    qramp_angle.previous_var = 0;
    qramp_angle.previous_out = consign;
    qramp_angle.previous_in = consign;
       
    rcs->reactivated = 0;
  }

  // compute control system first level (x,y,a)
  cs_manage(&csm_x);
  cs_manage(&csm_y);
  cs_manage(&csm_angle);

  // transform output vector from table coords to robot coords
  vx_t     = cs_get_out(&csm_x);
  vy_t     = cs_get_out(&csm_y);
  omegaz_t = cs_get_out(&csm_angle);

  hposition_get(rcs->hpm, &hvec);

  alpha = -hvec.alpha;
 
  _ca = cos(alpha);
  _sa = sin(alpha);

  vx_r = vx_t*_ca - vy_t*_sa;
  vy_r = vx_t*_sa + vy_t*_ca;

  // set second level consigns
  hrobot_set_motors(vx_r, vy_r, omegaz_t);
                              
}

void robot_cs_set_xy_consigns( robot_cs_t* rcs,
											  	  		int32_t x,
												  	  	int32_t y)
{
  INTLVL_DISABLE_BLOCK(INTLVL_LO) {
    cs_set_consign(&csm_x,x);
    cs_set_consign(&csm_y,y);
  }
}

void robot_cs_set_a_consign( robot_cs_t* rcs,
		    											int32_t angle)
{
  INTLVL_DISABLE_BLOCK(INTLVL_LO) {
    cs_set_consign(&csm_angle,angle);
  }
}

int32_t get_robot_x(void* dummy)
{
  hrobot_vector_t hvec;
  robot_cs_t* rcs = dummy;

  hposition_get(rcs->hpm, &hvec);
 
  return hvec.x * RCS_MM_TO_CSUNIT;
}

int32_t get_robot_y(void* dummy)
{
  hrobot_vector_t hvec;
  robot_cs_t* rcs = dummy;

  hposition_get(rcs->hpm, &hvec);
  
  return (hvec.y * RCS_MM_TO_CSUNIT);
}

int32_t get_robot_a(void* dummy)
{
  hrobot_vector_t hvec;
  robot_cs_t* rcs = dummy;

  hposition_get(rcs->hpm, &hvec);

  return (hvec.alpha * RCS_RAD_TO_CSUNIT);
}
