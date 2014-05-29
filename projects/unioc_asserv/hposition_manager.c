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

/** \file hposition_manager.c
  * \author JD
  *
  * Manages holonomic robot encoders to compute robot position
  *
  */

#include <avarix.h>
#include <avarix/intlvl.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include "quadramp.h"
#include "pid.h"
#include "control_system_manager.h"
#include "settings.h"
#include "motor_encoders.h"
#include "robot_cs.h"
#include "adxrs/adxrs.h"

#include <stdio.h>

#include "hposition_manager.h"
#include "hposition_manager_config.h"

#include "telemetry.h"

// robot CSs
extern robot_cs_t robot_cs;
extern struct quadramp_filter qramp_angle;
extern struct cs csm_angle;
extern struct pid_filter pid_x;
extern struct pid_filter pid_y;
extern struct pid_filter pid_angle;

void hposition_init( hrobot_position_t* hpos )
{
  INTLVL_DISABLE_BLOCK(INTLVL_LO) {
    hpos->position.x = 0.0; 
    hpos->position.y = 0.0; 
    hpos->position.alpha = 0.0; 
  }
  hpos->firstUpdate = 1;
  return;
}

void hposition_set( hrobot_position_t* hpos, double x, double y, double alpha)
{
  INTLVL_DISABLE_BLOCK(INTLVL_LO) {
    hpos->position.x = x;
    hpos->position.y = y;
    hpos->position.alpha = alpha;
    adxrs_set_angle(alpha);

    // set actual position to RCSs
    robot_cs_set_xy_consigns( &robot_cs, RCS_MM_TO_CSUNIT*x,
                                       RCS_MM_TO_CSUNIT*y);
    robot_cs_set_a_consign( &robot_cs, RCS_RAD_TO_CSUNIT*alpha);

    // reset CSs
    pid_reset(&pid_x);
    pid_reset(&pid_y);
    pid_reset(&pid_angle);
 
    int32_t consign;
    consign = cs_get_consign(&csm_angle);
    qramp_angle.previous_var = 0;
    qramp_angle.previous_out = consign;
    qramp_angle.previous_in = consign;
  }

  return;
}

void hposition_get_xy( hrobot_position_t *hpos, vect_xy_t *pv )
{
  INTLVL_DISABLE_BLOCK(INTLVL_LO) {
    pv->x = hpos->position.x;
    pv->y = hpos->position.y;
  }
}

void hposition_get_a( hrobot_position_t *hpos, double *pa )
{
  INTLVL_DISABLE_BLOCK(INTLVL_LO) {
    *pa = hpos->position.alpha;
  }
}

void hposition_get( hrobot_position_t *hpos, hrobot_vector_t *hvec)
{
  INTLVL_DISABLE_BLOCK(INTLVL_LO) {
    *hvec = hpos->position;
  }
  return;
}

void hposition_update(void *dummy)
{
  uint8_t i,k;
  int32_t v;
  double dp[3];
  hrobot_vector_t vec;
  double _ca,_sa;
  hrobot_position_t* hpos  = dummy;
  
  int16_t *vectors  = NULL;
  int16_t *pvectors = NULL;
  double *matrix    = NULL;

  // access motor encoders values
  motor_encoders_update();
  vectors = motor_encoders_get_value();

  // first time update => update vector, quit
  if( hpos->firstUpdate )
  {
    for(i=0;i<3;i++)
      hpos->pvectors[i] = vectors[i];
    hpos->firstUpdate = 0;
    return;
  }
  // set vectors & matrix
  pvectors = hpos->pvectors;
  matrix = hrobot_motors_invmatrix;

  //----------------------------------------------------------
  // Transform speed in encoders coordinates to robot coordinates
  for(k=0;k<3;k++)
    dp[k] = 0.0;

  bool is_moving = false;
  // for each encoder coordinate
  for(i=0;i<3;i++)
  {
    // compute speed in encoder coordinates
    v = vectors[i] - pvectors[i];
    // update previous ADNS vectors
    pvectors[i] = vectors[i];
 
    if(v > 0)
      is_moving = true;

    // for each robot coordinate (x,y,a) compute a dx of mouvement
    for(k=0;k<3;k++)
      dp[k] += matrix[i+k*3]*v;
  }

  // scale units
  for(k=0;k<3;k++)
    dp[k] /= 1000;

  vec.alpha = hpos->position.alpha;

	// transform ADNS position to table  
  _ca = cos(vec.alpha);
  _sa = sin(vec.alpha);
 
  // put gyro in calibration mode
  // XXX NDJD : is_moving will be used... later XXX
  (void)is_moving;

  //--------------------------------------------------
  // Integrate speed in robot coordinates to position
  vec.x = hpos->position.x + dp[HROBOT_DX]*_ca - dp[HROBOT_DY]*_sa;
  vec.y = hpos->position.y + dp[HROBOT_DX]*_sa + dp[HROBOT_DY]*_ca;
	vec.alpha = adxrs_get_angle();

  //------------------------------------
  // Latch computed values to accessors
  INTLVL_DISABLE_BLOCK(INTLVL_LO) {
    hpos->position.x = vec.x;
    hpos->position.y = vec.y;
    hpos->position.alpha = vec.alpha;
  }

  // update telemetry
  TM_DL_XYA(hpos->position.x, hpos->position.y, 1000*hpos->position.alpha);

  return;
}

