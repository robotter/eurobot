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

/** \file hrobot_manager.h
  * \author JD
  *
  * Abstract robot hardware
  *
  */

#ifndef _HROBOT_MANAGER_H_
#define _HROBOT_MANAGER_H_

#include <pwm/motor.h>
#include <avarix/portpin.h>

/**@brief Structure describing robot hardware
	*/
typedef struct
{
  // __ pwms ___
  // pwm for each of the 3 motors 
  pwm_motor_t pwms[3];
  // break PORT & PIN for each of the 3 motors
  portpin_t breaks[3];
  // sign port for each of the 3 motors
  portpin_t signs[3];

}hrobot_system_t;
// export hrobot system singleton
extern hrobot_system_t system;

/**@brief Initialize robot hardware
  */
void hrobot_init(void);

void hrobot_break(uint8_t brake );

/**@brief Set robot course and speed 
  *@param vx robot speed x coordinate
  *@param vy robot speed y coordinate
  *@param omega robot angular velocity
  */
void hrobot_set_motors(int32_t vx, int32_t vy, int32_t omega);

#endif/*_HROBOT_MANAGER_H_*/
