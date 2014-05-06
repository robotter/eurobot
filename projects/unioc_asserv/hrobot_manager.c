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

/** \file hrobot_manager.c
  * \author JD
  *
  * Abstract robot hardware
  *
  */
#include <avarix.h>

#include <stdio.h>
#include <stdlib.h>
#include <avarix/portpin.h>

#include "cli.h"

#include "settings.h"

#include "robot_cs.h"
#include "hrobot_manager.h"
#include "hrobot_manager_config.h"

#include "motors_scales.h"

// hrobot system singleton
hrobot_system_t system;

static void _set_motorA_sign(bool sign) {
  if(sign)
    portpin_outset(system.signs+0);
  else
    portpin_outclr(system.signs+0);
}

static void _set_motorB_sign(bool sign) {
  if(sign)
    portpin_outset(system.signs+1);
  else
    portpin_outclr(system.signs+1);
}

static void _set_motorC_sign(bool sign) {
  if(sign)
    portpin_outset(system.signs+2);
  else
    portpin_outclr(system.signs+2);
}

void hrobot_init()
{
  // configure PWMs
#if defined(BUILD_GALIPEUR)
  pwm_motor_init(system.pwms+0, &TCF0, 'A',  _set_motorA_sign); 
  pwm_motor_init(system.pwms+1, &TCF0, 'B',  _set_motorB_sign); 
  pwm_motor_init(system.pwms+2, &TCF0, 'C',  _set_motorC_sign); 
#elif defined(BUILD_GALIPETTE)
  pwm_motor_init(system.pwms+0, &TCF0, 'B',  _set_motorA_sign); 
  pwm_motor_init(system.pwms+1, &TCF0, 'C',  _set_motorB_sign); 
  pwm_motor_init(system.pwms+2, &TCF0, 'A',  _set_motorC_sign); 
#else
# error "Please define either BUILD_GALIPEUR or BUILD_GALIPETTE"
#endif
  // configure frequency
  uint8_t it;
  for(it=0;it<3;it++)
    pwm_motor_set_frequency(system.pwms+it, SETTING_PWM_FREQUENCY_KHZ*1000);

  //set break as output
  PORTH.DIRSET = _BV(4)|_BV(5);
  PORTA.DIRSET = _BV(6);

  // configure break pps
#if defined(BUILD_GALIPEUR)
  system.breaks[0] = PORTPIN(H,4);
  system.breaks[1] = PORTPIN(H,5);
  system.breaks[2] = PORTPIN(A,6);
  // configure sign pps
  system.signs[0] = PORTPIN(H,1);
  system.signs[1] = PORTPIN(H,2);
  system.signs[2] = PORTPIN(H,3);
#elif defined(BUILD_GALIPETTE)
  system.breaks[0] = PORTPIN(H,5);
  system.breaks[1] = PORTPIN(A,6);
  system.breaks[2] = PORTPIN(H,4);
  // configure sign pps
  system.signs[0] = PORTPIN(H,2);
  system.signs[1] = PORTPIN(H,3);
  system.signs[2] = PORTPIN(H,1);
#else
# error "Please define either BUILD_GALIPEUR or BUILD_GALIPETTE"
#endif

  for(it=0; it<3; it++)
    portpin_dirset(system.signs+it);

  return;
}

void hrobot_break(uint8_t state)
{
  // for each of the 3 motors
  uint8_t it;
  for(it=0;it<3;it++) {
    if(state)
      portpin_outset(system.breaks+it);
    else
      portpin_outclr(system.breaks+it);
  }
}

void hrobot_set_motors(int32_t vx, int32_t vy, int32_t omega)
{
  uint8_t k,i;
  float dp[3];
  float v[3];

  v[0] = vx/(float)RCS_MM_TO_CSUNIT;
  v[1] = vy/(float)RCS_MM_TO_CSUNIT;
  v[2] = omega/(float)RCS_RAD_TO_CSUNIT;

  for(k=0;k<3;k++)
    dp[k] = 0.0;

  // compute consign in motors coordinates
  for(i=0;i<3;i++)
    for(k=0;k<3;k++)
      dp[k] += hrobot_motors_matrix[i+k*3]*v[i];

  // for each motor
  for(i=0;i<3;i++)
  {
    float v = dp[i] * motors_scales[i];
    if(v > 32767) v = 32767;
    if(v < -32768) v = -32768;
    pwm_motor_set(system.pwms+i, (int16_t)v);
  }
  
  return;
}
