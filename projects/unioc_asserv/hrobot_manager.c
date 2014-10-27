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
#include <avarix/portpin.h>

#include "cli.h"

#include "settings.h"

#include "robot_cs.h"
#include "hrobot_manager.h"
#include "hrobot_manager_config.h"

#include "scales.h"

#include "telemetry.h"

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
#if defined(GALIPEUR)
  pwm_motor_init(system.pwms+0, &TCF0, 'A',  _set_motorA_sign); 
  pwm_motor_init(system.pwms+1, &TCF0, 'B',  _set_motorB_sign); 
  pwm_motor_init(system.pwms+2, &TCF0, 'C',  _set_motorC_sign); 
#elif defined(GALIPETTE)
  pwm_motor_init(system.pwms+0, &TCF0, 'B',  _set_motorA_sign); 
  pwm_motor_init(system.pwms+1, &TCF0, 'C',  _set_motorB_sign); 
  pwm_motor_init(system.pwms+2, &TCF0, 'A',  _set_motorC_sign); 
#else
# error "Please define either GALIPEUR or GALIPETTE"
#endif
  // configure frequency
  uint8_t it;
  for(it=0;it<3;it++)
    pwm_motor_set_frequency(system.pwms+it, SETTING_PWM_FREQUENCY_KHZ*1000);

  //set break as output
  PORTH.DIRSET = _BV(4)|_BV(5);
  PORTA.DIRSET = _BV(6);

  // configure break pps
#if defined(GALIPEUR)
  system.breaks[0] = PORTPIN(H,4);
  system.breaks[1] = PORTPIN(H,5);
  system.breaks[2] = PORTPIN(A,6);
  // configure sign pps
  // XXX NDJD hack 26/05/2014
  #define HACKED_PORT PORTPIN(H,2)
  portpin_dirclr(&HACKED_PORT);
  system.signs[0] = PORTPIN(H,1);
  system.signs[1] = PORTPIN(H,0);
  system.signs[2] = PORTPIN(H,3);
#elif defined(GALIPETTE)
  system.breaks[0] = PORTPIN(H,5);
  system.breaks[1] = PORTPIN(A,6);
  system.breaks[2] = PORTPIN(H,4);
  // configure sign pps
  system.signs[0] = PORTPIN(H,2);
  system.signs[1] = PORTPIN(H,3);
  system.signs[2] = PORTPIN(H,1);
#else
# error "Please define either GALIPEUR or GALIPETTE"
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
  int16_t motors[3];

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
    float v = dp[i];
    if(v > 32767) v = 32767;
    if(v < -32768) v = -32768;
    v *= motors_scales[i];
    motors[i] = (int16_t)v;
    pwm_motor_set(system.pwms+i, (int16_t)v);
  }
  
  // update telemetry
  TM_DL_MOTORS(motors[0], motors[1], motors[2]);

  return;
}
