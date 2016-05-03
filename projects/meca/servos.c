/*  
 *  Copyright RobOtter (2016)
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

#include <avarix.h>
#include <pwm/motor.h>
#include "servos.h"
pwm_motor_t servos[4];

void servos_init (void){
  pwm_servo_init(servos,   (TC0_t *) &TCD0, 'A');
  pwm_servo_init(servos+1, (TC0_t *) &TCD0, 'B');
  pwm_servo_init(servos+2, (TC0_t *) &TCD0, 'C');
  pwm_servo_init(servos+3, (TC0_t *) &TCC1, 'A');

  pwm_motor_set_frequency(servos,   50u);
  pwm_motor_set_frequency(servos+1, 50u);
  pwm_motor_set_frequency(servos+2, 50u);
  pwm_motor_set_frequency(servos+2, 50u);
}

void servo_set(uint8_t id, uint16_t value){
  uint16_t pwm = (value < PWM_MOTOR_MAX) ? value : PWM_MOTOR_MAX;
  switch(id){
    case 0:
      pwm_motor_set(servos, pwm);
      break;
    case 1:
      pwm_motor_set(servos+1, pwm);
      break;
    default:
      break;
   }
}
  
