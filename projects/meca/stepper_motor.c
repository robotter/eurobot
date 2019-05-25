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
#include "stepper_motor.h"
#include "config.h"
#include <avarix/portpin.h>
#include <timer/timer.h>

typedef struct{
  int16_t consign;
  bool arrived;
}
stepper_motor_t;

static stepper_motor_t steppers[2];

void callback_left_motor(void) {
  INTLVL_DISABLE_BLOCK(INTLVL_MED) {
    if (steppers[1].consign == 0) {
      steppers[1].arrived = true;
      TIMER_CLEAR_CALLBACK(E1, 'A');
    }
    else {
      portpin_outtgl(&LEFT_MOTOR_STEP_PIN);
      if (steppers[1].consign > 0)
        steppers[1].consign ++;
      else
        steppers[1].consign --;
    }
  }
}

void callback_right_motor(void) {
  portpin_outtgl(&LED_AN_PP(3));
  INTLVL_DISABLE_BLOCK(INTLVL_MED) {
    if (steppers[0].consign == 0) {
      portpin_outtgl(&LED_AN_PP(2));
      steppers[0].arrived = true;
      TIMER_CLEAR_CALLBACK(E1, 'B');
    }
    else {
      portpin_outtgl(&RIGHT_MOTOR_STEP_PIN);
      if (steppers[0].consign > 0)
        steppers[0].consign --;
      else
        steppers[0].consign ++;
    }
  }
}

void stepper_motor_init(void) {
  //init pins
  portpin_dirset(&LEFT_MOTOR_STEP_PIN    );
  portpin_outclr(&LEFT_MOTOR_STEP_PIN    );

  portpin_dirset(&RIGHT_MOTOR_STEP_PIN   );
  portpin_outclr(&RIGHT_MOTOR_STEP_PIN   );

  portpin_dirset(&LEFT_ARM_MOTOR_DIR_PIN );
  portpin_outclr(&LEFT_ARM_MOTOR_DIR_PIN );

  portpin_dirset(&RIGHT_ARM_MOTOR_DIR_PIN);
  portpin_outclr(&RIGHT_ARM_MOTOR_DIR_PIN);

  portpin_dirset(&LEFT_ARM_MOTOR_EN_PIN  );
  portpin_outclr(&LEFT_ARM_MOTOR_EN_PIN  );

  portpin_dirset(&RIGHT_ARM_MOTOR_EN_PIN );
  portpin_outclr(&RIGHT_ARM_MOTOR_EN_PIN );

  steppers[0].arrived = false;
  steppers[1].arrived = false;
}

void stepper_motor_move(bool side, int16_t value) {
  INTLVL_DISABLE_BLOCK(INTLVL_MED) {
    steppers[side].consign = 2*value;
  }
  steppers[side].arrived = false;

  if (value == 0){
    steppers[side].arrived = true;
    return;
  }

  if (side){
    if (value > 0)
      portpin_outclr(&LEFT_ARM_MOTOR_DIR_PIN );
    else
      portpin_outset(&LEFT_ARM_MOTOR_DIR_PIN );
    portpin_outset(&LEFT_ARM_MOTOR_EN_PIN  );
    TIMER_SET_CALLBACK_US(E1, 'A', 100, INTLVL_MED, callback_left_motor);
  }
  else {
    if (value > 0)
      portpin_outclr(&RIGHT_ARM_MOTOR_DIR_PIN );
    else
      portpin_outset(&RIGHT_ARM_MOTOR_DIR_PIN );
    portpin_outset(&RIGHT_ARM_MOTOR_EN_PIN  );
    TIMER_SET_CALLBACK_US(E1, 'B', 100, INTLVL_MED, callback_right_motor);
  }
}

bool stepper_motor_arrived(bool side) {
  return steppers[side].arrived;
}

void stepper_motor_shutdown(bool side) {
  if (side) {
    portpin_outclr(&LEFT_ARM_MOTOR_EN_PIN  );
    TIMER_CLEAR_CALLBACK(E1, 'A');
  }
  else {
    portpin_outclr(&RIGHT_ARM_MOTOR_EN_PIN  );
    TIMER_CLEAR_CALLBACK(E1, 'B');
  }
}
