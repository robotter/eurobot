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
#include <avr/io.h>
#include <clock/clock.h>
#include <util/delay.h>
#include <avarix.h>
#include <avarix/portpin.h>
#include <i2c/i2c.h>
#include <timer/uptime.h>
#include "arms.h"
#include "config.h"
#include "stepper_motor.h"
#include "servo_hat.h"

arm_t arm_l;
arm_t arm_r;

_Static_assert(LEFT_ARM_HEIGHT_STEPS < INT16_MAX, "left arm height in steps is too height");
_Static_assert(RIGHT_ARM_HEIGHT_STEPS < INT16_MAX, "right arm height in steps is too height");

// i2c interface with magichanism's mosboard
i2cm_t *const mosboard_i2c = i2cC;

static uint8_t mosboard_word = 0x00;

/// Update mosboard state from mosboard_word
void mosboard_update(void) {
  ROME_LOGF(UART_STRAT, DEBUG, "MECA: mosboard %x",mosboard_word);
  // send frame to mosboard
  INTLVL_DISABLE_BLOCK(INTLVL_MED) {
    uint8_t buffer[3] = {0x01, mosboard_word, ~mosboard_word};
    i2cm_send(mosboard_i2c, 0x30, buffer, ARRAY_LEN(buffer));
  }
}

void set_suckers_state(uint8_t state, uint8_t mask) {
  mosboard_word = (mosboard_word & ~mask) | state;
  mosboard_update();
}

#define ARM_STATE_MASK(x)  (ARM_PUMP(x) | ARM_LEFT_VALVE(x) | ARM_CENTER_VALVE(x) | ARM_RIGHT_VALVE(x))

/// Enable the pump and given suckers
#define suckers_enable(arm, left, center, right) \
    set_suckers_state( \
      ARM_PUMP((arm)->side) | \
      ((left) ? ARM_LEFT_VALVE((arm)->side) : 0) | \
      ((center) ? ARM_CENTER_VALVE((arm)->side) : 0) | \
      ((right) ? ARM_RIGHT_VALVE((arm)->side) : 0), \
      ARM_STATE_MASK((arm)->side))

/// Disable the pump and all the suckers
#define suckers_disable(arm) \
    set_suckers_state(0, ARM_STATE_MASK((arm)->side))

/// Disable the pump, don't change suckers
#define pump_disable(arm) \
    set_suckers_state(0, ARM_PUMP((arm)->side))

/// Disable the pump, don't change suckers
#define pump_enable(arm) \
    set_suckers_state(0xff, ARM_PUMP((arm)->side))


void arms_init(void) {
  arm_l.side = true;
  arm_r.side = false;
  arm_l.tm_state = ROME_ENUM_MECA_STATE_BUSY;
  arm_r.tm_state = ROME_ENUM_MECA_STATE_BUSY;
  arm_l.state = ARM_INIT;
  arm_r.state = ARM_INIT;
  arm_l.elevator.target = 0;
  arm_r.elevator.target = 0;
  arm_l.elevator.pos_known = false;
  arm_r.elevator.pos_known = false;

  barometer_init(&arm_l.baro, &ARM_BAROMETER_ADC, (LEFT_ARM_BAROMETER_ADC_PIN) << ADC_CH_MUXPOS_gp);
  barometer_init(&arm_r.baro, &ARM_BAROMETER_ADC, (RIGHT_ARM_BAROMETER_ADC_PIN) << ADC_CH_MUXPOS_gp);

  portpin_dirclr(&LEFT_ARM_STOP_PIN);
  PORTPIN_CTRL(&LEFT_ARM_STOP_PIN) = PORT_OPC_PULLUP_gc;

  portpin_dirclr(&RIGHT_ARM_STOP_PIN);
  PORTPIN_CTRL(&RIGHT_ARM_STOP_PIN) = PORT_OPC_PULLUP_gc;

  stepper_motor_init();

  servo_hat_init();
  //TODO replace channels by defines
  servo_hat_configure_channel(0, 102, 512);
  servo_hat_configure_channel(1, 102, 512);
  servo_hat_configure_channel(2, 102, 512);
  servo_hat_configure_channel(3, 102, 512);
  servo_hat_configure_channel(4, 102, 512);
  servo_hat_configure_channel(5, 102, 512);
  servo_hat_configure_channel(6, 102, 512);
  servo_hat_configure_channel(7, 102, 512);
  servo_hat_configure_channel(8, 102, 512);
  servo_hat_configure_channel(9, 102, 512);
  servo_hat_configure_channel(10, 102, 512);
  servo_hat_configure_channel(11, 102, 512);
  servo_hat_set_pwm(0, LEFT_ARM_LEFT_SERVO_DEPLOY_PWM_RATIO);
  servo_hat_set_pwm(1, LEFT_ARM_LEFT_SERVO_DEPLOY_PWM_RATIO);
  servo_hat_set_pwm(2, ARM_MAGNET_RELEASE_PWM_RATIO);
  servo_hat_set_pwm(3, ARM_MAGNET_RELEASE_PWM_RATIO);
  servo_hat_set_pwm(4, ARM_MAGNET_RELEASE_PWM_RATIO);
  servo_hat_set_pwm(5, ARM_MAGNET_RELEASE_PWM_RATIO);
  servo_hat_set_pwm(6, RIGHT_ARM_LEFT_SERVO_DEPLOY_PWM_RATIO);
  servo_hat_set_pwm(7, RIGHT_ARM_LEFT_SERVO_DEPLOY_PWM_RATIO);
  servo_hat_set_pwm(8, ARM_MAGNET_RELEASE_PWM_RATIO);
  servo_hat_set_pwm(9, ARM_MAGNET_RELEASE_PWM_RATIO);
  servo_hat_set_pwm(10,ARM_MAGNET_RELEASE_PWM_RATIO);
  servo_hat_set_pwm(11,ARM_MAGNET_RELEASE_PWM_RATIO);
}

static bool is_arm_up(const arm_t *arm) {
  return !portpin_in(arm->side ? &LEFT_ARM_STOP_PIN : &RIGHT_ARM_STOP_PIN);
}

uint8_t arms_get_tm_state(void) {
  //LEFT ARM doesn't work, so do not send its status.
  //return MIN(arm_l.tm_state, arm_r.tm_state);
  return arm_r.tm_state;
}

void arm_elevator_move(arm_t *arm, uint16_t pos) {
  ROME_LOGF(UART_STRAT, DEBUG, "MECA: %s move arm to %u", SIDE_NAME(arm->side), pos);
  arm->tm_state = ROME_ENUM_MECA_STATE_GROUND_CLEAR;
  if(pos == 0) {
    arm->elevator.target = 0;
    arm->state = ARM_ELEVATOR_RESET;
  } else {
    arm->elevator.target = MIN(pos, arm->side ? LEFT_ARM_HEIGHT_STEPS : RIGHT_ARM_HEIGHT_STEPS);
    arm->state = ARM_ELEVATOR_MOVE;
  }
}

void arm_elevator_shutdown(arm_t *arm) {
  stepper_motor_shutdown(arm->side);
  arm->elevator.pos_known = false;
}

static void arm_set_idle(arm_t *arm) {
  arm->tm_state = ROME_ENUM_MECA_STATE_READY;
  arm->state = ARM_IDLE;
}

void arm_update(arm_t *arm) {

  if (is_arm_up(arm))
    portpin_outset(&LED_AN_PP(arm->side));
  else
    portpin_outclr(&LED_AN_PP(arm->side));

  switch (arm->state) {
    // Switch off pump and suckers, then move to the top
    case ARM_INIT:
      // turn off pumps and valves
      mosboard_word = 0;
      mosboard_update();
      // move the arm
      arm_elevator_move(arm, 0);
      break;

    // Nothing to do
    case ARM_IDLE:
      break;

    // Move up to reset the position
    case ARM_ELEVATOR_RESET:
      if (!stepper_motor_arrived(arm->side)) {
        // wait
      } else if (is_arm_up(arm)) {
        ROME_LOGF(UART_STRAT, DEBUG, "MECA: %s, reset debounce", SIDE_NAME(arm->side));
        // top position hit: got to the next state
        arm->state = ARM_ELEVATOR_RESET_DEBOUNCE;
        arm->measure_end = uptime_us() + 1000;
      } else {
        // move up by small steps until the limit switch is hit
        stepper_motor_move(arm->side, 100);
      }
      break;

    // Check the switch during "debounce" period
    case ARM_ELEVATOR_RESET_DEBOUNCE:
      if (uptime_us() >= arm->measure_end) {
        // state confirmed
        stepper_motor_move(arm->side, 0);
        arm->elevator.pos_known = true;
        arm->elevator.pos = 0;
        if (arm->elevator.target != arm->elevator.pos) {
          // a move order is pending, execute it
          arm->state = ARM_ELEVATOR_MOVE;
        } else {
          arm_set_idle(arm);
        }
      } else if (!is_arm_up(arm)) {
        // state change, move up again
        arm->state = ARM_ELEVATOR_RESET;
      }
      break;

    // Move elevator to target position
    case ARM_ELEVATOR_MOVE:
      if (!arm->elevator.pos_known) {
        ROME_LOGF(UART_STRAT, DEBUG, "MECA: %s reset needed before moving", SIDE_NAME(arm->side));
        // reset first if needed (safety check)
        arm->state = ARM_ELEVATOR_RESET;
      } else if (arm->elevator.target == arm->elevator.pos) {
        // already in position
        arm_set_idle(arm);
      } else {
        // move, then wait for elevator to be arrived
        // note: value is negative to move down, positive to move up
        const int16_t steps = (int16_t)(arm->elevator.pos - arm->elevator.target);
        ROME_LOGF(UART_STRAT, DEBUG, "MECA: %s step motor: %d steps", SIDE_NAME(arm->side), steps);
        stepper_motor_move(arm->side, steps);
        arm->state = ARM_ELEVATOR_MOVE_WAIT;
      }
      break;

    // Elevator is moving, wait for it
    case ARM_ELEVATOR_MOVE_WAIT:
      if(stepper_motor_arrived(arm->side)) {
        stepper_motor_move(arm->side, 0);
        arm->elevator.pos = arm->elevator.target;
        arm_set_idle(arm);
      }
      break;

    default:
      break;
  }
}

void arm_grab_wings(arm_t *arm)
{
    uint8_t left_servo_hat_chan = arm->side ? LEFT_ARM_LEFT_SERVO_ID : RIGHT_ARM_LEFT_SERVO_ID;
    uint16_t left_servo_hat_value = arm->side ? LEFT_ARM_LEFT_SERVO_GRAB_PWM_RATIO : RIGHT_ARM_LEFT_SERVO_GRAB_PWM_RATIO;
    uint8_t right_servo_hat_chan = arm->side ? LEFT_ARM_RIGHT_SERVO_ID : RIGHT_ARM_RIGHT_SERVO_ID;
    uint16_t right_servo_hat_value = arm->side ? LEFT_ARM_RIGHT_SERVO_GRAB_PWM_RATIO : RIGHT_ARM_RIGHT_SERVO_GRAB_PWM_RATIO;
    servo_hat_set_pwm(left_servo_hat_chan, left_servo_hat_value);
    servo_hat_set_pwm(right_servo_hat_chan, right_servo_hat_value);
}

void arm_deploy_wings(arm_t *arm)
{
      ROME_LOGF(UART_STRAT, DEBUG, "MECA: fold wings");
    uint8_t left_servo_hat_chan = arm->side ? LEFT_ARM_LEFT_SERVO_ID : RIGHT_ARM_LEFT_SERVO_ID;
    uint16_t left_servo_hat_value = arm->side ? LEFT_ARM_LEFT_SERVO_DEPLOY_PWM_RATIO : RIGHT_ARM_LEFT_SERVO_DEPLOY_PWM_RATIO;
    uint8_t right_servo_hat_chan = arm->side ? LEFT_ARM_RIGHT_SERVO_ID : RIGHT_ARM_RIGHT_SERVO_ID;
    uint16_t right_servo_hat_value = arm->side ? LEFT_ARM_RIGHT_SERVO_DEPLOY_PWM_RATIO : RIGHT_ARM_RIGHT_SERVO_DEPLOY_PWM_RATIO;
    servo_hat_set_pwm(left_servo_hat_chan, left_servo_hat_value);
    servo_hat_set_pwm(right_servo_hat_chan, right_servo_hat_value);
}

void arm_fold_wings(arm_t *arm)
{
      ROME_LOGF(UART_STRAT, DEBUG, "MECA: deploy wings");
    uint8_t left_servo_hat_chan = arm->side ? LEFT_ARM_LEFT_SERVO_ID : RIGHT_ARM_LEFT_SERVO_ID;
    uint16_t left_servo_hat_value = arm->side ? LEFT_ARM_LEFT_SERVO_FOLD_PWM_RATIO : RIGHT_ARM_LEFT_SERVO_FOLD_PWM_RATIO;
    uint8_t right_servo_hat_chan = arm->side ? LEFT_ARM_RIGHT_SERVO_ID : RIGHT_ARM_RIGHT_SERVO_ID;
    uint16_t right_servo_hat_value = arm->side ? LEFT_ARM_RIGHT_SERVO_FOLD_PWM_RATIO : RIGHT_ARM_RIGHT_SERVO_FOLD_PWM_RATIO;
    servo_hat_set_pwm(left_servo_hat_chan, left_servo_hat_value);
    servo_hat_set_pwm(right_servo_hat_chan, right_servo_hat_value);
}

void arm_take_cans(arm_t *arm) {
      ROME_LOGF(UART_STRAT, DEBUG, "MECA: take cans");
      servo_hat_set_pwm(arm->side ? LEFT_ARM_MAGNET_SERVO_0_ID : RIGHT_ARM_MAGNET_SERVO_0_ID, ARM_MAGNET_TAKE_PWM_RATIO);
      servo_hat_set_pwm(arm->side ? LEFT_ARM_MAGNET_SERVO_1_ID : RIGHT_ARM_MAGNET_SERVO_1_ID, ARM_MAGNET_TAKE_PWM_RATIO);
      servo_hat_set_pwm(arm->side ? LEFT_ARM_MAGNET_SERVO_2_ID : RIGHT_ARM_MAGNET_SERVO_2_ID, ARM_MAGNET_TAKE_PWM_RATIO);
      servo_hat_set_pwm(arm->side ? LEFT_ARM_MAGNET_SERVO_3_ID : RIGHT_ARM_MAGNET_SERVO_3_ID, ARM_MAGNET_TAKE_PWM_RATIO);
}

void arm_release_cans(arm_t *arm) {
      ROME_LOGF(UART_STRAT, DEBUG, "MECA: release cans");
      servo_hat_set_pwm(arm->side ? LEFT_ARM_MAGNET_SERVO_0_ID : RIGHT_ARM_MAGNET_SERVO_0_ID, ARM_MAGNET_RELEASE_PWM_RATIO);
      servo_hat_set_pwm(arm->side ? LEFT_ARM_MAGNET_SERVO_1_ID : RIGHT_ARM_MAGNET_SERVO_1_ID, ARM_MAGNET_RELEASE_PWM_RATIO);
      servo_hat_set_pwm(arm->side ? LEFT_ARM_MAGNET_SERVO_2_ID : RIGHT_ARM_MAGNET_SERVO_2_ID, ARM_MAGNET_RELEASE_PWM_RATIO);
      servo_hat_set_pwm(arm->side ? LEFT_ARM_MAGNET_SERVO_3_ID : RIGHT_ARM_MAGNET_SERVO_3_ID, ARM_MAGNET_RELEASE_PWM_RATIO);
}


void arms_shutdown(void) {
  // disable everything
  mosboard_word = 0;
  mosboard_update();
}

