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

arm_t arm_l;
arm_t arm_r;

#define LEFT_SIDE true
#define RIGHT_SIDE false

// i2c interface with magichanism's mosboard
i2cm_t *const mosboard_i2c = i2cC;

static uint8_t mosboard_word = 0x00;

/// Update mosboard state from mosboard_word
void mosboard_update(void) {
  return;
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

/// Return true if the arm's vacuum is making void
#define VACUUM_OK(arm) \
    (barometer_get_pressure(&(arm)->baro) < BARO_VOID_PRESSURE)



void arms_init(void) {
  arm_l.tm_state = ROME_ENUM_MECA_STATE_BUSY;
  arm_r.tm_state = ROME_ENUM_MECA_STATE_BUSY;
  arm_r.elevator = ROME_ENUM_MECA_ELEVATOR_MOVING;
  arm_l.elevator = ROME_ENUM_MECA_ELEVATOR_MOVING;
  arm_l.state = ARM_INIT;
  arm_r.state = ARM_INIT;
  arm_l.side = LEFT_SIDE;
  arm_r.side = RIGHT_SIDE;
  // left arm, barometer ADCA4
  barometer_init(&arm_l.baro, &ADCA, ADC_CH_MUXPOS_PIN4_gc);
  // right arm, barometer ADCA5
  barometer_init(&arm_r.baro, &ADCA, ADC_CH_MUXPOS_PIN5_gc);

  portpin_dirclr(&LEFT_ELEVATOR_STOP_PIN);
  PORTPIN_CTRL(&LEFT_ELEVATOR_STOP_PIN) = PORT_OPC_PULLUP_gc;

  portpin_dirclr(&RIGHT_ELEVATOR_STOP_PIN);
  PORTPIN_CTRL(&RIGHT_ELEVATOR_STOP_PIN) = PORT_OPC_PULLUP_gc;

  stepper_motor_init();
}

bool is_arm_up(arm_t *arm) {
  if (arm->side)
    return !portpin_in(&LEFT_ELEVATOR_STOP_PIN);
  else
    return !portpin_in(&RIGHT_ELEVATOR_STOP_PIN);
}

uint8_t arms_get_tm_state(void) {
  return MIN(arm_l.tm_state, arm_r.tm_state);
}

void arm_take_atoms(arm_t *arm) {
  arm->tm_state = ROME_ENUM_MECA_STATE_BUSY;
  arm->state = ARM_TAKE_ATOMS;
}

void arm_release_atoms(arm_t *arm) {
  arm->tm_state = ROME_ENUM_MECA_STATE_BUSY;
  arm->state = ARM_RELEASE_ATOMS;
}

void arm_elevator_up(arm_t *arm) {
  arm->tm_state = ROME_ENUM_MECA_STATE_GROUND_CLEAR;
  arm->state = ARM_ELEVATOR_GO_UP;
}

void arm_elevator_down(arm_t *arm) {
  arm->tm_state = ROME_ENUM_MECA_STATE_GROUND_CLEAR;
  arm->state = ARM_ELEVATOR_GO_DOWN;
}

void arm_set_idle(arm_t *arm) {
  arm->tm_state = ROME_ENUM_MECA_STATE_READY;
  arm->state = ARM_IDLE;
}

void arm_update(arm_t *arm) {
  if (is_arm_up(arm))
    portpin_outset(&LED_AN_PP(arm->side));
  else
    portpin_outclr(&LED_AN_PP(arm->side));

  #if 0
  static arm_state_t r_os = 1;
  static arm_state_t l_os = 1;
  static uint32_t l_lsct = 0;
  static uint32_t r_lsct = 0;
  if (arm->side) {
    if (l_os != arm->state) {
      // if state changed, the meca is doing something and isn't blocked
      // idle and init stages are the only exeptions
      if (arm->state != ARM_IDLE && arm->state != ARM_INIT)
        l_lsct = uptime_us();
    }
    l_os = arm->state;

    if(uptime_us() - l_lsct > MECA_TIMEOUT_US) {
      ROME_LOGF(UART_STRAT, DEBUG, "left arm state timeout");
      arm_set_idle(arm);
    }
  }
  else{
    if (r_os != arm->state) {
      // if state changed, the meca is doing something and isn't blocked
      // idle and init stages are the only exeptions
      if (arm->state != ARM_IDLE && arm->state != ARM_INIT)
        r_lsct = uptime_us();
    }
    r_os = arm->state;

    if(uptime_us() - r_lsct > MECA_TIMEOUT_US) {
      ROME_LOGF(UART_STRAT, DEBUG, "left arm state timeout");
      arm_set_idle(arm);
    }
  }
  #endif

  switch (arm->state) {
    case ARM_INIT:
      // turn off pumps and valves
      mosboard_word = 0;
      mosboard_update();
      // reset arm to top position if it isn't there yet
      if (is_arm_up(arm)) {
        stepper_motor_move(arm->side, 100);
        arm->state = ARM_ELEVATOR_WAIT_UP;
        break;
      }
      arm_r.elevator = ROME_ENUM_MECA_ELEVATOR_UP;
      arm_set_idle(arm);
      break;

    case ARM_IDLE:
      break;

    case ARM_ELEVATOR_GO_UP:
      arm_r.elevator = ROME_ENUM_MECA_ELEVATOR_MOVING;
      // go up and wait for elevator up
      stepper_motor_move(arm->side, 15000);
      arm->state = ARM_ELEVATOR_WAIT_UP;
      break;

    case ARM_ELEVATOR_WAIT_UP:
      if(stepper_motor_arrived(arm->side)) {
        if (is_arm_up(arm)) {
          arm->state = ARM_ELEVATOR_WAIT_UP_DEBOUNCE;
          arm->debounce_time = uptime_us();
          break;
        }
        // motor should be up and doesn't, try to continue moving up
        stepper_motor_move(arm->side, 100);
        break;
      }
      break;

    case ARM_ELEVATOR_WAIT_UP_DEBOUNCE:
      if(uptime_us() - arm->debounce_time < 1000)
        break;

      if (is_arm_up(arm)) {
        arm_r.elevator = ROME_ENUM_MECA_ELEVATOR_UP;
        stepper_motor_move(arm->side, 0);
        arm_set_idle(arm);
      } else {
        arm->state = ARM_ELEVATOR_WAIT_UP;
      }
      break;

    case ARM_ELEVATOR_GO_DOWN:
      arm_r.elevator = ROME_ENUM_MECA_ELEVATOR_MOVING;
      // go down and wait for elevator down
      stepper_motor_move(arm->side, -15000);
      arm->state = ARM_ELEVATOR_WAIT_DOWN;
      break;

    case ARM_ELEVATOR_WAIT_DOWN:
      if(stepper_motor_arrived(arm->side)) {
        arm_r.elevator = ROME_ENUM_MECA_ELEVATOR_DOWN;
        stepper_motor_move(arm->side, 0);
        arm_set_idle(arm);
      }
      break;

    case ARM_TAKE_ATOMS:
      suckers_enable(arm, 1, 1, 1);
      // check barometer to know if there are atoms
      arm->state = ARM_CHECK_SUCKERS;
      break;

    case ARM_RELEASE_ATOMS:
      arm->atoms[0] = false;
      arm->atoms[1] = false;
      arm->atoms[2] = false;
      suckers_disable(arm);
      arm_set_idle(arm);
      break;

    case ARM_CHECK_SUCKERS:
      if (VACUUM_OK(arm)) {
        // all atoms have been grabbed
        arm->atoms[0] = true;
        arm->atoms[1] = true;
        arm->atoms[2] = true;
        arm_set_idle(arm);
        break;
      }
      // if it failed, check suckers one by one
      // start with the left one
      suckers_enable(arm, 1, 0, 0);
      arm->state = ARM_CHECK_LEFT_SUCKER;
      break;

    case ARM_CHECK_LEFT_SUCKER:
      // update left sucker state
      arm->atoms[0] = VACUUM_OK(arm);
      // then check center sucker
      suckers_enable(arm, arm->atoms[0], 1, 0);
      arm->state = ARM_CHECK_CENTER_SUCKER;
      break;

    case ARM_CHECK_CENTER_SUCKER:
      // update center sucker state
      arm->atoms[1] = VACUUM_OK(arm);
      // then check right sucker
      suckers_enable(arm, arm->atoms[0], arm->atoms[1], 1);
      arm->state = ARM_CHECK_RIGHT_SUCKER;
      break;

    case ARM_CHECK_RIGHT_SUCKER:
      // update right sucker state
      arm->atoms[2] = VACUUM_OK(arm);
      // then it's over
      suckers_enable(arm, arm->atoms[0], arm->atoms[1], arm->atoms[2]);
      arm_set_idle(arm);
      break;

    default:
      break;
  }
}

void arms_shutdown(void) {
  // disable everything
  mosboard_word = 0;
  mosboard_update();
}

