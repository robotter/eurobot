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
}

static bool is_arm_up(const arm_t *arm) {
  return !portpin_in(arm->side ? &LEFT_ARM_STOP_PIN : &RIGHT_ARM_STOP_PIN);
}

uint8_t arms_get_tm_state(void) {
  return MIN(arm_l.tm_state, arm_r.tm_state);
}

void arm_take_atoms(arm_t *arm) {
  if(arm->state != ARM_IDLE) {
    return;
  }
  arm->tm_state = ROME_ENUM_MECA_STATE_BUSY;
  arm->state = ARM_TAKE_ATOMS;
}

void arm_release_atoms(arm_t *arm) {
  if(arm->state != ARM_IDLE) {
    return;
  }
  arm->tm_state = ROME_ENUM_MECA_STATE_BUSY;
  arm->state = ARM_RELEASE_ATOMS;
}

void arm_elevator_move(arm_t *arm, uint16_t pos) {
  arm->tm_state = ROME_ENUM_MECA_STATE_GROUND_CLEAR;
  if(pos == 0) {
    arm->elevator.target = 0;
    arm->state = ARM_ELEVATOR_RESET;
  } else {
    arm->elevator.target = MAX(pos, arm->side ? LEFT_ARM_HEIGHT_STEPS : RIGHT_ARM_HEIGHT_STEPS);
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

// Step measure of barometer value, filter the result
static void arm_baro_measure_filter(arm_t *arm) {
  arm->pressure = 0.1 * barometer_get_pressure(&arm->baro) + 0.9 * arm->pressure;
}

// Initialize a measure of barometer value
static void arm_baro_measure_init(arm_t *arm) {
  arm->pressure = 0;
  arm->measure_end = uptime_us() + BARO_CHECK_TIME_US;
  arm_baro_measure_filter(arm);
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
      // move up by small steps until the limit switch is hit
      stepper_motor_move(arm->side, 100);
      if (is_arm_up(arm)) {
        // top position hit: got to the next state
        arm->state = ARM_ELEVATOR_RESET_DEBOUNCE;
        arm->measure_end = uptime_us() + 1000;
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
        // reset first if needed (safety check)
        arm->state = ARM_ELEVATOR_RESET;
      } else if (arm->elevator.target != arm->elevator.pos) {
        // already in position
        arm_set_idle(arm);
      } else {
        // move, then wait for elevator to be arrived
        // note: value is negative to move down, positive to move up
        stepper_motor_move(arm->side, (int16_t)(arm->elevator.pos - arm->elevator.target));
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

    // Switch on all the suckers to take atoms
    case ARM_TAKE_ATOMS:
      suckers_enable(arm, 1, 1, 1);
      ROME_LOGF(UART_STRAT, DEBUG, "MECA: pump on");
      // disable pump to check if there are atoms
      // note: this measure is only used for debug
      arm_baro_measure_init(arm);
      arm->state = ARM_CHECK_SUCKERS_DISABLE_PUMP;
      break;

    // Wait for pump to be off, for 1st pressure measure
    case ARM_CHECK_SUCKERS_DISABLE_PUMP:
      if (uptime_us() < arm->measure_end) {
        // measure not ready yet
        arm_baro_measure_filter(arm);
        ROME_LOGF(UART_STRAT, DEBUG, "MECA: pressure %u", arm->pressure);
      } else {
        // measure ready, check the suckers
        ROME_LOGF(UART_STRAT, DEBUG, "MECA: pump off");
        pump_disable(arm);
        arm_baro_measure_init(arm);
        arm->state = ARM_CHECK_SUCKERS_PRESSURE;
      }
      break;

    // Wait for pump to be off, for 1st pressure measure
    case ARM_CHECK_SUCKERS_PRESSURE:
      if (uptime_us() < arm->measure_end) {
        // measure not ready yet
        arm_baro_measure_filter(arm);
        ROME_LOGF(UART_STRAT, DEBUG, "MECA: pressure %u", arm->pressure);
      } else if (arm->pressure < BARO_VOID_PRESSURE) {
        // all atoms have been grabbed
        arm->atoms[0] = true;
        arm->atoms[1] = true;
        arm->atoms[2] = true;
        // reactivate the pump
        pump_enable(arm);
        arm_set_idle(arm);
      } else {
        // check suckers one by one, from left to right
        arm->atoms[0] = false;
        arm->atoms[1] = false;
        arm->atoms[2] = false;
        suckers_enable(arm, 1, 0, 0);
        // note: this measure is only used for debug
        arm_baro_measure_init(arm);
        arm->state = ARM_CHECK_LEFT_SUCKER_DISABLE_PUMP;
      }
      break;

    case ARM_CHECK_LEFT_SUCKER_DISABLE_PUMP:
      if (uptime_us() < arm->measure_end) {
        // measure not ready yet
        arm_baro_measure_filter(arm);
        ROME_LOGF(UART_STRAT, DEBUG, "MECA: pressure %u", arm->pressure);
      } else {
        // measure ready, check the suckers
        ROME_LOGF(UART_STRAT, DEBUG, "MECA: pump off");
        pump_disable(arm);
        arm_baro_measure_init(arm);
        arm->state = ARM_CHECK_LEFT_SUCKER_PRESSURE;
      }
      break;

    case ARM_CHECK_LEFT_SUCKER_PRESSURE:
      if (uptime_us() < arm->measure_end) {
        // measure not ready yet
        arm_baro_measure_filter(arm);
        ROME_LOGF(UART_STRAT, DEBUG, "MECA: pressure %u", arm->pressure);
      } else {
        // atom is grabbed if pressure is low
        arm->atoms[0] = arm->pressure < BARO_VOID_PRESSURE;
        // then, check center sucker
        suckers_enable(arm, arm->atoms[0], 1, 0);
        // note: this measure is only used for debug
        arm_baro_measure_init(arm);
        arm->state = ARM_CHECK_CENTER_SUCKER_DISABLE_PUMP;
      }
      break;

    case ARM_CHECK_CENTER_SUCKER_DISABLE_PUMP:
      if (uptime_us() < arm->measure_end) {
        // measure not ready yet
        arm_baro_measure_filter(arm);
        ROME_LOGF(UART_STRAT, DEBUG, "MECA: pressure %u", arm->pressure);
      } else {
        // measure ready, check the suckers
        ROME_LOGF(UART_STRAT, DEBUG, "MECA: pump off");
        pump_disable(arm);
        arm_baro_measure_init(arm);
        arm->state = ARM_CHECK_CENTER_SUCKER_PRESSURE;
      }
      break;

    case ARM_CHECK_CENTER_SUCKER_PRESSURE:
      if (uptime_us() < arm->measure_end) {
        // measure not ready yet
        arm_baro_measure_filter(arm);
        ROME_LOGF(UART_STRAT, DEBUG, "MECA: pressure %u", arm->pressure);
      } else {
        // atom is grabbed if pressure is low
        arm->atoms[1] = arm->pressure < BARO_VOID_PRESSURE;
        // then, check right sucker
        suckers_enable(arm, arm->atoms[0], arm->atoms[1], 1);
        // note: this measure is only used for debug
        arm_baro_measure_init(arm);
        arm->state = ARM_CHECK_RIGHT_SUCKER_DISABLE_PUMP;
      }
      break;

    case ARM_CHECK_RIGHT_SUCKER_DISABLE_PUMP:
      if (uptime_us() < arm->measure_end) {
        // measure not ready yet
        arm_baro_measure_filter(arm);
        ROME_LOGF(UART_STRAT, DEBUG, "MECA: pressure %u", arm->pressure);
      } else {
        // measure ready, check the suckers
        ROME_LOGF(UART_STRAT, DEBUG, "MECA: pump off");
        pump_disable(arm);
        arm_baro_measure_init(arm);
        arm->state = ARM_CHECK_RIGHT_SUCKER_PRESSURE;
      }
      break;

    case ARM_CHECK_RIGHT_SUCKER_PRESSURE:
      if (uptime_us() < arm->measure_end) {
        // measure not ready yet
        arm_baro_measure_filter(arm);
        ROME_LOGF(UART_STRAT, DEBUG, "MECA: pressure %u", arm->pressure);
      } else {
        // atom is grabbed if pressure is low
        arm->atoms[2] = arm->pressure < BARO_VOID_PRESSURE;
        // it's over, re-enable the pump (only if there are atoms)
        if (arm->atoms[0] || arm->atoms[1] || arm->atoms[2]) {
          suckers_enable(arm, arm->atoms[0], arm->atoms[1], arm->atoms[2]);
        } else {
          suckers_disable(arm);
        }
        arm_set_idle(arm);
      }
      break;

    // Switch off the suckers to release atoms
    case ARM_RELEASE_ATOMS:
      arm->atoms[0] = false;
      arm->atoms[1] = false;
      arm->atoms[2] = false;
      suckers_disable(arm);
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

