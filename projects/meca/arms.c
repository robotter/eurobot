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
#include <avarix.h>
#include <avarix/portpin.h>
#include <ax12/ax12.h>
#include "config.h"
#include "stepper_motor.h"
#include <clock/clock.h>
#include <util/delay.h>
#include "arms.h"
#include <timer/uptime.h>
#include <i2c/i2c.h>

extern ax12_t ax12;

extern rome_intf_t rome_strat;

#define RETRY_AND_UPDATE(fcall) while(!(fcall)) { \
  portpin_outtgl(&LED_AN_PP(0)); \
  _delay_ms(10); \
  }

arm_t arm_l;
arm_t arm_r;

#define LEFT_SIDE true
#define RIGHT_SIDE false

//i2c interface with magichanism's mosboard
i2cm_t *const mosboard_i2c = i2cC;

uint8_t mosboard_word = 0x00;

void suck_left_atom(bool side,bool b){
  if (b)
    mosboard_word |= ARM_LEFT_VALVE(side);
  else
    mosboard_word &= ~(ARM_LEFT_VALVE(side));
}

void suck_center_atom(bool side,bool b){
  if (b)
    mosboard_word |= ARM_CENTER_VALVE(side);
  else
    mosboard_word &= ~(ARM_CENTER_VALVE(side));
}

void suck_right_atom(bool side,bool b){
  if (b)
    mosboard_word |= ARM_RIGHT_VALVE(side);
  else
    mosboard_word &= ~(ARM_RIGHT_VALVE(side));
}

void suck_all_atom(bool side,bool b){
  if (b)
    mosboard_word |= ARM_RIGHT_VALVE(side);
  else
    mosboard_word &= ~(ARM_RIGHT_VALVE(side));
}

void pump_power(bool side,bool b){
  if (b)
    mosboard_word |= ARM_PUMP(side);
  else
    mosboard_word &= ~(ARM_PUMP(side));
}

uint8_t mosboard_buffer[3];
void mosboard_update(void){
  return;
  //send frame to mosboard
  INTLVL_DISABLE_BLOCK(INTLVL_MED) {
    mosboard_buffer[0] = 0x01;
    mosboard_buffer[1] = mosboard_word;
    mosboard_buffer[2] = ~mosboard_word;
    i2cm_send(mosboard_i2c, 0x30, mosboard_buffer, 3);
  }
}

void arms_init(void){
  arm_l.tm_state = ROME_ENUM_MECA_STATE_BUSY;
  arm_r.tm_state = ROME_ENUM_MECA_STATE_BUSY;
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

bool is_arm_up(arm_t* arm){
  if (arm->side)
    return !(portpin_in(&LEFT_ELEVATOR_STOP_PIN));
  else
    return !(portpin_in(&RIGHT_ELEVATOR_STOP_PIN));
}

uint8_t arms_get_tm_state(void){
  if (arm_l.tm_state <= arm_r.tm_state)
    return arm_l.tm_state;
  else
    return arm_r.tm_state;
}

void arm_take_atoms(arm_t* arm){
  arm->tm_state = ROME_ENUM_MECA_STATE_BUSY;
  arm->state = ARM_TAKE_ATOMS;
}

void arm_release_atoms(arm_t* arm){
  arm->tm_state = ROME_ENUM_MECA_STATE_BUSY;
  arm->state = ARM_RELEASE_ATOMS;
}

void arm_elevator_up(arm_t* arm){
  arm->tm_state = ROME_ENUM_MECA_STATE_GROUND_CLEAR;
  arm->state = ARM_ELEVATOR_GO_UP;
}

void arm_elevator_down(arm_t* arm){
  arm->tm_state = ROME_ENUM_MECA_STATE_GROUND_CLEAR;
  arm->state = ARM_ELEVATOR_GO_DOWN;
}

void arm_set_idle(arm_t* arm){
  arm->tm_state = ROME_ENUM_MECA_STATE_READY;
  arm->state = ARM_IDLE;
}

void arm_update(arm_t* arm){
  if (is_arm_up(arm))
    portpin_outset(&LED_AN_PP(arm->side));
  else
    portpin_outclr(&LED_AN_PP(arm->side));

  #if 0
  static arm_state_t r_os = 1;
  static arm_state_t l_os = 1;
  static uint32_t l_lsct = 0;
  static uint32_t r_lsct = 0;
  if (arm->side){
    if (l_os != arm->state){
      //if state changed, the meca is doing something and isn't blocked
      //idle and init stages are the only exeptions
      if (arm->state != ARM_IDLE && arm->state != ARM_INIT)
        l_lsct = uptime_us();
    }
    l_os = arm->state;

    if(uptime_us() - l_lsct > MECA_TIMEOUT_US){
      ROME_LOGF(&rome_strat, DEBUG,"left arm state timeout");
      arm_set_idle(arm);
    }
  }
  else{
    if (r_os != arm->state){
      //if state changed, the meca is doing something and isn't blocked
      //idle and init stages are the only exeptions
      if (arm->state != ARM_IDLE && arm->state != ARM_INIT)
        r_lsct = uptime_us();
    }
    r_os = arm->state;

    if(uptime_us() - r_lsct > MECA_TIMEOUT_US){
      ROME_LOGF(&rome_strat, DEBUG,"left arm state timeout");
      arm_set_idle(arm);
    }
  }
  #endif

  switch (arm->state){
    case ARM_INIT:
      //turn off pumps and valves
      mosboard_word = 0;
      mosboard_update();
      //reset arm to top position if it isn't there yet
      arm->up = is_arm_up(arm);
      if (!arm->up) {
        stepper_motor_move(arm->side, 100);
        arm->state = ARM_ELEVATOR_WAIT_UP;
        break;
      }
      arm_set_idle(arm);
      break;

    case ARM_IDLE:
      break;

    case ARM_ELEVATOR_GO_UP:
      //go up and wait for elevator up
      stepper_motor_move(arm->side, 15000);
      arm->state = ARM_ELEVATOR_WAIT_UP;
      break;

    case ARM_ELEVATOR_WAIT_UP:
      if(stepper_motor_arrived(arm->side)){
        portpin_outtgl(&LED_AN_PP(3));
        if (is_arm_up(arm)){
          arm->state = ARM_ELEVATOR_WAIT_UP_DEBOUNCE;
          arm->debounce_time = uptime_us();
          break;
        }
        //motor should be up and doesn't, try to continue moving up
        stepper_motor_move(arm->side, 100);
        break;
      }
      break;

    case ARM_ELEVATOR_WAIT_UP_DEBOUNCE:
      if(uptime_us() - arm->debounce_time < 1000)
        break;

      if (is_arm_up(arm)) {
        arm->up = true;
        stepper_motor_move(arm->side, 0);
        arm_set_idle(arm);
      }
      else
        arm->state = ARM_ELEVATOR_WAIT_UP;
      break;

    case ARM_ELEVATOR_GO_DOWN:
      //go down and wait for elevator down
      stepper_motor_move(arm->side, -15000);
      arm->state = ARM_ELEVATOR_WAIT_DOWN;
      break;
    case ARM_ELEVATOR_WAIT_DOWN:
      if(stepper_motor_arrived(arm->side)){
        arm->up = false;
        stepper_motor_move(arm->side, 0);
        arm_set_idle(arm);
      }
      break;

    case ARM_TAKE_ATOMS:
      suck_left_atom(arm->side,true);
      suck_center_atom(arm->side,true);
      suck_right_atom(arm->side,true);
      pump_power(arm->side,true);
      mosboard_update();

      //check barometer to know if there are atoms
      arm->state = ARM_CHECK_SUCKERS;
      break;

    case ARM_RELEASE_ATOMS:
      pump_power(arm->side,false);
      suck_left_atom(arm->side,false);
      arm->atoms[0] = false;
      suck_center_atom(arm->side,false);
      arm->atoms[1] = false;
      suck_right_atom(arm->side,false);
      arm->atoms[2] = false;
      mosboard_update();
      arm_set_idle(arm);
      break;

    case ARM_CHECK_SUCKERS:
      if (barometer_get_pressure(&arm->baro) < BARO_VOID_PRESSURE){
        arm->atoms[0] = true;
        arm->atoms[1] = true;
        arm->atoms[2] = true;
        arm_set_idle(arm);
        break;
      }
      //if it failed, check suckers one by one
      suck_left_atom(&arm->side, true);
      suck_center_atom(&arm->side, false);
      suck_right_atom(&arm->side, false);
      mosboard_update();
      arm->state = ARM_CHECK_LEFT_SUCKER;
      break;

    case ARM_CHECK_LEFT_SUCKER:
      //update left sucker state and then check center sucker
      if (barometer_get_pressure(&arm->baro) < BARO_VOID_PRESSURE)
        arm->atoms[0] = true;
      else{
        arm->atoms[0] = false;
        suck_left_atom(&arm->side, false);
        mosboard_update();
      }
      arm->state = ARM_CHECK_CENTER_SUCKER;
      break;

    case ARM_CHECK_CENTER_SUCKER:
      //update center sucker state and then check right sucker
      if (barometer_get_pressure(&arm->baro) < BARO_VOID_PRESSURE)
        arm->atoms[1] = true;
      else{
        arm->atoms[1] = false;
        suck_center_atom(&arm->side, false);
        mosboard_update();
      }
      arm->state = ARM_CHECK_RIGHT_SUCKER;
      break;

    case ARM_CHECK_RIGHT_SUCKER:
      //update right sucker state and then it's over !
      if (barometer_get_pressure(&arm->baro) < BARO_VOID_PRESSURE)
        arm->atoms[2] = true;
      else{
        arm->atoms[2] = false;
        suck_right_atom(&arm->side, false);
        mosboard_update();
      }
      arm_set_idle(arm);
      break;

    default:
      break;
  }
}

void arms_shutdown(void){
  arm_release_atoms(&arm_l);
  arm_release_atoms(&arm_r);
  mosboard_update();
}
