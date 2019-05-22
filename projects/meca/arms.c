#include <avr/io.h>
#include <avarix.h>
#include <avarix/portpin.h>
#include <ax12/ax12.h>
#include "config.h"
#include "servos.h"
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
i2cm_t mosboard_i2c;
uint8_t mosboard_word;

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

void mosboard_update(void){
 //send frame to mosboard
 uint8_t buffer[3];
 buffer[0] = 0x01;
 buffer[1] = mosboard_word;
 buffer[1] = ~mosboard_word;
 i2cm_send(&mosboard_i2c, 0x30, buffer, 3);
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

  portpin_dirclr(&LEFT_MOTOR_STOP_PIN);
  portpin_dirclr(&RIGHT_MOTOR_STOP_PIN);
  arms_shutdown();
}

bool is_arm_up(arm_t* arm){
  if (arm->side)
    return portpin_in(&LEFT_MOTOR_STOP_PIN);
  else
    return portpin_in(&RIGHT_MOTOR_STOP_PIN);
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

  switch (arm->state){
    case ARM_INIT:
      //reset arm to top position if it isn't there yet
      arm->up = is_arm_up(arm);
      if (!arm->up)
        arm->state = ARM_ELEVATOR_GO_UP;
      break;

    case ARM_IDLE:
      break;

    case ARM_ELEVATOR_GO_UP:
      //go up and wait for elevator up
      arm->state = ARM_ELEVATOR_WAIT_UP;
      break;
    case ARM_ELEVATOR_WAIT_UP:
      if (is_arm_up(arm)){
        arm->up = true;
        arm_set_idle(arm);
      }
      break;

    case ARM_ELEVATOR_GO_DOWN:
      //go down and wait for elevator down
      arm->state = ARM_ELEVATOR_WAIT_DOWN;
      break;
    case ARM_ELEVATOR_WAIT_DOWN:
      if(false){
        arm->up = false;
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

  mosboard_update();
}

void arms_shutdown(void){
  arm_release_atoms(&arm_l);
  arm_release_atoms(&arm_r);
  mosboard_update();
}
