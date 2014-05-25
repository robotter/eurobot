#include "arm.h"
#include "pid.h"
#include "control_system_manager.h"
#include "ramp.h"
#include "control_system_manager.h"
#include "config.h"
#include <encoder/quadra/quadra.h>
#include <uart/uart.h>
#include <pwm/motor.h>
#include <ax12/ax12.h>
#include "telemetry.h"

#define MOTOR_SCALE (24.0/35.0)

#define UPPER_ARM_POSITION_OFFSET 12000
#define LOWER_ARM_SPEED 0x100

typedef enum {
  LA_ELBOW = 0x01,
  LA_WRIST = 0x02,
  LA_BACK = 0x10,
}lower_arm_t;

extern ax12_t ax12;

struct cs cs_arm;
static struct pid_filter pid_arm;
static struct ramp_filter rampf;
static pwm_motor_t pwm_arm;
static portpin_t sign_arm;
static portpin_t break_arm;
static quadra_t quadra_arm;

typedef enum {
  ARM_STATE_INIT = 0,
  ARM_STATE_CALIBRATION_STARTING,
  ARM_STATE_CALIBRATION_STARTING_WAITING,
  ARM_STATE_CALIBRATION_IN_POSITION,
  ARM_STATE_RUNNING,

}arm_internal_state_t;

typedef struct {
  int32_t consign;
  uint8_t in_position;
}arm_consign_t;

typedef struct {
  arm_internal_state_t state;
  uint8_t motor_is_active;
  uint16_t count;
  int32_t upper_arm_offset;

  arm_consign_t upper_consign;
  arm_consign_t elbow_consign;
  arm_consign_t wrist_consign;

}arm_t;

// arm singleton
arm_t arm;

// -- private functions --

static void _set_arm_motor_sign(bool sign) {
  if(sign)
    portpin_outset(&sign_arm);
  else
    portpin_outclr(&sign_arm);
}

static void set_arm_motor_consign(void *dummy, int32_t consign) {
  // scale motor consign
  consign *= MOTOR_SCALE;
  if(arm.motor_is_active) {
    pwm_motor_set(&pwm_arm, -consign);
  }
  else {
    pwm_motor_set(&pwm_arm, 0);
  }
}

static int32_t get_upper_arm_position(void *dummy) {
  return quadra_get_value(&quadra_arm) - arm.upper_arm_offset;
}

static int16_t _arm_lower_get_position(arm_part_t armpart) {
  uint16_t w;
  switch(armpart) {
    case A_ELBOW:
      ax12_read_word(&ax12, LA_ELBOW, AX12_ADDR_PRESENT_POSITION_L, &w);
      return (int16_t)w-0x1FF;
    case A_WRIST:
      ax12_read_word(&ax12, LA_WRIST, AX12_ADDR_PRESENT_POSITION_L, &w);
      return (int16_t)w-0x1FF;
    default: 
      return 0;
  }
}

static void _arm_lower_set_position(lower_arm_t arm, int16_t position) {
  ax12_write_byte(&ax12, arm, AX12_ADDR_TORQUE_ENABLE, 0x01);
  ax12_write_word(&ax12, arm, AX12_ADDR_GOAL_POSITION_L, 0x1FF + position);
  ax12_write_word(&ax12, arm, AX12_ADDR_MOVING_SPEED_L, LOWER_ARM_SPEED);
}

static void _arm_upper_set_position(int16_t position) {
  cs_set_consign(&cs_arm, position);
}

int16_t tm_arm_upper_position;
int16_t tm_arm_elbow_position;
int16_t tm_arm_arm_position;
static void _update_arm_telemetry(void) {
  tm_arm_upper_position = get_upper_arm_position(NULL);
  tm_arm_elbow_position = _arm_lower_get_position(A_ELBOW);
  tm_arm_arm_position = _arm_lower_get_position(A_WRIST);
}

// -- public functions --

/** @brief Initialize arm */
void arm_init() {

  // arm 
  sign_arm = PORTPIN(E,2);
  portpin_dirset(&sign_arm);
  // break
  break_arm = PORTPIN(E,3);
  portpin_dirset(&break_arm);
  portpin_outclr(&break_arm);

  pwm_motor_init(&pwm_arm, &TCD0, 'D', _set_arm_motor_sign);
  pwm_motor_set_frequency(&pwm_arm, 20000);

  quadra_init(&quadra_arm,  &TCC1, 0, PORTPIN(E,0), PORTPIN(E,1), 8);

  pid_init(&pid_arm);
  pid_set_gains(&pid_arm, 500, 20, 5);
  pid_set_maximums(&pid_arm, 0, 100, 0);
  pid_set_out_shift(&pid_arm, 8);

  ramp_init(&rampf);
  ramp_set_vars(&rampf, 1000, 1000);

  cs_init(&cs_arm);
  cs_set_consign_filter(&cs_arm, ramp_do_filter, &rampf);
  cs_set_feedback_filter(&cs_arm, NULL, NULL);
  cs_set_correct_filter(&cs_arm, pid_do_filter, &pid_arm);
  cs_set_process_in(&cs_arm, set_arm_motor_consign, NULL);
  cs_set_process_out(&cs_arm, get_upper_arm_position, NULL);

  // init AX-12
  portpin_dirset(&AX12_DIR_PP);
  portpin_outclr(&AX12_DIR_PP);
  portpin_dirset(&PORTPIN_TXDN(uart_get_usart(UART_AX12)));

  arm.upper_arm_offset = 0;
  arm.motor_is_active = false;
  arm.state = ARM_STATE_INIT;
}

void arm_set_position(arm_part_t armpart, int32_t position) {
  switch(armpart) {
    case A_UPPER:
      arm.upper_consign.in_position = false;
      arm.upper_consign.consign = position;
      _arm_upper_set_position(position);
      break;
    case A_ELBOW:
      arm.elbow_consign.in_position = false;
      arm.elbow_consign.consign = position;
      _arm_lower_set_position(LA_ELBOW, position);
      break;
    case A_WRIST:
      arm.wrist_consign.in_position = false;
      arm.wrist_consign.consign = position;
      _arm_lower_set_position(LA_WRIST, position);
      break;
    default: break;
  }
}

void arm_start_calibration() {
  arm.state = ARM_STATE_CALIBRATION_STARTING;
  _arm_lower_set_position(LA_ELBOW, 0);
  _arm_lower_set_position(LA_WRIST, 0);
}

uint8_t arm_is_running() {
  return arm.state == ARM_STATE_RUNNING;
}

void arm_update() {

  portpin_outtgl(&LED_RUN_PP);

  quadra_update(&quadra_arm);

  cs_manage(&cs_arm);

  arm.count++;

  static int _old_state = 0;
  if(_old_state != arm.state) {
    _old_state = arm.state;
  }

  switch(arm.state) {
    case ARM_STATE_INIT:
      break;

    case ARM_STATE_CALIBRATION_STARTING:
      arm.count = 0;
      arm.state = ARM_STATE_CALIBRATION_STARTING_WAITING;
      _arm_lower_set_position(LA_ELBOW, 0);
      _arm_lower_set_position(LA_WRIST, 0);
      break;

    case ARM_STATE_CALIBRATION_STARTING_WAITING:
      if(arm.count > 10) {
        arm.state = ARM_STATE_CALIBRATION_IN_POSITION;
        // secure arm
        _arm_lower_set_position(LA_ELBOW, 0);
        _arm_lower_set_position(LA_WRIST, 0);

        arm.count = 0;
      }
      break;

    case ARM_STATE_CALIBRATION_IN_POSITION: {
      static int32_t last_arm_position = 0x7fffffff;
      int32_t arm_position = get_upper_arm_position(NULL);
      // check arm does not move
      if(abs(last_arm_position - arm_position) < 4) {
        
        // reset position
        arm.upper_arm_offset = arm_position + UPPER_ARM_POSITION_OFFSET;

        // set arm position to neutral
        _arm_lower_set_position(LA_ELBOW, 0);
        _arm_lower_set_position(LA_WRIST, 0);
        _arm_upper_set_position(-UPPER_ARM_POSITION_OFFSET);
        // reset CS and PID
        pid_reset(&pid_arm);
        ramp_reset(&rampf, -UPPER_ARM_POSITION_OFFSET);
        arm.motor_is_active = true;

        // switch state to running
        arm.state = ARM_STATE_RUNNING;
      }
      last_arm_position = arm_position;
      break;
    }

    case ARM_STATE_RUNNING:
      break;

    default: break;
  }

  // update telemetry
  TM_PERIODIC(_update_arm_telemetry());
  TM_DL_ARM(tm_arm_upper_position, tm_arm_elbow_position, tm_arm_arm_position);
}

void arm_activate_debug(bool b) {

  uint8_t w = b?0x01:0x00;
  // deactivate lower arm elbow and wrist
  ax12_write_byte(&ax12, LA_ELBOW, AX12_ADDR_TORQUE_ENABLE, w);
  ax12_write_byte(&ax12, LA_WRIST, AX12_ADDR_TORQUE_ENABLE, w);
  // deactivate upper arm motor
  arm.motor_is_active = b;
}

void arm_get_debug(arm_debug_t *debug) {
  uint16_t w;
  debug->upper = get_upper_arm_position(NULL);
  ax12_read_word(&ax12, LA_ELBOW, AX12_ADDR_PRESENT_POSITION_L, &w);
  debug->elbow = w-0x1FF;
  ax12_read_word(&ax12, LA_WRIST, AX12_ADDR_PRESENT_POSITION_L, &w);
  debug->wrist = w-0x1FF;
}

