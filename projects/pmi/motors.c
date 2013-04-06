#include <avarix/portpin.h>
#include <pwm/motor.h>
#include <encoder/aeat/aeat.h>
#include "motors.h"
#include "config.h"


/// Single motor data
typedef struct {
  aeat_t encoder;
  pwm_motor_t pwm;

} motor_t;

static motor_t motor_left;
static motor_t motor_right;


/// Set left motor rotation direction
static void motor_left_set_sign(bool sign)
{
  if(sign) {
    portpin_outclr(&MOTOR_LEFT_ROTATE_A_PP);
    portpin_outset(&MOTOR_LEFT_ROTATE_B_PP);
  } else {
    portpin_outset(&MOTOR_LEFT_ROTATE_A_PP);
    portpin_outclr(&MOTOR_LEFT_ROTATE_B_PP);
  }
}

/// Set right motor rotation direction
static void motor_right_set_sign(bool sign)
{
  if(sign) {
    portpin_outclr(&MOTOR_RIGHT_ROTATE_A_PP);
    portpin_outset(&MOTOR_RIGHT_ROTATE_B_PP);
  } else {
    portpin_outset(&MOTOR_RIGHT_ROTATE_A_PP);
    portpin_outclr(&MOTOR_RIGHT_ROTATE_B_PP);
  }
}


void motors_init(void)
{
  // set port pins direction
  portpin_dirset(&MOTOR_LEFT_ENABLE_A_PP);
  portpin_dirset(&MOTOR_LEFT_ENABLE_B_PP);
  portpin_dirset(&MOTOR_LEFT_ROTATE_A_PP);
  portpin_dirset(&MOTOR_LEFT_ROTATE_B_PP);
  portpin_dirset(&MOTOR_LEFT_ENCODER_CS_PP);
  portpin_dirset(&MOTOR_RIGHT_ENABLE_A_PP);
  portpin_dirset(&MOTOR_RIGHT_ENABLE_B_PP);
  portpin_dirset(&MOTOR_RIGHT_ROTATE_A_PP);
  portpin_dirset(&MOTOR_RIGHT_ROTATE_B_PP);
  portpin_dirset(&MOTOR_RIGHT_ENCODER_CS_PP);

  // encoders
  aeat_init(&motor_left.encoder, MOTOR_LEFT_ENCODER_CS_PP);
  aeat_init(&motor_right.encoder, MOTOR_RIGHT_ENCODER_CS_PP);
  aeat_spi_init();

  // PWMs
  pwm_motor_init(&motor_left.pwm, &TCD0, MOTOR_LEFT_PWM_TC_CH, MOTOR_LEFT_PWM_PP, motor_left_set_sign);
  pwm_motor_init(&motor_right.pwm, &TCD0, MOTOR_RIGHT_PWM_TC_CH, MOTOR_RIGHT_PWM_PP, motor_right_set_sign);
  pwm_motor_set_frequency(&motor_left.pwm, MOTOR_FREQUENCY);
  pwm_motor_set_frequency(&motor_right.pwm, MOTOR_FREQUENCY);
  motors_set_consign(0, 0);

  portpin_outset(&MOTOR_LEFT_ENABLE_A_PP);
  portpin_outset(&MOTOR_LEFT_ENABLE_B_PP);
  portpin_outset(&MOTOR_RIGHT_ENABLE_A_PP);
  portpin_outset(&MOTOR_RIGHT_ENABLE_B_PP);
}


void motors_set_consign(int16_t left, int16_t right)
{
  pwm_motor_set(&motor_left.pwm, left);
  pwm_motor_set(&motor_right.pwm, right);
}


void motors_brake(void)
{
  portpin_outclr(&MOTOR_LEFT_ROTATE_A_PP);
  portpin_outclr(&MOTOR_LEFT_ROTATE_B_PP);
  portpin_outclr(&MOTOR_RIGHT_ROTATE_A_PP);
  portpin_outclr(&MOTOR_RIGHT_ROTATE_B_PP);
}


void motors_update_encoders(void)
{
  aeat_update(&motor_left.encoder);
  aeat_update(&motor_right.encoder);
}


int32_t motor_left_encoder_value(void)
{
  return aeat_get_value(&motor_left.encoder);
}

int32_t motor_right_encoder_value(void)
{
  return aeat_get_value(&motor_right.encoder);
}


