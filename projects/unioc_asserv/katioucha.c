#include "katioucha.h"
#include <pwm/motor.h>
#include "katioucha_config.h"

static pwm_motor_t servos[2] ;

void katioucha_init(void)
{
  // initialise pwm port for servos
  pwm_servo_init(servos, (TC0_t*)&KATIOUCHA_LOW_PWM_PORT, KATIOUCHA_LOW_PWM_CHANNEL); 
  pwm_servo_init(servos+1, (TC0_t*)&KATIOUCHA_HIGH_PWM_PORT, KATIOUCHA_HIGH_PWM_CHANNEL); 

  // frequency set to 50Hz => 20ms
  pwm_motor_set_frequency(servos, 50u);
  pwm_motor_set_frequency(servos+1, 50u);

  //set position
  katioucha_set_position(KATIOUCHA_LINE_LOW, KATIOUCHA_RECHARGE);
  katioucha_set_position(KATIOUCHA_LINE_HIGH, KATIOUCHA_RECHARGE);
}


#include <stdio.h>

// update position of servo line
void katioucha_set_position(katioucha_line_t line, katioucha_pos_t pos)
{
  if (line <= 1 && pos <= 3)
  {
    uint8_t idx = line * 4 + pos;

    int16_t servo_position = katioucha_servo_position[idx];

    if (line == KATIOUCHA_LINE_LOW)
    {
      pwm_motor_set(servos, servo_position);
    }
    else if (line == KATIOUCHA_LINE_HIGH)
    {
      pwm_motor_set(servos+1, servo_position);
    }
  }
}

void katioucha_set_servo_position(katioucha_line_t line, int16_t pos)
{
    if (line == KATIOUCHA_LINE_LOW)
    {
      pwm_motor_set(servos, pos);
    }
    else if (line == KATIOUCHA_LINE_HIGH)
    {
      pwm_motor_set(servos+1, pos);
    }
}
