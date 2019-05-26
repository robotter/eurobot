#include <pwm/motor.h>
#include "servos.h"

#if defined(GALIPETTE)

pwm_motor_t servos[SERVO_COUNT];

void servos_init(void) {
  pwm_servo_init(servos,   (TC0_t *) &TCC0, 'C');
  pwm_servo_init(servos+1, (TC0_t *) &TCC0, 'D');
  pwm_servo_init(servos+2, (TC0_t *) &TCD0, 'C');
  pwm_servo_init(servos+3, (TC0_t *) &TCD0, 'D');
}

void servo_set(uint8_t id, uint16_t value) {
  if(id < SERVO_COUNT) {
    uint16_t pwm = value < PWM_MOTOR_MAX ? value : PWM_MOTOR_MAX;
    pwm_motor_set(servos+id, pwm);
  }
}

#else

// no servos on Galipeur

#endif

