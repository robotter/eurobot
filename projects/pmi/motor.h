#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

/// Initialize the motor pwm (PB2 ie 0C0A) and direction pin (PD1)
void motor_init(void);

/// Set the motor pwm
void motor_set_pwm(int16_t pos);

#endif //MOTOR_H
