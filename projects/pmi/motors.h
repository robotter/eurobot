#ifndef MOTORS_H
#define MOTORS_H

#include <stdint.h>


/// Initialize motors
void motors_init(void);

/// Set motor frequencies
void motors_set_consign(int16_t left, int16_t right);

/// Brake (b is true) or "restore" (b is false) both motors
void motors_brake(bool b);

/// Update motor encoders
void motors_update_encoders(void);

/// Get left motor encoder value
int32_t motor_left_encoder_value(void);
/// Get right motor encoder value
int32_t motor_right_encoder_value(void);


#endif
