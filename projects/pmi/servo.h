#ifndef SERVO_H
#define SERVO_H

#define SERVO_POS_MIN   500
#define SERVO_POS_MAX   2000

/// Initialize the servo pin (PB3 ie OC1A)
void servo_init(void);

/// Set the servo position connected to PB4 (in pwm count)
void servo_set(uint16_t pos);

#endif //SERVO_H
