#ifndef SERVOS_H
#define SERVOS_H

#include <stdint.h>

#if defined(GALIPETTE)

#define SERVO_COUNT 4  // max ID = SERVO_COUNT - 1

void servos_init(void);
void servo_set(uint8_t id, uint16_t value);

#else
// no servos on Galipeur
#endif

#endif
