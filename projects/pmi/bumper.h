#ifndef BUMPER_H
#define BUMPER_H

#include <stdint.h>

void bumper_init(void);

uint8_t is_front_bumper_pushed(void);
uint8_t is_rear_bumper_pushed(void);
uint8_t is_hook_lifted_up(void);

void bumper_manage(void);

#endif //BUMPER_H
