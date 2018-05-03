#ifndef BUMPERS_H_
#define BUMPERS_H_

#if defined(GALIPETTE)

#include <stdbool.h>

void bumpers_init(void);
void bumpers_update(void);
bool bumpers_pushed(void);

bool bumper_left_pushed(void);
bool bumper_right_pushed(void);

#endif

#endif
