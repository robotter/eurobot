#ifndef TYROLIENNE_H
#define TYROLIENNE_H

#include <stdbool.h>
#include <stdint.h>

void tyrolienne_init(void);

void tyrolienne_start(void);
bool tyrolienne_arrived(void);

void tyrolienne_shutdown(void);

#endif
