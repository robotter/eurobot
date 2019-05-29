#ifndef BATTERY_H
#define BATTERY_H

#include <stdint.h>

/// Initialize the ADC used for the battery
void battery_init(void);
/// Get raw battery value
uint16_t battery_get_value(void);

#endif
