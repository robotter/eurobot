#ifndef BATTERY_MONITOR_H__
#define BATTERY_MONITOR_H__

#include <stdint.h>

/** @brief Initialize xmega ADC module */
void battery_monitor_init(void);

/** @brief Do one measure */
uint8_t battery_monitor_measure(void);

#endif//BATTERY_MONITOR_H___
