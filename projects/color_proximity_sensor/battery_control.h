#ifndef BATTERY_CONTROL_H
#define BATTERY_CONTROL_H

#include "battery_control_config.h"

/*
 * @brief initialize battery monitoring
 */
void BATTMON_Init(void);


/*
 * @brief
 * @retval 0 battery not discharged
 * @retval 1 battery discharged
 */
uint8_t BATTMON_IsBatteryDischarged(void);

#endif //BATTERY_CONTROL_H
