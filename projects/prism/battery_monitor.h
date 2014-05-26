#ifndef BATTERY_MONITOR_H
#define BATTERY_MONITOR_H

#include <avarix.h>
#include <avarix/portpin.h>

typedef enum{
  LIPO = 0,
  PB,
}BATTMON_BatteryType_t;

typedef enum{
  LIPO_1S = 0,
  LIPO_2S,
  LIPO_3S,
  LIPO_4S,
  LIPO_5S,
  LIPO_6S
}BATTMON_LipoSize_t;

typedef enum{
  BATTERY_DISCHARGED,
  BATTERY_CHARGED,
}BATTMON_BatteryStatus_t;

typedef struct{
  BATTMON_LipoSize_t Size;
} BATTMON_Lipo_t;

typedef struct{
  uint8_t NominalVoltage_V;
} BATTMON_Pb_t;


/*
 * Structure that contains everything necessary to the battery monitoring
 */
typedef struct{
  portpin_t Led;
  uint8_t LedDefined;

  BATTMON_BatteryType_t Type;

  union{
    BATTMON_Lipo_t Lipo;
    BATTMON_Pb_t Pb;
  }BatterySpecs;

  portpin_t Adc;
  BATTMON_BatteryStatus_t Status;
  uint16_t BatteryVoltage_mV;
  float FilterMemory;
}BATTMON_t;

/*
 * @brief initialize battery monitoring
 * @param Batt pointer to a BATTMON_t structure
 */
void BATTMON_Init(void);


/*
 * @brief monitor the battery voltage and make blink battery alarm led 
 * @brief must be called at 2Hz (500ms period)
 */
void BATTMON_monitor(void);

/*
 * @brief return the status of the battery
 * @retval 0 battery not discharged
 * @retval 1 battery discharged
 */
BATTMON_BatteryStatus_t BATTMON_IsBatteryDischarged(void);

uint16_t BATTMON_GetVoltage_mV(void);

#endif //BATTERY_MONITOR_H
