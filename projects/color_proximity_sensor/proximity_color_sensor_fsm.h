#ifndef PROXIMITY_COLOR_SENSOR_FSM_H
#define PROXIMITY_COLOR_SENSOR_FSM_H

#include <avarix.h>
#include "object_def.h"
#include "color_defs.h"

/* @brief structure containing the different color threshold to detect
 */
typedef struct{
  uint8_t UseRedThreshold;
  uint16_t RedLowThreshold;
  uint16_t RedHighThreshold;

  uint8_t UseGreenThreshold;
  uint16_t GreenLowThreshold;
  uint16_t GreenHighThreshold;

  uint8_t UseBlueThreshold;
  uint16_t BlueLowThreshold;
  uint16_t BlueHighThreshold;

  Color_t color;  
}Color_Filter_t;

/*
 * @brief initialize the PCFSFSM module
 */
void PCSFSM_Init(void);

/*
 * @brief update the proximity and color color finite state machine
 * Remark :must be called every XXX ms
 */
void PCSFSM_Update(void);

/*
 * @brief return if object have been detected or not 
 * @return APDS9800_NO_OBJECT if no object has been seen by sensor
 * @return APDS9800_OBJECT_DETECTED if an object has been seen by sensor
 */
OBJECT_t PCCFSM_IsObjectDetected(void);

/*
 * 
 */
Color_t PCCFSM_GetObjectColor(void);

/*
 * @brief Update color filter stored in EEPROM
 * @param filter must point to a Color_Filter_t structure. filter->color must be set
 */
void PCSFSM_updateColorFilter(Color_Filter_t *filter);
/*
 * @brief return the color filter stored in EEPROM
 * @param filter must point to a Color_Filter_t structure. filter->color must be set
 */
void PCSFSM_GetColorFilter(Color_Filter_t *filter, Color_t color);

uint16_t PCSFSM_GetTcs37725ProxDistance(void);

void PCSFSM_SetTcs37725ProximityThreshold(uint16_t threshold_low, uint16_t threshold_high, uint8_t consecutive_detect_threshold);


#endif //PROXIMITY_COLOR_SENSOR_FSM_H
