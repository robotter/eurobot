#ifndef APDS9800_H
#define APDS9800_H

#include <avarix/portpin.h>
#include "object_def.h"

typedef struct{
  portpin_t EnablePort;
  portpin_t PSDoutPort;
  portpin_t IrLedPwmPort;
  uint16_t ProximityLedPulseThresholdNb;
} apds9800_t;


/*
 * @brief set the port of the enable pin of the apds9800
 * this enable pin enables the proximity sensor
 * @param s structure of the apds9800
 * @param port the port of the µC connected to PS_ENB of the APDS9800
 */
void APDS9800_SetProximityEnablePort(apds9800_t *s, portpin_t *port);

/*
 * @brief set the port of connected to the LEDON of the apds9800
 * this enable pin enables the proximity sensor
 * @param s structure of the apds9800
 * @param port the port of the µC connected to LEDON of the APDS9800
 */
void APDS9800_SetIrLedPort(apds9800_t *s, portpin_t *port);

/*
 * @brief set the port of connected to the PS_DOUT of the apds9800
 * this enable pin enables the proximity sensor
 * @param s structure of the apds9800
 * @param port the port of the µC connected to PS_DOUT of the APDS9800
 */
void APDS9800_SetProximityOutputPort(apds9800_t *s, portpin_t *port);

/* @brief initialise the ports of the apds9800
 * @warning must be called after apds9800_setProximityEnablePort, apds9800_setIrLedPort and apds9800_setProximityOutputPort functions
 * @param s structure of the apds9800
 */
void APDS9800_Init(apds9800_t *s);

/* @brief sets the threshold number of pwm that can be sent to detect an object
 * @param s structure of the apds9800
 * @param threshold maximum number of pulse sent to toogle dout pin 
 *  if dout pin doesn't toogle after threshold IR pulses, there is no object
 */
void APDS9800_SetProximityLedPulseThresholdNb(apds9800_t *s, uint16_t threshold);

/* @brief return presence of object  
 * @param s structure of the apds9800
 * @param ObjectFound pointer, value returne indicates if object is present or not (
 */
void APDS9800_IsObjectPresent(apds9800_t *s, OBJECT_t *ObjectFound);

/* @brief return ambient light level  
 * @param s structure of the apds9800
 * @param level : value returned by the atxmega adc (must be between 0 and 4048)
 */
void APDS9800_GetAmbientLightLevel(apds9800_t *s, uint16_t *Level);

/* @brief return proximity distance
 * This is in fact an analog value converted by the adc, this is the integrated value of the proximity sensor photodiode after the proximity sensor test
 * @warning apds9800_IsObjectPresent must be called before
 * @param s structure of the apds9800
 * @param level : value returned by the atxmega adc (must be between 0 and 4048)
 */
void APDS9800_GetProximityDistance(apds9800_t *s, uint16_t *Level);

#endif //APDS9800_H
