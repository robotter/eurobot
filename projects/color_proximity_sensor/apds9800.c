#include <avarix.h>
#include <stdio.h>
#include <clock/clock.h>
#include <util/delay.h>
#include "apds9800.h"
#include "APDS9800_config.h"
#include "adc.h"

/*------------------------- local function ----------------------------*/

/*
 * @brief send one pulse IrLedPwmPort (and wait with port at low state)
 * @param apds9800_t structure of the sensor
 */
void Apds9800_send_one_pulse(apds9800_t *s);

/*------------------------- global function ---------------------------*/


void APDS9800_SetProximityEnablePort(apds9800_t *s, portpin_t *port)
{
  if (s != NULL)
  {
    s->EnablePort = *port;
  }
}

void APDS9800_SetIrLedPort(apds9800_t *s, portpin_t *port)
{
  if (s != NULL)
  {
    s->IrLedPwmPort = *port;
  }
}


void APDS9800_SetProximityOutputPort(apds9800_t *s, portpin_t *port)
{
  if (s != NULL)
  {
    s->PSDoutPort = *port;
  }
}

void APDS9800_Init(apds9800_t *s)
{
  portpin_dirclr(&(s->PSDoutPort));
  portpin_dirset(&(s->EnablePort));
  portpin_dirset(&(s->IrLedPwmPort));
  portpin_outset(&(s->EnablePort));
}

void APDS9800_SetProximityLedPulseThresholdNb(apds9800_t *s, uint16_t threshold)
{
  if (s != NULL)
  {
    s->ProximityLedPulseThresholdNb = threshold; 
  } 
}

void APDS9800_IsObjectPresent(apds9800_t *s, OBJECT_t *ObjectFound)
{
  if (ObjectFound != NULL)
  {
    *ObjectFound = APDS9800_NO_OBJECT;

    /* initiate conversion */
    portpin_outclr(&(s->EnablePort));

    /* wait for component to be stabilized */
    _delay_us( APDS9800_ENABLE_TO_LED_ON_WAIT_DURATION_us); 

    for(uint8_t it=0u; it < s->ProximityLedPulseThresholdNb; it ++)
    {
      Apds9800_send_one_pulse(s);
      /* if pin low, object detected*/
      if (portpin_in(&(s->PSDoutPort)) == 0x00)
      {
        *ObjectFound = APDS9800_OBJECT_DETECTED;
        break;
      }
    }
    //portpin_outset(&(s->EnablePort));
  }
}

void APDS9800_GetAmbientLightLevel(apds9800_t *s, uint16_t *Level)
{
  if (Level != NULL)
  {
    *Level = adc_GetValue(&ADCA,  APDS9800_AMBIANT_LIGHT_ANA_PORT.pin);
  }
}

void APDS9800_GetProximityDistance(apds9800_t *s, uint16_t *Level)
{
  if (Level != NULL)
  {
    *Level = adc_GetValue(&ADCA, APDS9800_PROXIMITY_ANA_PORT.pin);
  }
}

/*------------------------- local function ----------------------------*/

void Apds9800_send_one_pulse(apds9800_t *s)
{
  portpin_outset(&(s->IrLedPwmPort));
  _delay_us( APDS9800_IR_LED_MAX_PW_us ); 
  portpin_outclr(&(s->IrLedPwmPort));
  _delay_us(1000000ul/(uint32_t)APDS9800_IR_LED_FREQUENCY_HZ);
}
