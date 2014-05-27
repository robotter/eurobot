#include <avarix.h>
#include <avarix/portpin.h>
#include <stdio.h>
#include "battery_monitor.h"
#include "adc.h"
#include "battery_monitor_config.h"

static BATTMON_t Batt;

/*----------------- local function ---------------------*/

float Battmon_GetDischargedThresholdVoltage_mV(void);

/*---------------- global function ---------------------*/


void BATTMON_Init()
{
  Batt.Type = BATTERY_TYPE;

#if (BATTERY_TYPE == LIPO)
  Batt.BatterySpecs.Lipo.Size = BATTERY_SIZE;
#elif (BATTERY_TYPE == PB)
  Batt.BatterySpecs.Pb.NominalVoltage_V = BATTERY_PB_VOLTAGE_V; 
#endif

  Batt.Adc = BATTERY_ADC_PORTPIN;
  Batt.FilterMemory = 0.0f;
  Batt.Status = BATTERY_DISCHARGED;

#ifdef BATTERY_LED_ALARM_PORTPIN
  Batt.Led = BATTERY_LED_ALARM_PORTPIN; 
  Batt.LedDefined = 1u;
  portpin_dirset(&Batt.Led);
#else
  Batt.LedDefined = 0u;
#endif

  portpin_dirclr(&(Batt.Adc));
}

void BATTMON_monitor(void)
{
  /* get analog value from adc*/
  uint16_t AdcVal =0u;

  AdcVal = adc_GetValue(&BATTERY_ADC, BATTERY_ADC_PORTPIN.pin);
  /* convert it to battery voltage*/
  Batt.BatteryVoltage_mV = AdcVal * BATTERY_VOLT_DIVIDER_GAIN;

  /* convert it from adc unit to voltage */
  Batt.BatteryVoltage_mV =  (uint16_t)(((uint32_t)Batt.BatteryVoltage_mV * 33000u)/16u/4096u); 

  /* add 0.6V due to protection diode */
  Batt.BatteryVoltage_mV += 600;
  //Batt.BatteryVoltage_mV = AdcVal;

  /* filter analog value */
  float NewFilerVal;
  NewFilerVal = Batt.FilterMemory + (float)(BATTERY_LOW_PASS_FILTER_COEFF)*((float)(Batt.BatteryVoltage_mV) - Batt.FilterMemory);
  Batt.FilterMemory = NewFilerVal;

  /* update status */ 
  if (Batt.FilterMemory >= Battmon_GetDischargedThresholdVoltage_mV())
  {
    Batt.Status = BATTERY_CHARGED;

    if (Batt.LedDefined)
    {
      portpin_outclr(&(Batt.Led));
    }
  }
  else
  {
    Batt.Status = BATTERY_DISCHARGED;
    if (Batt.LedDefined)
    {
      portpin_outtgl(&(Batt.Led));
    }
  }
}

BATTMON_BatteryStatus_t BATTMON_IsBatteryDischarged(void)
{
  return Batt.Status;
}

/*---------------------- Local functions --------------------------*/

/* 
 * @brief return the discharged threshold voltage of each battery 
 */
float Battmon_GetDischargedThresholdVoltage_mV(void)
{
  if (Batt.Type == LIPO)
  {
    // for lipo, consider 3.7V per cell as discharged cell
    switch(Batt.BatterySpecs.Lipo.Size)
    {
      case LIPO_1S :
        return 3700.0f;
      case LIPO_2S :
        return 7400.f;
      case LIPO_3S :
        return 11100.0f;
      case LIPO_4S :
        return 14800.0f;
      case LIPO_5S :
        return 18500.0f;
      case LIPO_6S :
        return 22200.0f;
      default : 
        return 32000.0f;
    }
  }
  else if (Batt.Type == PB)
  {
    // compute number of cell (2.0V nominal per cell) and consider it discharged if voltage is lower than 1.8V)
    return ( (((float)Batt.BatterySpecs.Pb.NominalVoltage_V)*1.8f) /2.0f);  
  }

  // default : consider system discharged (100V !!!!)
  return 100.0f;
}

uint16_t BATTMON_GetVoltage_mV(void)
{
  return Batt.BatteryVoltage_mV;
}
