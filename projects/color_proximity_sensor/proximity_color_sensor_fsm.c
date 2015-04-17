#include <avr/eeprom.h>
#include <string.h>
#include <avr/interrupt.h>
#include <avarix/intlvl.h>
#include <i2c/i2c.h>
#include <timer/uptime.h>
#include <rome/rome.h>
#include "proximity_color_sensor_fsm.h"
#include "proximity_color_sensor_fsm_config.h"
#include "apds9800.h"
#include "tcs3772x.h"
#include "rgb_led.h"

#include "APDS9800_config.h"
#include "tcs3772x_config.h"

extern rome_intf_t rome;

uint8_t Pcsfsm_IsIrqRequested(void);
void Pcsfsm_init_tcs37725_irq(void);
void Pcsfsm_ApplyFilters(void);
void Pcsfsm_EnableLeds(void);
void Pcsfsm_DisableLeds(void);



typedef enum{
  PCSFSM_INACTIVE = 0x00u,
  PCSFSM_WAIT_FOR_PRESENCE_DETECTION,
  PCSFSM_APDS9800_PROXIMITY_DETECTION,
  PCSFSM_ENABLE_LEDS,
  PCSFSM_LIGHT_STABILIZATION,
  PCSFSM_TCS3772_COLOR_MEASURE,
  PCSFSM_DISABLE_LEDS,
  PCSFSM_APPLY_FILTER,
  PCSFSM_UPDATE_OBJ_DETECT_STATE,

} Pcsfsm_state_t;

EEMEM Color_Filter_t Eep_Color_Filter[COLOR_T_ELEMENT_NUMBER-1];

/*
 * @brief structure used to define if apds9800 detects or not an object
 */
typedef struct{
  apds9800_t apds;
  uint8_t object_detected_cnt;
  uint8_t no_object_detected_cnt;
}apds9800_object_detection_t;


/*
 * @ brief Structure used by the FSM to funtion
 */
typedef struct{
  Pcsfsm_state_t fsm_state;

  /* raw values of the color directly measured by the sensor*/
  tcs3772x_ColorResult_t color_measured;
  Color_Filter_t Filter[COLOR_T_ELEMENT_NUMBER-1];

  OBJECT_t object;
  Color_t color;
  
  /* uptime at color detection */
  uint32_t detect_uptime;
  uint32_t detect_uptime_threshold;
  
  /* analog color sensor structure */
  apds9800_object_detection_t apds9800;

  /* i2c color sensor structure */
  tcs3772x_t tcs37725;
  i2cm_t tcs37725_i2c;

  /* debug data */
  uint16_t tcs37725_distance;
} Pcsfsm_t;

/* singleton */
static Pcsfsm_t fsm;

volatile uint8_t tcs37725_irq_request;

/****************************** Global Function ******************************/

void PCSFSM_Init(void)
{
  fsm.fsm_state = PCSFSM_INACTIVE;
  
  //eeprom_read_block(&fsm.Filter, &Eep_Color_Filter, sizeof(Eep_Color_Filter)*sizeof(Color_Filter_t));
  fsm.object = APDS9800_NO_OBJECT;
  fsm.color = UNDEFINED_COLOR;

  fsm.detect_uptime_threshold = OBJECT_DETECTION_TIMEOUT_US;
  /* Initialize APDS9800 structure */

  APDS9800_SetProximityEnablePort(&fsm.apds9800.apds, &APDS9800_EN_PROXIMITY_SENSOR_PORT);
  APDS9800_SetIrLedPort(&fsm.apds9800.apds, &APDS9800_IR_LED_PWM_PORT);
  APDS9800_SetProximityOutputPort(&fsm.apds9800.apds, &APDS9800_PS_DOUT_PORT);

  APDS9800_Init(&fsm.apds9800.apds);

  /* Initialize tcs37725 structure */
  i2c_init();
  tcs3772x_init(&fsm.tcs37725, &TCS37725_I2C.MASTER ,37725);
  tcs3772x_ProximitySetPulseNb(&fsm.tcs37725, TCS37725_PULSE_NB);
  tcs3772x_RGBCSetIntegrationTime_ms(&fsm.tcs37725, TCS37725_INTEGRATION_TIME_MS);
  tcs3772x_SetProximityInterruptThreshold(&fsm.tcs37725, TCS37725_PROX_LOW_THRESHOLD, 
                                                         TCS37725_PROX_HI_THRESHOLD, 
                                                         TCS37725_CONSECUTIVE_DETECT_THRESHOLD );
  tcs3772x_SetIRLedCurrentAndRGBCGain(&fsm.tcs37725, TCS37725_IR_LED_CURRENT, TCS37725_RGBC_GAIN);

  /* Initialize tcs3772 irq detection*/
  Pcsfsm_init_tcs37725_irq();

  tcs3772x_ProximityEnableDetection(&fsm.tcs37725);
  tcs3772x_ProximityEnableInterrupt(&fsm.tcs37725);
  /* Initializa the leds */
  rgb_led_Init(); 


  /* finite state machine ready to funtion */
  fsm.fsm_state = PCSFSM_WAIT_FOR_PRESENCE_DETECTION;
}

/* Irq function called at tcs3772 object detection */


void PCSFSM_Update(void)
{
  OBJECT_t object_detected;
  switch(fsm.fsm_state)
  {
    case PCSFSM_INACTIVE:
      break;

    case PCSFSM_WAIT_FOR_PRESENCE_DETECTION:
      if (Pcsfsm_IsIrqRequested())
      {
        // rearm interruption
        tcs3772x_ProximityClearInterrupt(&fsm.tcs37725);
        // go to next step
        fsm.fsm_state = PCSFSM_APDS9800_PROXIMITY_DETECTION;
        fsm.fsm_state = PCSFSM_ENABLE_LEDS;
        ROME_SEND_COLOR_SENSOR_TM_DETECTION(&rome, fsm.object==APDS9800_OBJECT_DETECTED, fsm.color);
      }
      else
      {
        uint32_t uptime = uptime_us();
        if (( (uptime - fsm.detect_uptime) > fsm.detect_uptime_threshold) &&
        (fsm.object == APDS9800_OBJECT_DETECTED))
        {
          fsm.object = APDS9800_NO_OBJECT;
        }
      }
      fsm.tcs37725_distance = tcs3772x_GetProxRawData(&fsm.tcs37725);
      break;


    case PCSFSM_APDS9800_PROXIMITY_DETECTION:
      APDS9800_IsObjectPresent(&fsm.apds9800.apds, &object_detected);
      if (object_detected == APDS9800_OBJECT_DETECTED)
      {
        fsm.apds9800.object_detected_cnt ++;
      }
      else
      {
        fsm.apds9800.no_object_detected_cnt ++;
      }

      if(fsm.apds9800.object_detected_cnt >= APDS9800_OBJECT_PRESENCE_CHOICE_THRESHOLD ) 
      {
        fsm.apds9800.object_detected_cnt =0;
        fsm.apds9800.no_object_detected_cnt =0;
        fsm.fsm_state = PCSFSM_ENABLE_LEDS;
      }
      else if(fsm.apds9800.no_object_detected_cnt >= APDS9800_OBJECT_PRESENCE_CHOICE_THRESHOLD ) 
      {
        fsm.apds9800.object_detected_cnt =0;
        fsm.apds9800.no_object_detected_cnt =0;
        fsm.fsm_state = PCSFSM_WAIT_FOR_PRESENCE_DETECTION;
      }

      break;

    case PCSFSM_ENABLE_LEDS:
        Pcsfsm_EnableLeds();
        fsm.fsm_state = PCSFSM_LIGHT_STABILIZATION;
      break;

    case PCSFSM_LIGHT_STABILIZATION:
        fsm.fsm_state = PCSFSM_TCS3772_COLOR_MEASURE;
      break;

    case PCSFSM_TCS3772_COLOR_MEASURE:
      if (tcs3772x_RGBCGetValue(&fsm.tcs37725, &fsm.color_measured) == 0)
      {
        fsm.fsm_state = PCSFSM_DISABLE_LEDS;
      }
      break;
        
    case PCSFSM_DISABLE_LEDS:
        Pcsfsm_DisableLeds();
        fsm.fsm_state = PCSFSM_APPLY_FILTER;
      break;

    case PCSFSM_APPLY_FILTER:
      Pcsfsm_ApplyFilters();
      tcs3772x_ProximityEnableInterrupt(&fsm.tcs37725);
      fsm.fsm_state = PCSFSM_UPDATE_OBJ_DETECT_STATE;
      break;

    case PCSFSM_UPDATE_OBJ_DETECT_STATE:
      fsm.fsm_state = PCSFSM_WAIT_FOR_PRESENCE_DETECTION;
      fsm.detect_uptime = uptime_us();
      fsm.object = APDS9800_OBJECT_DETECTED;
      break;

    default : fsm.fsm_state = PCSFSM_INACTIVE;
              break;
  }
}
OBJECT_t PCCFSM_IsObjectDetected(void)
{
  return fsm.object;
}

Color_t PCCFSM_GetObjectColor(void)
{
  return fsm.color;
}

uint16_t PCSFSM_GetTcs37725ProxDistance(void)
{
  return fsm.tcs37725_distance;
}

void PCSFSM_SetTcs37725ProximityThreshold(uint16_t threshold_low, uint16_t threshold_high, uint8_t consecutive_detect_threshold)
{
  tcs3772x_SetProximityInterruptThreshold(&fsm.tcs37725, threshold_low, threshold_high, consecutive_detect_threshold);
}

void PCSFSM_updateColorFilter(Color_Filter_t *filter)
{
  memcpy(&fsm.Filter[filter->color], filter, sizeof(Color_Filter_t));
  eeprom_update_block(&fsm.Filter, &Eep_Color_Filter, sizeof(Eep_Color_Filter)*sizeof(Color_Filter_t));

}

void PCSFSM_GetColorFilter(Color_Filter_t *filter, Color_t color)
{
  memcpy(filter, &fsm.Filter[color], sizeof(Color_Filter_t));
}
/****************************** Local Function ********************************/

void Pcsfsm_init_tcs37725_irq(void)
{
  /* configure pin interrup sens: must detect falling edge */
  TCS37725_IRQ_PORT.PIN7CTRL = PORT_ISC_FALLING_gc; 

  /* set irq level */
  TCS37725_IRQ_PORT.INTCTRL &= PORT_INT1LVL_gm; 
  TCS37725_IRQ_PORT.INTCTRL |= PORT_INT1LVL_LO_gc; 

  /* enable interrupt pin of the tcs37725 */
  TCS37725_IRQ_PORT.INT1MASK |= _BV(TCS37725_IRQ_PIN_NB);
}


ISR(PORTD_INT1_vect)
{
  tcs37725_irq_request = 1;
}

uint8_t Pcsfsm_IsIrqRequested(void)
{
  uint8_t request = 0;

  INTLVL_DISABLE_BLOCK(INTLVL_LO) 
  {
    request = tcs37725_irq_request;
    tcs37725_irq_request = 0;
  }

  return request;
}



void Pcsfsm_ApplyFilters(void)
{
  fsm.color = UNDEFINED_COLOR;
  for (uint8_t it = 0; it < COLOR_T_ELEMENT_NUMBER-1; it++)
  {
    if ((fsm.Filter[it].UseRedThreshold == 0)
        || ((fsm.color_measured.red_value >= fsm.Filter[it].RedLowThreshold) && (fsm.color_measured.red_value <= fsm.Filter[it].RedHighThreshold)))
    {
      if ((fsm.Filter[it].UseGreenThreshold == 0)
          || ((fsm.color_measured.green_value >= fsm.Filter[it].GreenLowThreshold) && (fsm.color_measured.green_value <= fsm.Filter[it].GreenHighThreshold)))
      {
        if ((fsm.Filter[it].UseBlueThreshold == 0)
            || ((fsm.color_measured.blue_value >= fsm.Filter[it].BlueLowThreshold) && (fsm.color_measured.blue_value <= fsm.Filter[it].BlueHighThreshold)))
        {
          fsm.color = fsm.Filter[it].color;
        }
      }
    }
  }
}


void Pcsfsm_EnableLeds(void)
{
  rgb_led_SetRedLedRatio(PCFSM_RED_LED_LVL); 
  rgb_led_SetBlueLedRatio(PCFSM_BLUE_LED_LVL); 
  rgb_led_SetGreenLedRatio(PCFSM_GREEN_LED_LVL); 
}

void Pcsfsm_DisableLeds(void)
{
  rgb_led_SetLedOff(); 
}
