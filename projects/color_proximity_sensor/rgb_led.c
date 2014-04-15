#include <pwm/motor.h>
#include "rgb_led.h"
#include "led_config.h"

typedef struct{
  pwm_motor_t RedPwm;
  pwm_motor_t GreenPwm;
  pwm_motor_t BluePwm;
  uint8_t RedPwmRatio;
  uint8_t GreenPwmRatio;
  uint8_t BluePwmRatio;
} rgb_led_t;

static rgb_led_t RGBLed;

static void rgb_led_UpdateLeds(void);

void rgb_led_Init(void)
{
  /* initialise led pwm */
  pwm_servo_init(&(RGBLed.RedPwm), (TC0_t*)(&RED_LED_PWM_TC), RED_LED_PWM_CH);
  pwm_servo_init(&(RGBLed.GreenPwm), (TC0_t*)(&GREEN_LED_PWM_TC), GREEN_LED_PWM_CH);
  pwm_servo_init(&(RGBLed.BluePwm), (TC0_t*)(&BLUE_LED_PWM_TC), BLUE_LED_PWM_CH);

  /* set pwm frequency */
  pwm_motor_set_frequency(&(RGBLed.RedPwm), RGB_LED_PWM_FREQUENCY_HZ);  
  pwm_motor_set_frequency(&(RGBLed.GreenPwm), RGB_LED_PWM_FREQUENCY_HZ);  
  pwm_motor_set_frequency(&(RGBLed.BluePwm), RGB_LED_PWM_FREQUENCY_HZ);  

  rgb_led_SetLedOff();
}

void rgb_led_SetLedOn(void)
{
  rgb_led_SetRedLedRatio(255u);
  rgb_led_SetGreenLedRatio(255u);
  rgb_led_SetBlueLedRatio(255u);
}

void rgb_led_SetLedOff(void)
{
  rgb_led_SetRedLedRatio(0u);
  rgb_led_SetGreenLedRatio(0u);
  rgb_led_SetBlueLedRatio(0u);
}

void rgb_led_SetBlueLedRatio(uint8_t Ratio)
{
  RGBLed.BluePwmRatio = Ratio;
  rgb_led_UpdateLeds();
}

uint8_t rgb_led_getBlueLedRatio(void)
{ 
  return RGBLed.BluePwmRatio;
}

void rgb_led_SetGreenLedRatio(uint8_t Ratio)
{
  RGBLed.GreenPwmRatio = Ratio;
  rgb_led_UpdateLeds();
}
uint8_t rgb_led_getGreenLedRatio(void)
{ 
  return RGBLed.GreenPwmRatio;
}

void rgb_led_SetRedLedRatio(uint8_t Ratio)
{
  RGBLed.RedPwmRatio = Ratio;
  rgb_led_UpdateLeds();
}
uint8_t rgb_led_getRedLedRatio(void)
{ 
  return RGBLed.RedPwmRatio;
}

/***************** local functions *******************/
static void rgb_led_UpdateLeds(void)
{
  pwm_motor_set(&(RGBLed.RedPwm), (PWM_MOTOR_MAX * (int16_t)RGBLed.RedPwmRatio)/255); 
  pwm_motor_set(&(RGBLed.GreenPwm), (PWM_MOTOR_MAX * (int16_t)RGBLed.GreenPwmRatio)/255); 
  pwm_motor_set(&(RGBLed.BluePwm), (PWM_MOTOR_MAX * (int16_t)RGBLed.BluePwmRatio)/255); 
}

