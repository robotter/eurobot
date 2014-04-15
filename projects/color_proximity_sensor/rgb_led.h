#ifndef RGB_LED_H
#define RGB_LED_H

/* @biref initialise led port, set default pwm value for rgb led */
void rgb_led_Init(void);

/* @brief set all rgb leds on led on */
void rgb_led_SetLedOn(void);

/* @brief set all rgb leds on led off */
void rgb_led_SetLedOff(void);

/* @brief blue rgb led pwm ratio accessors
 * @param Radio duty cycle of the blue led must be between 0 aand 255
 * */ 
void rgb_led_SetBlueLedRatio(uint8_t Ratio);
uint8_t rgb_led_getBlueLedRatio(void);

/* @brief green rgb led pwm ratio accessors
 * @param Radio duty cycle of the green led must be between 0 and 255
 * */ 
void rgb_led_SetGreenLedRatio(uint8_t Ratio);
uint8_t rgb_led_getGreenLedRatio(void);

/* @brief Red rgb led pwm ratio accessors
 * @param Radio duty cycle of the Red led must be between 0 and 255
 * */ 
void rgb_led_SetRedLedRatio(uint8_t Ratio);
uint8_t rgb_led_getRedLedRatio(void);

#endif //RGB_LED_H 
