#ifndef LED_CONFIG_H
#define LED_CONFIG_H

// pinout
#define LED_PC6  PORTPIN(C,6u)
#define LED_PC7  PORTPIN(C,7u)


/// Timer/Counter used for blue led PWM
#define BLUE_LED_PWM_TC  TCC0
/// Timer/Counter channel used for blue led PWM
#define BLUE_LED_PWM_CH  'A'

/// Timer/Counter used for green led PWM
#define GREEN_LED_PWM_TC  TCC0
/// Timer/Counter channel used for green led PWM
#define GREEN_LED_PWM_CH 'B'

/// Timer/Counter used for red led PWM
#define RED_LED_PWM_TC  TCC1
/// Timer/Counter channel used for red led PWM
#define RED_LED_PWM_CH  'A'

/// led pwm frequency in Hz
#define RGB_LED_PWM_FREQUENCY_HZ  1000u

/// blue led default PWM ratio (between 0 and 255)
#define DEFAULT_BLUE_LED_PWM_RATIO  100u
/// green led default PWM ratio (between 0 and 255)
#define DEFAULT_GREEN_LED_PWM_RATIO  100
/// red led default PWM ratio (between 0 and 255)
#define DEFAULT_RED_LED_PWM_RATIO  100


#endif //LED_CONFIG_H
