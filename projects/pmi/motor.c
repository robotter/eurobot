#include <avr/io.h>
#include "motor.h"

#define DIR_DDR     DDRD
#define DIR_PORT    PORTD
#define DIR_PIN_NB  1


void motor_init(void)
{
  // set direction pin as output
  DIR_DDR |= _BV(DIR_PORT);

  // no irq
  TIMSK &= ~((1 << OCIE0B) | (1 << OCIE0A) | (1 << TOIE0));

  // reset motor pwm
  motor_set_pwm(0);

  // finally enable and connect timer to port in pwm mode

  // IC0B in pwm mode and fast pwm mode (period set to 0xFF)
  TCCR0B = (0 << WGM02) | (0 << CS02) | (0 << CS01) | (1 << CS00);
  TCCR0A = (1 << COM0A1) | (0 << COM0A0) | (1 << WGM11) | (1 << WGM10);

  DDRB |= _BV(2);
}

void motor_set_pwm(int16_t pwm)
{
  uint8_t abs_pwm = 0;
  if (pwm >= 0)
  {
    abs_pwm = (uint8_t)pwm;
    DIR_PORT &= ~_BV(DIR_PIN_NB);
  }
  else
  {
    abs_pwm = (uint8_t)(255 + pwm);
    DIR_PORT |= _BV(DIR_PIN_NB);
  }

  OCR0A = abs_pwm;
}

