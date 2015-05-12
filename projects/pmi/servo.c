#include <avr/io.h>
#include "config.h"
#include "servo.h"

// middle position of analog servo
#define SERVO_POS_MIDDLE ((SERVO_POS_MIN + SERVO_POS_MAX)/2)

// frequency of pwm signal (in HZ)
#define SERVO_PWM_FREQ_HZ   50


void servo_init(void)
{
  // no irq
  TIMSK &= ~((1 << OCIE1A) | (1 << OCIE1B) | (1 << TOIE1) | (1 << ICIE1));

  // set pwm frequency
  ICR1 = (CPU_FREQ / SERVO_PWM_FREQ_HZ);

  // set default servo position to middle
  servo_set(SERVO_POS_MIDDLE);

  // finally enable and connect timer to port in pwm mode

  // OC1A in pwm mode and fast pwm mode, period defined by ICR1
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (0 << CS12) | (0 << CS11) | (1 << CS10);
  TCCR1A = (1 << COM1A1) | (0 << COM1A0) | (1 << WGM11) | (0 << WGM10);

  // OC1A must be set as output pin
  DDRB |= _BV(3);
}

void servo_set(uint16_t pos)
{
  // servo position limitation (pwm on duration must be between 1 and 2 ms)
  if (pos > SERVO_POS_MAX)
  {
    pos = SERVO_POS_MAX;
  }
  else if (pos < SERVO_POS_MIN)
  {
    pos = SERVO_POS_MIN;
  }

  OCR1A = pos;
}

