#include <avr/io.h>
#include "bumper.h"

#define FRONT_BUMPER_DDR      DDRD
#define FRONT_BUMPER_PIN      PIND
#define FRONT_BUMPER_PORT     PORTD
#define FRONT_BUMPER_PIN_NB   2

#define REAR_BUMPER_DDR       DDRA
#define REAR_BUMPER_PIN       PINA
#define REAR_BUMPER_PORT      PORTA
#define REAR_BUMPER_PIN_NB    1

#define HOOK_BUMPER_DDR       DDRA
#define HOOK_BUMPER_PIN       PINA
#define HOOK_BUMPER_PORT      PORTA
#define HOOK_BUMPER_PIN_NB    0


typedef enum {
  FRONT_BUMPER,
  REAR_BUMPER,
  HOOK_BUMPER,
  // must be the last element
  BUMPER_NB
} bumper_id_t;

static uint8_t bumper_cnt[BUMPER_NB];


void bumper_init(void)
{
  // set pins as inputs
  FRONT_BUMPER_DDR &= _BV(FRONT_BUMPER_PIN_NB);
  REAR_BUMPER_DDR &= _BV(REAR_BUMPER_PIN_NB);
  HOOK_BUMPER_DDR &= _BV(HOOK_BUMPER_PIN_NB);

  // activate pullups
  FRONT_BUMPER_PORT |= _BV(FRONT_BUMPER_PIN_NB);
  REAR_BUMPER_PORT |= _BV(REAR_BUMPER_PIN_NB);
  HOOK_BUMPER_PORT |= _BV(HOOK_BUMPER_PIN_NB);
}

uint8_t is_front_bumper_pushed(void)
{
  return bumper_cnt[FRONT_BUMPER] > 10;
}

uint8_t is_rear_bumper_pushed(void)
{
  return bumper_cnt[REAR_BUMPER] > 10;
}

uint8_t is_hook_lifted_up(void)
{
  return bumper_cnt[HOOK_BUMPER] > 10;
}


void bumper_manage(void)
{
  // filter FRONT bumper
  if ((FRONT_BUMPER_PIN & _BV(FRONT_BUMPER_PIN_NB)) == 0)
  {
    if (bumper_cnt[FRONT_BUMPER] < 0xff)
    {
      bumper_cnt[FRONT_BUMPER]++;
    }
  }
  else
  {
    bumper_cnt[FRONT_BUMPER] = 0;
  }

  // filter REAR bumper
  if ((REAR_BUMPER_PIN & _BV(REAR_BUMPER_PIN_NB)) == 0)
  {
    if (bumper_cnt[REAR_BUMPER] < 0xff)
    {
      bumper_cnt[REAR_BUMPER]++;
    }
  }
  else
  {
    bumper_cnt[REAR_BUMPER] = 0;
  }

  // filter HOOK bumper
  if ((HOOK_BUMPER_PIN & _BV(HOOK_BUMPER_PIN_NB)) == 0)
  {
    if (bumper_cnt[HOOK_BUMPER] < 0xff)
    {
      bumper_cnt[HOOK_BUMPER]++;
    }
  }
  else
  {
    bumper_cnt[HOOK_BUMPER] = 0;
  }

}

