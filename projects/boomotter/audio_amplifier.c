#include <avarix/portpin.h>
#include "audio_amplifier.h"

void amplifier_init(void)
{
  portpin_dirset(&AMPLI_GAIN_0_PP);
  portpin_dirset(&AMPLI_GAIN_1_PP);
  portpin_dirset(&AMPLI_MUTE_PP);
  portpin_dirset(&AMPLI_SHUTDOWN_PP);

  amplifier_set_shutdown(1);
  amplifier_set_mute(1);
  amplifier_set_gain(GAIN_20DB);
}


void amplifier_set_gain(amplifier_gain_t gain)
{
  switch(gain)
  {
    case GAIN_20DB:
          portpin_outclr(&AMPLI_GAIN_0_PP);
          portpin_outclr(&AMPLI_GAIN_1_PP);
          break;
    case GAIN_26DB:
          portpin_outset(&AMPLI_GAIN_0_PP);
          portpin_outclr(&AMPLI_GAIN_1_PP);
          break;
    case GAIN_32DB:
          portpin_outclr(&AMPLI_GAIN_0_PP);
          portpin_outset(&AMPLI_GAIN_1_PP);
          break;
    case GAIN_36DB:
          portpin_outset(&AMPLI_GAIN_0_PP);
          portpin_outset(&AMPLI_GAIN_1_PP);
          break;
    default: break;
  }
}

void amplifier_set_mute(uint8_t state)
{
  if (state)
    portpin_outset(&AMPLI_MUTE_PP);
  else
    portpin_outclr(&AMPLI_MUTE_PP);
}

void amplifier_set_shutdown(uint8_t state)
{
  if (state)
    portpin_outset(&AMPLI_SHUTDOWN_PP);
  else
    portpin_outclr(&AMPLI_SHUTDOWN_PP);
}


