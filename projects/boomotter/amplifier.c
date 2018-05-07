#include <avarix/portpin.h>
#include "amplifier.h"
#include "config.h"


void amplifier_init(void)
{
  portpin_dirset(&AMPLI_GAIN_0_PP);
  portpin_dirset(&AMPLI_GAIN_1_PP);
  portpin_dirset(&AMPLI_MUTE_PP);
  portpin_dirset(&AMPLI_SHUTDOWN_PP);

  amplifier_shutdown(true);
  amplifier_mute(true);
  amplifier_set_gain(AMPLI_GAIN_20DB);
}


void amplifier_set_gain(amplifier_gain_t gain)
{
  switch(gain) {
    case AMPLI_GAIN_20DB:
      portpin_outclr(&AMPLI_GAIN_0_PP);
      portpin_outclr(&AMPLI_GAIN_1_PP);
      break;
    case AMPLI_GAIN_26DB:
      portpin_outset(&AMPLI_GAIN_0_PP);
      portpin_outclr(&AMPLI_GAIN_1_PP);
      break;
    case AMPLI_GAIN_32DB:
      portpin_outclr(&AMPLI_GAIN_0_PP);
      portpin_outset(&AMPLI_GAIN_1_PP);
      break;
    case AMPLI_GAIN_36DB:
      portpin_outset(&AMPLI_GAIN_0_PP);
      portpin_outset(&AMPLI_GAIN_1_PP);
      break;
    default:
      break;
  }
}

void amplifier_mute(bool shutdown)
{
  if(shutdown) {
    portpin_outset(&AMPLI_MUTE_PP);
  } else {
    portpin_outclr(&AMPLI_MUTE_PP);
  }
}

void amplifier_shutdown(bool mute)
{
  if(mute) {
    portpin_outclr(&AMPLI_SHUTDOWN_PP);
  } else {
    portpin_outset(&AMPLI_SHUTDOWN_PP);
  }
}


