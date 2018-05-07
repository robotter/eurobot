#include "dfplayer_mini.h"
#include "amplifier.h"
#include "audio.h"

void audio_init(void)
{
  dfplayer_init();
  amplifier_init();
}

void audio_on(void)
{
  amplifier_shutdown(false);
  amplifier_mute(false);
  //amplifier_set_gain(GAIN_26DB);
}

