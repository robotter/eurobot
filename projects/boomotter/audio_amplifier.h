#ifndef _AUDIO_AMPLIFIER_H_
#define _AUDIO_AMPLIFIER_H_

#include <stdint.h>

typedef enum {
  AMPLI_GAIN_20DB,
  AMPLI_GAIN_26DB,
  AMPLI_GAIN_32DB,
  AMPLI_GAIN_36DB,
} amplifier_gain_t;


/// Initialize port pin and set gain to GAIN_20DB
void amplifier_init(void);

/// Set amplifier gain
void amplifier_set_gain(amplifier_gain_t gain);

/// Update shutdown state
void amplifier_shutdown(bool shutdown);

/// Update mute state
void amplifier_mute(bool mute);


#endif //_AUDIO_AMPLIFIER_H_
