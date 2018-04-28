#ifndef _AUDIO_AMPLIFIER_H_
#define _AUDIO_AMPLIFIER_H_

#include <stdint.h>
#include "audio_amplifier_config.h"

typedef enum {GAIN_20DB, 
              GAIN_26DB, 
              GAIN_32DB, 
              GAIN_36DB
             } amplifier_gain_t;


/// initialize port pin and set gain to GAIN_20DB
void amplifier_init(void);

/// set amplifier gain (must be one of the amplifier_gain_t enum value)
void amplifier_set_gain(amplifier_gain_t gain);

/// update shutdown state (0 => enabled, everything else => disabled)
void amplifier_set_shutdown(uint8_t state);

/// update mute state (0=> enabled, everything else => mute)
void amplifier_set_mute(uint8_t state);


#endif //_AUDIO_AMPLIFIER_H_
