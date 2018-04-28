#ifndef _DFPLAYER_MINI_H_
#define _DFPLAYER_MINI_H_

#include <stdint.h>
#include "dfplayer_mini_config.h"

/// Initialize the dfplayer mini
void dfplayer_init(void);

/// specify volume
uint8_t dfplayer_specify_volume(uint8_t volume);

/// pause play
uint8_t dfplayer_pause(void);

/// play current track
uint8_t dfplayer_play(void);

/// reset dfplayer mini
uint8_t dfplayer_reset(void);

/// play one specific track
uint8_t dfplayer_play_track(uint16_t track_id);

/// return 0 if dfplayer is not busy
uint8_t dfplayer_is_busy(void);

#endif//_DFPLAYER_MINI_H_
