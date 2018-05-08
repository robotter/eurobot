#ifndef _DFPLAYER_MINI_H_
#define _DFPLAYER_MINI_H_

#include <stdint.h>
#include <stdbool.h>


/// Initialize the dfplayer mini
void dfplayer_init(void);

/// Callback for dfplayer_handle_input()
typedef void (*dfplayer_handler_t)(uint8_t cmd, uint16_t param);

/** @brief Handle dfplayer UART input
 *
 * The callback will be called for each available message.
 */
void dfplayer_handle_input(dfplayer_handler_t cb);

/// Send a raw command
void dfplayer_send_cmd(uint8_t cmd, uint16_t param);

/// Equalizer mode
typedef enum {
  DF_EQUALIZER_NORMAL = 0,
  DF_EQUALIZER_POP = 1,
  DF_EQUALIZER_ROCK = 2,
  DF_EQUALIZER_JAZZ = 3,
  DF_EQUALIZER_CLASSIC = 4,
  DF_EQUALIZER_BASE = 4,
} dfplayer_equalizer_t;


/// Play next track, loop after last
void dfplayer_next(void);
/// Play previous track, loop before first
void dfplayer_previous(void);
/// Play a track by ID (1-2999, storage order)
void dfplayer_play_track_id(uint16_t track_id);
/// Increase volume by 1
void dfplayer_volume_up(void);
/// Decrease volume by 1
void dfplayer_volume_down(void);
/// Set volume (0-30)
void dfplayer_set_volume(uint8_t volume);
/// Set equalizer
void dfplayer_set_equalizer(dfplayer_equalizer_t eq);
/// Repeat a track by ID (1-2999, storage order)
void dfplayer_repeat_track(uint16_t track_id);
/// Reset player (track, volume)
void dfplayer_reset(void);
/// Play/resume current track
void dfplayer_play(void);
/// Pause current track
void dfplayer_pause(void);
/// Play a track, by folder and number (`XX/YYY.mp3`)
void dfplayer_play_folder_track(uint8_t folder, uint8_t num);
/// Loop all tracks, restart at track 1
void dfplayer_loop_all(void);
/// Play track from `mp3` directory (`mp3/YYYY.mp3`, 0-2999)
void dfplayer_play_mp3_track(uint16_t num);
/// Stop playing current track
void dfplayer_stop(void);
/// Loop given folder (in storage order)
void dfplayer_loop_folder(uint8_t num);
/// Play all tracks in random order, always starts at track 1
void dfplayer_random_all(void);
/// Loop current track
void dfplayer_loop_current(void);
/// Set pause (on/off)
void dfplayer_set_pause(bool pause);

/// Return true if dfplayer is busy
bool dfplayer_is_busy(void);

#endif//_DFPLAYER_MINI_H_
