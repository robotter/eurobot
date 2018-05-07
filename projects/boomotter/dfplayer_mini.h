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

typedef enum {
  // repeat mode is acting weird, values might be buggy
  DF_REPEAT_NORMAL = 0,
  DF_REPEAT_FOLDER = 1,
  DF_REPEAT_SINGLE = 2,
  DF_REPEAT_RANDOM = 3,
  DF_REPEAT_OFF = 0xf,
} dfplayer_repeat_mode_t;

/// specify volume
void dfplayer_set_volume(uint8_t volume);
/// pause play
void dfplayer_pause(void);
/// play current track
void dfplayer_play(void);
/// reset dfplayer mini
void dfplayer_reset(void);
/// play one specific track
void dfplayer_play_track(uint16_t track_id);
/// Return true if dfplayer is busy
bool dfplayer_is_busy(void);
/// Configure repeat mode
void dfplayer_set_repeat(dfplayer_repeat_mode_t mode);
void dfplayer_set_equalizer(uint8_t eq);
void dfplayer_normal_mode(void);

#endif//_DFPLAYER_MINI_H_
