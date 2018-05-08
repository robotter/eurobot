#ifndef _DFPLAYER_MINI_DEFS_H_
#define _DFPLAYER_MINI_DEFS_H_

#include <avarix.h>
#include <stdint.h>

typedef enum {
  // commands (no response)
  DF_NEXT = 0x01,
  DF_PREVIOUS = 0x02,
  DF_PLAY_TRACK_ID = 0x03,
  DF_VOLUME_UP = 0x04,
  DF_VOLUME_DOWN = 0x05,
  DF_SET_VOLUME = 0x06,
  DF_SET_EQUALIZER = 0x07,
  DF_REPEAT_TRACK_ID = 0x08,
  DF_SET_STORAGE_SOURCE = 0x09,
  DF_SHUTDOWN = 0x0A,
  DF_NORMAL_WORKING = 0x0B,  // unknown use
  DF_RESET_MODULE = 0x0C,
  DF_PLAY = 0x0D,
  DF_PAUSE = 0x0E,
  DF_PLAY_FOLDER_TRACK = 0x0F,
  DF_VOLUME_ADJUST_SET = 0x10,  // unknown use
  DF_LOOP_ALL = 0x11,
  DF_PLAY_MP3_TRACK = 0x12,
  DF_STOP = 0x16,
  DF_LOOP_FOLDER = 0x17,
  DF_RANDOM_ALL = 0x18,
  DF_LOOP_CURRENT = 0x19,
  DF_SET_PAUSE = 0x1A,

  // events
  DF_MEDIUM_INSERTED = 0x3A,
  DF_MEDIUM_EJECTED = 0x3B,
  DF_TRACK_END_U = 0x3C,
  DF_TRACK_END_SD = 0x3D,
  DF_TRACK_END_NORFLASH = 0x3E,
  DF_INIT_PARAMS = 0x3F,  // after reset, return some status 

  // generic replies
  DF_ERROR = 0x40,  // error code are unknown
  DF_REPLY = 0x41,  // returned when "feedback" is enabled, 0x00: no error, other: unknown

  // queries and their replies
  DF_GET_STATUS = 0x42,
  DF_GET_VOLUME = 0x43,
  DF_GET_EQUALIZER = 0x44,
  DF_GET_PLAYBACK_MODE = 0x45,  // 02 if DF_REPEAT_TRACK_ID in use
  DF_GET_SW_VERSION = 0x46,
  DF_GET_TRACK_COUNT_U = 0x47,
  DF_GET_TRACK_COUNT_SD = 0x48,
  DF_GET_TRACK_COUNT_NORFLASH = 0x49,
  DF_GET_CURRENT_TRACK = 0x4B, // also 0x4C, 0x4D
  DF_GET_FOLDER_TRACK_COUNT = 0x4E,  // param: folder number
  DF_GET_FOLDER_COUNT = 0x4F,  // param: 1, folder count, including root

} dfplayer_cmd_t;

typedef enum{
  DF_STORAGE_U = 0x00,
  DF_STORAGE_SD = 0x01,
  DF_STORAGE_AUX = 0x02,
  DF_STORAGE_SLEEP = 0x03,
  DF_STORAGE_NORFLASH = 0x04,
} dfplayer_storage_src_t;

#define DFPLAYER_START_BYTE  0x7E
#define DFPLAYER_STOP_BYTE  0xEF

#endif //_DFPLAYER_MINI_DEFS_H_
