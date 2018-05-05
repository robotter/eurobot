#ifndef _DFPLAYER_MINI_DEFS_H_
#define _DFPLAYER_MINI_DEFS_H_

#include <avarix.h>
#include <stdint.h>

typedef enum{
    DF_NEXT       = 0x01,
    DF_PREVIOUS   = 0x02,
    DF_READ_TRACK = 0x03,
    DF_INCREASE_VOLUME  = 0x04,
    DF_DECREASE_VOLUME  = 0x05,
    DF_SET_VOLUME       = 0x06,
    DF_SET_EQUALIZER    = 0x07,
    DF_SET_PLAY_MODE    = 0x08,
    DF_SET_STORAGE_SOURCE = 0x09,
    DF_SHUTDON          = 0x0A,
    DF_NORMAL_WORKING   = 0x0B,  
    DF_RESET_MODULE     = 0x0C,
    DF_PLAY             = 0x0D,
    DF_PAUSE            = 0x0E,
    DF_SET_PLAY_FOLDER  = 0x0F,
    DF_VOLUME_ADJUST_SET= 0x10,
    DF_REPEAT_PLAY      = 0x11,
    DF_REQUEST_RETRANSMIT = 0x40,
    DF_REQUEST_REPLY    = 0x41,
    DF_READ_STATUS      = 0x42,
    DF_READ_VOLUME      = 0x43,
    DF_READ_EQUALIZER   = 0x44,
    DF_READ_SW_VERSION  = 0x46,
    DF_READ_FILE_NB_IN_TF = 0x47,
    DF_READ_CURRENT_FILENAME = 0x4B,
} dfplayer_cmd_t;

typedef enum{
  DF_STORAGE_U        = 0x00,
  DF_STORAGE_SDFLASH  = 0x01,
  DF_STORAGE_AUX      = 0x02,
  DF_STORAGE_SLEEP    = 0x03,
  DF_STORAGE_NORFLASH = 0x04
}dfplayer_storage_src_t;

#define DFPLAYER_START_FRAME  0x7E
#define DFPLAYER_STOP_FRAME   0xEF
#define DFPLAYER_LEN          0x06

typedef struct{
  uint8_t version;
  dfplayer_cmd_t cmd;
  uint8_t feedback;
  uint8_t para1;
  uint8_t para2;
  uint16_t checksum;
} dfplayer_frame_t;



#endif //_DFPLAYER_MINI_DEFS_H_
