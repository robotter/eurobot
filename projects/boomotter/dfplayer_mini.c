#include <clock/clock.h>
#include <util/delay.h>
#include <avarix/portpin.h>
#include <uart/uart.h>
#include <rome/rome.h>
#include "dfplayer_mini.h"
#include "dfplayer_mini_defs.h"
#include "config.h"

extern rome_intf_t rome_intf;

void dfplayer_set_source(dfplayer_storage_src_t src);

static uint16_t dfplayer_checksum(uint8_t *buf, uint8_t n)
{
  uint16_t checksum = 0;
  for(uint8_t i = 0; i < n; i++) {
    checksum += buf[i];
  }
  return -checksum;
}


void dfplayer_init(void)
{
  portpin_dirclr(&DFPLAYER_BUSY_PP);
  dfplayer_set_source(DF_STORAGE_SD);
}


void dfplayer_send_cmd(uint8_t cmd, uint16_t param)
{
  ROME_LOGF(&rome_intf, DEBUG, "send MP3 command: %02x %04x (%u)", cmd, param, param);

  uint8_t buf[10];
  buf[0] = DFPLAYER_START_BYTE;
  buf[1] = 0xff;  // version
  buf[2] = 6;  // payload length
  buf[3] = cmd;
  buf[4] = 1;  // feedback
  buf[5] = param >> 8;
  buf[6] = param;

  uint16_t checksum = dfplayer_checksum(buf+1, 6);
  buf[7] = checksum >> 8;
  buf[8] = checksum;
  buf[9] = DFPLAYER_STOP_BYTE;

  for(uint8_t it = 0; it < sizeof(buf); it++) {
    uart_send(UART_DFPLAYER, buf[it]);
  }
}

void dfplayer_handle_input(dfplayer_handler_t cb)
{
  static uint8_t buf[10-1];  // don't store start byte
  static uint8_t pos;

  for(;;) {
    // start byte
    while(pos < 1) {
      switch(uart_recv_nowait(UART_DFPLAYER)) {
        case -1:
          return;
        case DFPLAYER_START_BYTE:
          pos++;
          break;
        default:
          pos = 0;
      }
    }

    // version (must be 0xff)
    if(pos < 2) {
      int c = uart_recv_nowait(UART_DFPLAYER);
      if(c == -1) {
        return;
      } else if(c != 0xff) {
        pos = 0;
        continue;
      }
      buf[pos-1] = c;
      pos++;
    }

    // payload size (must be 6)
    if(pos < 3) {
      int c = uart_recv_nowait(UART_DFPLAYER);
      if(c == -1) {
        return;
      } else if(c != 6) {
        pos = 0;
        continue;
      }
      buf[pos-1] = c;
      pos++;
    }

    // payload, checksum and stop byte
    while(pos < 10) {
      int ret = uart_recv_nowait(UART_DFPLAYER);
      if(ret == -1) {
        return;
      }
      buf[pos-1] = ret;
      pos++;
    }

    pos = 0;  // reset pos now to be able to 'continue' directly
    if(buf[9-1] != DFPLAYER_STOP_BYTE) {
      continue;
    }
    uint16_t checksum_got = (buf[7-1] << 8) | buf[8-1];
    uint16_t checksum_exp = dfplayer_checksum(buf, 6);
    if(checksum_got == checksum_exp) {
      uint8_t cmd = buf[3-1];
      uint16_t param = (buf[5-1] << 8) | buf[6-1];
      ROME_LOGF(&rome_intf, DEBUG, "dfplayer reply: %02x %04x (%u)", cmd, param, param);
      if(cb) {
        cb(cmd, param);
      }
    }
  }
}


void dfplayer_next(void)
{
  dfplayer_send_cmd(DF_NEXT, 0);
}

void dfplayer_previous(void)
{
  dfplayer_send_cmd(DF_PREVIOUS, 0);
}

void dfplayer_play_track(uint16_t track_id)
{
  dfplayer_send_cmd(DF_PLAY_TRACK_ID, track_id);
}

void dfplayer_volume_up(void)
{
  dfplayer_send_cmd(DF_VOLUME_UP, 0);
}

void dfplayer_volume_down(void)
{
  dfplayer_send_cmd(DF_VOLUME_DOWN, 0);
}

void dfplayer_set_volume(uint8_t volume)
{
  if(volume > 30) {
    volume = 30;
  }
  dfplayer_send_cmd(DF_SET_VOLUME, volume);
}

void dfplayer_set_equalizer(dfplayer_equalizer_t eq)
{
  dfplayer_send_cmd(DF_SET_EQUALIZER, eq);
}

void dfplayer_repeat_track(uint16_t track_id)
{
  dfplayer_send_cmd(DF_REPEAT_TRACK_ID, track_id);
}

void dfplayer_set_source(dfplayer_storage_src_t src)
{
  // note: seems to be overriden by automatic detection
  dfplayer_send_cmd(DF_SET_STORAGE_SOURCE, src);
}

void dfplayer_reset(void)
{
  dfplayer_send_cmd(DF_RESET_MODULE, 0);
}

void dfplayer_play(void)
{
  dfplayer_send_cmd(DF_PLAY, 0);
}

void dfplayer_pause(void)
{
  dfplayer_send_cmd(DF_PAUSE, 0);
}

void dfplayer_play_folder_track(uint8_t folder, uint8_t num)
{
  dfplayer_send_cmd(DF_PLAY_FOLDER_TRACK, (folder << 8) | num);
}

void dfplayer_loop_all(void)
{
  dfplayer_send_cmd(DF_LOOP_ALL, 0);
}

void dfplayer_play_mp3_track(uint16_t num)
{
  dfplayer_send_cmd(DF_PLAY_MP3_TRACK, num);
}

void dfplayer_stop(void)
{
  dfplayer_send_cmd(DF_STOP, 0);
}

void dfplayer_loop_folder(uint8_t num)
{
  dfplayer_send_cmd(DF_LOOP_FOLDER, num);
}

void dfplayer_random_all(void)
{
  dfplayer_send_cmd(DF_RANDOM_ALL, 0);
}

void dfplayer_loop_current(void)
{
  dfplayer_send_cmd(DF_LOOP_CURRENT, 0);
}

void dfplayer_set_pause(bool pause)
{
  dfplayer_send_cmd(DF_SET_PAUSE, pause ? 1 : 0);
}


bool dfplayer_is_busy(void)
{
  return !portpin_in(&DFPLAYER_BUSY_PP);
}

