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

  dfplayer_set_source(DF_STORAGE_SDFLASH);
  dfplayer_normal_mode();
}


void dfplayer_send_cmd(uint8_t cmd, uint16_t param)
{
  ROME_LOGF(&rome_intf, DEBUG, "send MP3 command: %02x %04x (%u)", cmd, param, param);

  uint8_t buf[10];
  buf[0] = DFPLAYER_START_BYTE;
  buf[1] = 0xff;  // version
  buf[2] = 6;  // payload length
  buf[3] = cmd;
  buf[4] = 0;  // feedback
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
      cb(buf[3-1], (buf[4-1] << 8) | buf[5-1]);
    }
  }
}


void dfplayer_set_source(dfplayer_storage_src_t src)
{
  dfplayer_send_cmd(DF_SET_STORAGE_SOURCE, src);
}

void dfplayer_set_volume(uint8_t volume)
{
  if(volume > 30) {
    volume = 30;
  }
  dfplayer_send_cmd(DF_SET_VOLUME, volume);
}

void dfplayer_pause(void)
{
  dfplayer_send_cmd(DF_PAUSE, 0);
}

void dfplayer_play(void)
{
  dfplayer_send_cmd(DF_PLAY, 0);
}

void dfplayer_reset(void)
{
  dfplayer_send_cmd(DF_RESET_MODULE, 0);
}

void dfplayer_play_track(uint16_t track_id)
{
  dfplayer_send_cmd(DF_READ_TRACK, track_id);
}

void dfplayer_set_equalizer(uint8_t eq)
{
  dfplayer_send_cmd(DF_RESET_MODULE, eq);
}

void dfplayer_normal_mode(void)
{
  dfplayer_send_cmd(DF_NORMAL_WORKING, 0);
}

bool dfplayer_is_busy(void)
{
  return !portpin_in(&DFPLAYER_BUSY_PP);
}

