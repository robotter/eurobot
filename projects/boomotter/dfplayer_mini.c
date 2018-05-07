#include <avarix/portpin.h>
#include <uart/uart.h>
#include "dfplayer_mini.h"
#include "dfplayer_mini_defs.h"
#include "config.h"

void dfplayer_send(dfplayer_frame_t *frame);
void dfplayer_set_source(dfplayer_storage_src_t src);


void dfplayer_init(void)
{
  portpin_dirclr(&DFPLAYER_BUSY_PP);

  dfplayer_set_source(DF_STORAGE_SDFLASH);
  dfplayer_normal_mode();
}


void dfplayer_send(dfplayer_frame_t *frame)
{
  uint8_t buf[10];
  buf[0] = DFPLAYER_START_FRAME;
  buf[1] = frame->version;
  buf[2] = 6;
  buf[3] = frame->cmd;
  buf[4] = frame->feedback;
  buf[5] = frame->para1;
  buf[6] = frame->para2;

  uint16_t checksum = 0;
  for(uint8_t it = 1; it <= 6; it++) {
    checksum += buf[it];
  }
  checksum = -checksum;

  frame->checksum = checksum;

  buf[7] = frame->checksum >> 8;
  buf[8] = frame->checksum;
  buf[9] = DFPLAYER_STOP_FRAME;

  for(uint8_t it = 0; it < sizeof(buf); it++) {
    uart_send(UART_DFPLAYER, buf[it]);
  }
}

void dfplayer_cmd(uint8_t cmd, uint16_t param)
{
  dfplayer_frame_t frame = {
    .version = 0xff,
    .cmd = cmd,
    .feedback = 0x00,
    .para1 = param >> 8,
    .para2 = param,
  };
  dfplayer_send(&frame);
}

void dfplayer_set_source(dfplayer_storage_src_t src)
{
  dfplayer_cmd(DF_SET_STORAGE_SOURCE, src);
}

void dfplayer_set_volume(uint8_t volume)
{
  if(volume > 30) {
    volume = 30;
  }
  dfplayer_cmd(DF_SET_VOLUME, volume);
}

void dfplayer_pause(void)
{
  dfplayer_cmd(DF_PAUSE, 0);
}

void dfplayer_play(void)
{
  dfplayer_cmd(DF_PLAY, 0);
}

void dfplayer_reset(void)
{
  dfplayer_cmd(DF_RESET_MODULE, 0);
}

void dfplayer_play_track(uint16_t track_id)
{
  dfplayer_cmd(DF_READ_TRACK, track_id);
}

void dfplayer_set_equalizer(uint8_t eq)
{
  dfplayer_cmd(DF_RESET_MODULE, eq);
}

void dfplayer_normal_mode(void)
{
  dfplayer_cmd(DF_NORMAL_WORKING, 0);
}

bool dfplayer_is_busy(void)
{
  return !portpin_in(&DFPLAYER_BUSY_PP);
}

