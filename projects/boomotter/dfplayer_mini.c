#include <avarix/portpin.h>
#include <uart/uart.h>
#include "dfplayer_mini.h"
#include "dfplayer_mini_defs.h"
#include "dfplayer_mini_config.h"
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

void dfplayer_set_source(dfplayer_storage_src_t src)
{
  dfplayer_frame_t frame = {
    .version = 0xff,
    .cmd = DF_SET_STORAGE_SOURCE,
    .feedback = 0x00,
    .para1 = 0,
    .para2 = src,
  };
  dfplayer_send(&frame);
}

void dfplayer_set_volume(uint8_t volume)
{
  if(volume > 30) {
    volume = 30;
  }

  dfplayer_frame_t frame = {
    .version = 0xff,
    .cmd = DF_SET_VOLUME,
    .feedback = 0x00,
    .para1 = 0,
    .para2 = volume,
  };
  dfplayer_send(&frame);
}

void dfplayer_pause(void)
{
  dfplayer_frame_t frame = {
    .version = 0xff,
    .cmd = DF_PAUSE,
    .feedback = 0x00,
    .para1 = 0x00,
    .para2 = 0x00,
  };
  dfplayer_send(&frame);
}

void dfplayer_play(void)
{
  dfplayer_frame_t frame = {
    .version = 0xff,
    .cmd = DF_PLAY,
    .feedback = 0x00,
    .para1 = 0x00,
    .para2 = 0x00,
  };
  dfplayer_send(&frame);
}

void dfplayer_reset(void)
{
  dfplayer_frame_t frame = {
    .version = 0xff,
    .cmd = DF_RESET_MODULE,
    .feedback = 0x00,
    .para1 = 0x00,
    .para2 = 0x00,
  };
  dfplayer_send(&frame);
}

void dfplayer_play_track(uint16_t track_id)
{
  dfplayer_frame_t frame = {
    .version = 0xff,
    .cmd = DF_READ_TRACK,
    .feedback = 0x00,
    .para1 = track_id >> 8,
    .para2 = track_id,
  };
  dfplayer_send(&frame);
}

void dfplayer_set_equalizer(uint8_t eq)
{
  dfplayer_frame_t frame = {
    .version = 0xff,
    .cmd = DF_SET_EQUALIZER,
    .feedback = 0x00,
    .para1 = 0x00,
    .para2 = eq,
  };
  dfplayer_send(&frame);
}

void dfplayer_normal_mode(void)
{
  dfplayer_frame_t frame = {
    .version = 0xff,
    .cmd = DF_NORMAL_WORKING,
    .feedback = 0x00,
    .para1 = 0x00,
    .para2 = 0x00,
  };
  dfplayer_send(&frame);
}

bool dfplayer_is_busy(void)
{
  return !portpin_in(&DFPLAYER_BUSY_PP);
}
