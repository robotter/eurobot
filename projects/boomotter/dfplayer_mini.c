#include <stdbool.h>
#include <avarix/portpin.h>
#include <uart/uart.h>
#include "dfplayer_mini.h"
#include "dfplayer_mini_defs.h"

void dfplayer_send(dfplayer_frame_t *frame);
void dfplayer_populate_buffer(dfplayer_frame_t *frame, uint8_t *buf, uint8_t buf_size);
uint8_t dfplayer_set_source(dfplayer_storage_src_t src);


void dfplayer_init(void)
{
  portpin_dirclr(&DFPLAYER_BUSY_PP);
 
  dfplayer_set_source(DF_STORAGE_SDFLASH);
}


void dfplayer_send(dfplayer_frame_t *frame)
{
  uint8_t buf[10];

  dfplayer_populate_buffer(frame, buf, sizeof(buf));

  for (uint8_t  it=0; it < sizeof(buf); it ++)
    uart_send(uartD0, buf[it]);
}

void dfplayer_populate_buffer(dfplayer_frame_t *frame, uint8_t *buf, uint8_t buf_size)
{
  if (buf_size >= 10)
  {
    buf[0] = DFPLAYER_START_FRAME;
    buf[1] = frame->version;
    buf[2] = 6;
    buf[3] = (uint8_t)frame->cmd;
    buf[4] = frame->feedback;
    buf[5] = frame->para1;
    buf[6] = frame->para2;
    
    uint16_t checksum = 0;

    for (uint8_t it =1; it <=6; it ++)
      checksum += buf[it];
    checksum = -checksum;

    frame->checksum = checksum;

    buf[7] = (uint8_t)(frame->checksum>>8);
    buf[8] = (uint8_t)(frame->checksum);
    buf[9] = DFPLAYER_STOP_FRAME;
  }
}

uint8_t dfplayer_set_source(dfplayer_storage_src_t src)
{
  dfplayer_frame_t frame = {.version = 0xff,
                            .cmd = DF_SET_STORAGE_SOURCE,
                            .feedback = 0x00,
                            .para1 = 0,
                            .para2 = (uint8_t)src};
  dfplayer_send(&frame);
  return 0;

}

uint8_t dfplayer_specify_volume(uint8_t volume)
{
  if (volume > 30)
    volume = 30;

  dfplayer_frame_t frame = {.version = 0xff,
                            .cmd = DF_SET_VOLUME,
                            .feedback = 0x00,
                            .para1 = 0,
                            .para2 = volume};
  dfplayer_send(&frame);
  return 0;
}

uint8_t dfplayer_pause(void)
{
  dfplayer_frame_t frame = {.version = 0xff,
                            .cmd = DF_PAUSE,
                            .feedback = 0x00,
                            .para1 = 0x00,
                            .para2 = 0x00};
  dfplayer_send(&frame);
  return 0;
}

uint8_t dfplayer_play(void)
{
  dfplayer_frame_t frame = {.version = 0xff,
                            .cmd = DF_PLAY,
                            .feedback = 0x00,
                            .para1 = 0x00,
                            .para2 = 0x00};
  dfplayer_send(&frame);
  return 0;
}

uint8_t dfplayer_reset(void)
{
  dfplayer_frame_t frame = {.version = 0xff,
                            .cmd = DF_RESET_MODULE,
                            .feedback = 0x00,
                            .para1 = 0x00,
                            .para2 = 0x00};
  dfplayer_send(&frame);
  return 0;
}

uint8_t dfplayer_play_track(uint16_t track_id)
{
  dfplayer_frame_t frame = {.version = 0xff,
                            .cmd = DF_READ_TRACK,
                            .feedback = 0x00,
                            .para1 = (uint8_t)(track_id>>8),
                            .para2 = (uint8_t)(track_id)};
  dfplayer_send(&frame);
  return 0;
}

uint8_t dfplayer_is_busy(void)
{
  return portpin_in(&DFPLAYER_BUSY_PP) == false;
}
