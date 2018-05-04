#include <avarix.h>
#include <avarix/intlvl.h>
#include <avarix/portpin.h>
#include <uart/uart.h>
#include <clock/clock.h>
#include <rome/rome.h>
#include "leds.h"
#include "dfplayer_mini.h"
#include "audio_amplifier.h"
#include "ws2812.h"
#include <util/delay.h>

#include "pic.c"

rome_intf_t rome_intf;

static void rome_handler(rome_intf_t *intf, const rome_frame_t *frame)
{
  #if 0
  switch(frame->mid) {
    case ROME_MID_R3D2_SET_ROTATION:
      r3d2_set_rotation(frame->r3d2_set_rotation.speed_rpm,
        frame->r3d2_set_rotation.threshold_percent);
      rome_reply_ack(intf, frame);
      break;

    case ROME_MID_R3D2_SET_BLIND_SPOT:
      r3d2_set_blind_spot(
        1.0*frame->r3d2_set_blind_spot.begin/1000,
        1.0*frame->r3d2_set_blind_spot.end/1000);
      rome_reply_ack(intf, frame);
      break;
        
    default:
      break;
  }
#else

#endif
}

#define UPPER_WIDTH   21
#define UPPER_HEIGHT  6
#define LOWER_WIDTH   11
#define LOWER_HEIGHT  10

static void invcpy(ws2812_pixel_t *dst, ws2812_pixel_t*src, size_t len) {
  for(size_t i=0; i<len; i++) {
    size_t j = len-i-1;
    dst[i] = src[j];
  }
}

static void screen_to_pixel(ws2812_pixel_t *screen, ws2812_pixel_t *pixels)
{
  const size_t SZ = sizeof(ws2812_pixel_t);
  const size_t UW = UPPER_WIDTH;
  const size_t LW = LOWER_WIDTH;

  pixels ++;
  memcpy(pixels, screen, UW*SZ);

  pixels += UW; screen += UW;
  invcpy(pixels,screen, UW);
  pixels += UW; screen += UW;
  memcpy(pixels, screen, UW*SZ);

  pixels += UW; screen += UW;
  invcpy(pixels,screen, UW);
  pixels += UW; screen += UW;
  memcpy(pixels, screen, UW*SZ);

  pixels += UW; screen += UW;
  invcpy(pixels,screen, UW);
  pixels += UW; screen += UW;
  memcpy(pixels, screen, LW*SZ);

  pixels += LW; screen += UW;
  invcpy(pixels,screen, LW);
  pixels += LW; screen += UW;
  memcpy(pixels, screen, LW*SZ);

  pixels += LW; screen += UW;
  invcpy(pixels,screen, LW);
  pixels += LW; screen += UW;
  memcpy(pixels, screen, LW*SZ);
 
  pixels += LW; screen += UW;
  invcpy(pixels,screen, LW);
  pixels += LW; screen += UW;
  memcpy(pixels, screen, LW*SZ);

  pixels += LW; screen += UW;
  invcpy(pixels,screen, LW);
  pixels += LW; screen += UW;
  memcpy(pixels, screen, LW*SZ);

  pixels += LW; screen += UW;
  invcpy(pixels,screen, LW);
  pixels += LW; screen += UW;
  memcpy(pixels, screen, LW*SZ);

}


int main(void)
{
  portpin_dirset(&LED_RUN_PP);
  portpin_dirset(&LED_ERROR_PP);
  portpin_dirset(&LED_COM_PP);


  portpin_outset(&LED_COM_PP);
  portpin_outset(&LED_RUN_PP);
  portpin_outset(&LED_ERROR_PP);
  clock_init();
  portpin_outclr(&LED_COM_PP);

  uart_init();
  uart_fopen(uartC0);
  portpin_outclr(&LED_RUN_PP);

  INTLVL_ENABLE_ALL();
  __asm__("sei");

  rome_intf_init(&rome_intf);
  rome_intf.uart = uartC0;
  rome_intf.handler = rome_handler;

  ROME_LOGF(&rome_intf, INFO, "RST.STATUS=%x booting...\n", RST.STATUS);
  RST.STATUS = 0;

  portpin_outclr(&LED_ERROR_PP);

  dfplayer_init();
  amplifier_init();

  ws2812_init();
  
  _delay_ms(500);
  dfplayer_specify_volume(10);

  amplifier_shutdown(0);
  amplifier_mute(0);
  amplifier_set_gain(GAIN_26DB);
  
  ws2812_pixel_t pixels[UPPER_WIDTH*UPPER_HEIGHT+LOWER_WIDTH*LOWER_HEIGHT+1];
  ws2812_pixel_t screen[UPPER_WIDTH*(UPPER_HEIGHT+LOWER_HEIGHT)];

  memset(pixels, 0, sizeof(pixels));
  memset(screen, 0, sizeof(screen));
  
  const int SH = UPPER_HEIGHT+LOWER_HEIGHT;
  const int SW = UPPER_WIDTH;

  portpin_outset(&LED_RUN_PP); 

  (void)SW;(void)SH;
  while(1) {
    rome_handle_input(&rome_intf);
    
    if (!dfplayer_is_busy())
    {
      dfplayer_play_track(10);
    }

    for(int j=0;j<SH;j++)
    for(int i=0;i<SW;i++) {
      uint8_t r = gimp_image.pixel_data[3*j*SW + 3*i + 0];
      uint8_t g = gimp_image.pixel_data[3*j*SW + 3*i + 1];
      uint8_t b = gimp_image.pixel_data[3*j*SW + 3*i + 2];
      screen[j*SW + i] = (ws2812_pixel_t) {
        .r = r,
        .g = g,
        .b = b,
      };
    }

    screen_to_pixel(screen, pixels);
    for (uint16_t it = 0; it < sizeof(pixels)/sizeof(ws2812_pixel_t); it ++)
    {
      ws2812_sendPixel(&pixels[it]);
    }


    _delay_ms(20);
//²:w    portpin_outtgl(&LED_RUN_PP);
  }

  while(1);
}



