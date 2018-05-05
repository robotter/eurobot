#include <avarix.h>
#include <avarix/intlvl.h>
#include <avarix/portpin.h>
#include <uart/uart.h>
#include <clock/clock.h>
#include <rome/rome.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include "leds.h"
#include "dfplayer_mini.h"
#include "audio_amplifier.h"
#include "ws2812.h"
#include "screen.h"

//#include "pic.c"

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


#include "font.h"

void draw_glyph(pixel_t *screen, int x, int y, uint8_t c, pixel_t color) {
 
  int n = font_pixels_lut[c];
  if(n < 0) {
    for(int dy=0;dy<font_height;dy++)
    for(int dx=0;dx<font_width;dx++) {
      screen[I(dx,dy)] = 0x7f0000;
    }
    return;
  }

  for(int dy=0;dy<font_height;dy++)
  for(int dx=0;dx<font_width;dx++) {
    uint8_t v =pgm_read_byte(&font_pixels[n][dx+font_width*dy]);
    screen[I(dx,dy)] = v;
  }
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
  dfplayer_set_volume(0);

  amplifier_shutdown(0);
  amplifier_mute(0);
//  amplifier_set_gain(GAIN_26DB);
  
  screen_t screen;
  memset(screen, 0, sizeof(screen));
  
  portpin_outset(&LED_RUN_PP); 

  int i =0;
  while(1) {
    rome_handle_input(&rome_intf);
    
    if(!dfplayer_is_busy()) {
      dfplayer_play_track(1);
    }

    memset(screen, 0, sizeof(screen));
    i = (i+1)%10;
    draw_glyph(screen,0,0,'0'+i,0x333333);

    screen_draw(screen);

    _delay_ms(500);
  }

  while(1);
}



