#include <avarix.h>
#include <avarix/intlvl.h>
#include <avarix/portpin.h>
#include <uart/uart.h>
#include <clock/clock.h>
#include <rome/rome.h>
#include <util/delay.h>
#include "leds.h"
#include "dfplayer_mini.h"
#include "audio_amplifier.h"
#include "ws2812.h"
#include "draw.h"
#include "font_bitmap.inc.c"


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

  SCREEN_TEXTURE_DECL(screen);
  texture_clear(screen);
  display_screen(screen);

  portpin_outset(&LED_RUN_PP); 

  const char *scrolling_text = "DEBUG TEAM  ";
  const uint8_t scrolling_text_width = draw_text(0, &font_base, 0, 0, scrolling_text, 0);
  int8_t pos = 0;
  int8_t color_shift = 0;
  uint8_t color_dir = 1;
  while(1) {
    rome_handle_input(&rome_intf);

    if(!dfplayer_is_busy()) {
      dfplayer_play_track(1);
    }

    texture_clear(screen);
    const draw_rect_t upper_rect = { 0, 0, SCREEN_UW, SCREEN_UH };

    draw_text(screen, &font_base, pos, 0, scrolling_text, 0xffff00);
    if(pos < 0) {
      draw_text(screen, &font_base, pos + scrolling_text_width, 0, scrolling_text, 0xffff00);
    }
    pixel_t blending_color = (1U << color_shift) | ((uint16_t)1U << (color_shift + 8)) | ((uint32_t)1U << (color_shift + 16));
    blend_texture_mul(screen, &upper_rect, blending_color);
    display_screen(screen);

    if(--pos <= -scrolling_text_width) {
      pos = 0;
    }

    color_shift += color_dir;
    if(color_shift >= 8) {
      color_shift = 7;
      color_dir = -1;
    } else if(color_shift < 2) {
      color_shift = 2;
      color_dir = 1;
    }

    _delay_ms(100);
  }

  while(1);
}

