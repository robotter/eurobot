#include <string.h>
#include "screen.h"
#include "ws2812.h"

void screen_draw(const screen_t screen)
{
  ws2812_sendPixel(0);
  const pixel_t *p = screen;

#define send_pixels_l2r(n) \
  for(uint8_t i=0; i<(n); i++) ws2812_sendPixel(*p++);
#define send_pixels_r2l(n) \
  for(uint8_t i=0; i<(n); i++) ws2812_sendPixel(p[(n)-i-1]); \
  p += (n);

  send_pixels_l2r(SCREEN_UW);
  send_pixels_r2l(SCREEN_UW);
  send_pixels_l2r(SCREEN_UW);
  send_pixels_r2l(SCREEN_UW);
  send_pixels_l2r(SCREEN_UW);
  send_pixels_r2l(SCREEN_UW);

  send_pixels_l2r(SCREEN_LW); p += SCREEN_W - SCREEN_LW;
  send_pixels_r2l(SCREEN_LW); p += SCREEN_W - SCREEN_LW;
  send_pixels_l2r(SCREEN_LW); p += SCREEN_W - SCREEN_LW;
  send_pixels_r2l(SCREEN_LW); p += SCREEN_W - SCREEN_LW;
  send_pixels_l2r(SCREEN_LW); p += SCREEN_W - SCREEN_LW;
  send_pixels_r2l(SCREEN_LW); p += SCREEN_W - SCREEN_LW;
  send_pixels_l2r(SCREEN_LW); p += SCREEN_W - SCREEN_LW;
  send_pixels_r2l(SCREEN_LW); p += SCREEN_W - SCREEN_LW;
  send_pixels_l2r(SCREEN_LW); p += SCREEN_W - SCREEN_LW;
  send_pixels_r2l(SCREEN_LW);

#undef send_pixels_l2r
#undef send_pixels_r2l
}

