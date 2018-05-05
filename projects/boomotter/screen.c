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

  send_pixels_l2r(UPPER_WIDTH);
  send_pixels_r2l(UPPER_WIDTH);
  send_pixels_l2r(UPPER_WIDTH);
  send_pixels_r2l(UPPER_WIDTH);
  send_pixels_l2r(UPPER_WIDTH);
  send_pixels_r2l(UPPER_WIDTH);

  send_pixels_l2r(LOWER_WIDTH); p += UPPER_WIDTH - LOWER_WIDTH;
  send_pixels_r2l(LOWER_WIDTH); p += UPPER_WIDTH - LOWER_WIDTH;
  send_pixels_l2r(LOWER_WIDTH); p += UPPER_WIDTH - LOWER_WIDTH;
  send_pixels_r2l(LOWER_WIDTH); p += UPPER_WIDTH - LOWER_WIDTH;
  send_pixels_l2r(LOWER_WIDTH); p += UPPER_WIDTH - LOWER_WIDTH;
  send_pixels_r2l(LOWER_WIDTH); p += UPPER_WIDTH - LOWER_WIDTH;
  send_pixels_l2r(LOWER_WIDTH); p += UPPER_WIDTH - LOWER_WIDTH;
  send_pixels_r2l(LOWER_WIDTH); p += UPPER_WIDTH - LOWER_WIDTH;
  send_pixels_l2r(LOWER_WIDTH); p += UPPER_WIDTH - LOWER_WIDTH;
  send_pixels_r2l(LOWER_WIDTH);

#undef send_pixels_l2r
#undef send_pixels_r2l
}

