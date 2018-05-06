#include <string.h>
#include "draw.h"
#include "ws2812.h"


void display_screen(const texture_t *tex) {
  ws2812_sendPixel(0);
  const pixel_t *p = tex->pixels;

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


uint8_t draw_char(texture_t *tex, const font_t *font, int x, int y, char c, pixel_t color) {
  if(c < FONT_FIRST_CHAR || c > FONT_LAST_CHAR) {
    goto unknown;
  }
  uint8_t width = pgm_read_byte(&font->glyphs[c - ' '].width);
  if(width == 0) {
    goto unknown;
  }

  if(tex) {
    uint16_t offset = pgm_read_word(&font->glyphs[c - ' '].offset);
    for(int dy = 0; dy < font->height; dy++) {
      for(int dx = 0; dx < width; dx++) {
        uint8_t v = pgm_read_byte(&font->data[offset + dy * width + dx]);
        *TEXTURE_PIXEL(tex, x+dx, y+dy) = v ? color : 0;
      }
    }
  }
  return width;

unknown:
  // draw red rectangle
  width = 4;
  if(tex) {
    for(int dy = 0; dy < font->height; dy++) {
      for(int dx = 0; dx < width; dx++) {
        *TEXTURE_PIXEL(tex, x+dx, y+dy) = 0x7f0000;
      }
    }
  }
  return width;
}

uint8_t draw_text(texture_t *screen, const font_t *font, int x, int y, const char *c, pixel_t color) {
  const uint8_t x0 = x;
  for(; *c; c++) {
    x += 1 + draw_char(screen, font, x, y, *c, color);
  }
  return x - x0;
}

