#include "fonts.h"


uint8_t draw_char(pixel_t *screen, const font_t *font, int x, int y, char c, pixel_t color) {
  if(c < FONT_FIRST_CHAR || c > FONT_LAST_CHAR) {
    goto unknown;
  }
  uint8_t width = pgm_read_byte(&font->glyphs[c - ' '].width);
  if(width == 0) {
    goto unknown;
  }

  uint16_t offset = pgm_read_word(&font->glyphs[c - ' '].offset);
  for(int dy = 0; dy < font->height; dy++) {
    for(int dx = 0; dx < width; dx++) {
      uint8_t v = pgm_read_byte(&font->data[offset + dy * width + dx]);
      screen[SCREEN_PIXEL(x+dx,y+dy)] = v ? color : 0;
    }
  }
  return width;

unknown:
  // draw red rectangle
  width = 4;
  for(int y = 0; y < font->height; y++) {
    for(int x = 0; x < width; x++) {
      screen[SCREEN_PIXEL(x,y)] = 0x7f0000;
    }
  }
  return width;
}

uint8_t draw_text(pixel_t *screen, const font_t *font, int x, int y, const char *c, pixel_t color) {
  const uint8_t x0 = x;
  for(; *c; c++) {
    x += 1 + draw_char(screen, font, x, y, *c, color);
  }
  return x - x0;
}

