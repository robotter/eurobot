#include <string.h>
#include "draw.h"
#include "ws2812.h"


void display_screen(const texture_t *tex) {
  ws2812_send_pixel(0, 0, 0);
  const pixel_t *p = tex->pixels;

#define send_pixels_l2r(n) \
  for(uint8_t i=0; i<(n); i++) { \
    ws2812_send_pixel(p->r, p->g, p->b); \
    p++; \
  }
#define send_pixels_r2l(n) \
  for(uint8_t i=0; i<(n); i++) { \
    const pixel_t *p2 = p + (n) - i - 1; \
    ws2812_send_pixel(p2->r, p2->g, p2->b); \
  } \
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


void draw_pixels(texture_t *tex, int8_t x, int8_t y, uint8_t w, uint8_t h, const pixel_t *pixels) {
  const draw_rect_t bb = bound_to_texture(tex, x, y, w, h);
  for(uint8_t dy = bb.y0; dy < bb.y1; dy++) {
    for(uint8_t dx = bb.x0; dx < bb.x1; dx++) {
      *TEXTURE_PIXEL(tex, x+dx, y+dy) = pixels[dy * w + dx];
    }
  }
}

void draw_pixels_pgm(texture_t *tex, int8_t x, int8_t y, uint8_t w, uint8_t h, const pixel_t *pixels) {
  const draw_rect_t bb = bound_to_texture(tex, x, y, w, h);
  for(uint8_t dy = bb.y0; dy < bb.y1; dy++) {
    for(uint8_t dx = bb.x0; dx < bb.x1; dx++) {
      const pixel_t *pixel = &pixels[dy * w + dx];
      const uint8_t r = pgm_read_byte(&pixel->r);
      const uint8_t g = pgm_read_byte(&pixel->g);
      const uint8_t b = pgm_read_byte(&pixel->b);
      *TEXTURE_PIXEL(tex, x+dx, y+dy) = RGB(r, g, b);
    }
  }
}

void draw_pixels_color(texture_t *tex, int8_t x, int8_t y, uint8_t w, uint8_t h, const uint8_t *values, pixel_t color) {
  const draw_rect_t bb = bound_to_texture(tex, x, y, w, h);
  for(uint8_t dy = bb.y0; dy < bb.y1; dy++) {
    for(uint8_t dx = bb.x0; dx < bb.x1; dx++) {
      *TEXTURE_PIXEL(tex, x+dx, y+dy) = blend_gray_mul(color, values[dy * w + dx]);
    }
  }
}

void draw_pixels_color_pgm(texture_t *tex, int8_t x, int8_t y, uint8_t w, uint8_t h, const uint8_t *values, pixel_t color) {
  const draw_rect_t bb = bound_to_texture(tex, x, y, w, h);
  for(uint8_t dy = bb.y0; dy < bb.y1; dy++) {
    for(uint8_t dx = bb.x0; dx < bb.x1; dx++) {
      *TEXTURE_PIXEL(tex, x+dx, y+dy) = blend_gray_mul(color, pgm_read_byte(&values[dy * w + dx]));
    }
  }
}

void draw_rect(texture_t *tex, const draw_rect_t *rect, pixel_t color) {
  for(uint8_t y = rect->y0; y < rect->y1; y++) {
    for(uint8_t x = rect->x0; x < rect->x1; x++) {
      *TEXTURE_PIXEL(tex, x, y) = color;
    }
  }
}

void blend_texture_mul(texture_t *tex, const draw_rect_t *rect, pixel_t color) {
  for(uint8_t y = rect->y0; y < rect->y1; y++) {
    for(uint8_t x = rect->x0; x < rect->x1; x++) {
      *TEXTURE_PIXEL(tex, x, y) = blend_mul(*TEXTURE_PIXEL(tex, x, y), color);
    }
  }
}


uint8_t draw_char(texture_t *tex, const font_t *font, int8_t x, int8_t y, char c, pixel_t color) {
  if(c < FONT_FIRST_CHAR || c > FONT_LAST_CHAR) {
    goto unknown;
  }
  uint8_t width = pgm_read_byte(&font->glyphs[c - ' '].width);
  if(width == 0) {
    goto unknown;
  }

  if(tex) {
    const uint16_t offset = pgm_read_word(&font->glyphs[c - ' '].offset);
    draw_pixels_color_pgm(tex, x, y, width, font->height, font->data + offset, color);
  }
  return width;

unknown:
  // draw red rectangle
  width = 4;
  if(tex) {
    const draw_rect_t bb = bound_to_texture(tex, x, y, width, font->height);
    for(uint8_t dy = bb.y0; dy < bb.y1; dy++) {
      for(uint8_t dx = bb.x0; dx < bb.x1; dx++) {
        *TEXTURE_PIXEL(tex, x+dx, y+dy) = RGB(0x7f, 0, 0);
      }
    }
  }
  return width;
}

uint8_t draw_text(texture_t *screen, const font_t *font, int8_t x, int8_t y, const char *c, pixel_t color) {
  if(!*c) {
    return 0;
  }
  const uint8_t x0 = x;
  for(; *c; c++) {
    x += 1 + draw_char(screen, font, x, y, *c, color);
  }
  return x - x0 - 1;
}

