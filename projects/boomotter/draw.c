#include <string.h>
#include <util/atomic.h>
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

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
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
  }

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

void blend_pixels_gray(texture_t *tex, int8_t x, int8_t y, uint8_t w, uint8_t h, const uint8_t *values, blend_gray_t blender) {
  const draw_rect_t bb = bound_to_texture(tex, x, y, w, h);
  for(uint8_t dy = bb.y0; dy < bb.y1; dy++) {
    for(uint8_t dx = bb.x0; dx < bb.x1; dx++) {
      blender(TEXTURE_PIXEL(tex, x+dx, y+dy), values[dy * w + dx]);
    }
  }
}

void blend_pixels_gray_pgm(texture_t *tex, int8_t x, int8_t y, uint8_t w, uint8_t h, const uint8_t *values, blend_gray_t blender) {
  const draw_rect_t bb = bound_to_texture(tex, x, y, w, h);
  for(uint8_t dy = bb.y0; dy < bb.y1; dy++) {
    for(uint8_t dx = bb.x0; dx < bb.x1; dx++) {
      blender(TEXTURE_PIXEL(tex, x+dx, y+dy), pgm_read_byte(&values[dy * w + dx]));
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

void blend_rect(texture_t *tex, const draw_rect_t *rect, pixel_t color, blend_color_t blender) {
  for(uint8_t y = rect->y0; y < rect->y1; y++) {
    for(uint8_t x = rect->x0; x < rect->x1; x++) {
      blender(TEXTURE_PIXEL(tex, x, y), color);
    }
  }
}


#define BAD_CHAR_WIDTH  4

uint8_t get_char_width(const font_t *font, char c) {
  if(c < FONT_FIRST_CHAR || c > FONT_LAST_CHAR) {
    return BAD_CHAR_WIDTH;
  }
  uint8_t width = pgm_read_byte(&font->glyphs[c - ' '].width);
  return width == 0 ? BAD_CHAR_WIDTH : width;
}

uint8_t get_text_width(const font_t *font, const char *c) {
  if(!*c) {
    return 0;
  }
  uint8_t x = 0;
  for(; *c; c++) {
    x += 1 + get_char_width(font, *c);
  }
  return x - 1;
}


uint8_t blend_char(texture_t *tex, const font_t *font, int8_t x, int8_t y, char c, blend_gray_t blender) {
  if(c < FONT_FIRST_CHAR || c > FONT_LAST_CHAR) {
    goto unknown;
  }
  uint8_t width = pgm_read_byte(&font->glyphs[c - ' '].width);
  if(width == 0) {
    goto unknown;
  }

  if(tex) {
    const uint16_t offset = pgm_read_word(&font->glyphs[c - ' '].offset);
    blend_pixels_gray_pgm(tex, x, y, width, font->height, font->data + offset, blender);
  }
  return width;

unknown:
  // draw red rectangle
  width = BAD_CHAR_WIDTH;
  if(tex) {
    const draw_rect_t bb = bound_to_texture(tex, x, y, width, font->height);
    draw_rect(tex, &bb, RGB(0x7f, 0, 0));
  }
  return width;
}

uint8_t blend_text(texture_t *tex, const font_t *font, int8_t x, int8_t y, const char *c, blend_gray_t blender) {
  if(!*c) {
    return 0;
  }
  const uint8_t x0 = x;
  for(; *c; c++) {
    x += 1 + blend_char(tex, font, x, y, *c, blender);
  }
  return x - x0 - 1;
}


void blend_gray_set(pixel_t *p, uint8_t gray) {
  *p = RGB(gray, gray, gray);
}

void blend_gray_mul(pixel_t *p, uint8_t gray) {
  p->r = ((uint16_t)p->r * gray) >> 8;
  p->g = ((uint16_t)p->g * gray) >> 8;
  p->b = ((uint16_t)p->b * gray) >> 8;
}

void blend_color_mul(pixel_t *p, pixel_t c) {
  p->r = ((uint16_t)p->r * c.r) >> 8;
  p->g = ((uint16_t)p->g * c.g) >> 8;
  p->b = ((uint16_t)p->b * c.b) >> 8;
}

