#include <string.h>
#include <util/atomic.h>
#include "draw.h"
#include "ws2812.h"

const draw_rect_t screen_upper_rect = RECT(0, 0, SCREEN_UW, SCREEN_UH);
const draw_rect_t screen_lower_rect = RECT(0, SCREEN_UH, SCREEN_LW, SCREEN_H);


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

uint16_t get_text_width(const font_t *font, const char *c) {
  if(!*c) {
    return 0;
  }
  uint16_t x = 0;
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


void scrolling_text_init(scrolling_text_t *scroll, const font_t *font, const char *text, uint8_t prescaler) {
  scroll->font = font;
  scroll->text = text;
  scroll->text_width = get_text_width(font, text) + 1;
  scroll->pos = 0;
  scroll->pos_prescaler = prescaler;
}

void scrolling_text_draw(const scrolling_text_t *scroll, texture_t *tex, int8_t y) {
  int8_t pos = scroll->pos / scroll->pos_prescaler;
  for(int16_t x = pos; x < tex->width; x += scroll->text_width) {
    blend_text(tex, scroll->font, x, y, scroll->text, blend_gray_set);
  }
  for(int16_t x = pos - scroll->text_width; x > (int16_t)-scroll->text_width; x -= scroll->text_width) {
    blend_text(tex, scroll->font, x, y, scroll->text, blend_gray_set);
  }
}

void scrolling_text_scroll(scrolling_text_t *scroll, int8_t n) {
  scroll->pos += n;
  uint16_t scaled_width = scroll->text_width * scroll->pos_prescaler;
  while(scroll->pos < 0) {
    scroll->pos += scaled_width;
  }
  // pos >= 0 due to above loop, cast to unsigned is safe
  while((uint16_t)scroll->pos >= scaled_width) {
    scroll->pos -= scaled_width;
  }
}

