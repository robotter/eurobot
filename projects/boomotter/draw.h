#ifndef DRAW_H
#define DRAW_H

#include <stdint.h>
#include <string.h>
#include <avr/pgmspace.h>

#define SCREEN_UW  21  // Screen upper width
#define SCREEN_UH  6   // Screen upper height
#define SCREEN_LW  11  // Screen lower width
#define SCREEN_LH  10  // Screen lower height
#define SCREEN_W  (SCREEN_UW)  // Screen total width
#define SCREEN_H  (SCREEN_UW+SCREEN_LH)  // Screen total height

/// A single screen pixel (-RGB)
typedef uint32_t pixel_t;

/// Blend color and gray, multiple
inline pixel_t blend_gray_mul(pixel_t color, uint8_t gray) {
  const uint8_t r = ((uint8_t)(color >> 16) * (uint16_t)gray) >> 8;
  const uint8_t g = ((uint8_t)(color >> 8) * (uint16_t)gray) >> 8;
  const uint8_t b = ((uint8_t)(color >> 0) * (uint16_t)gray) >> 8;
  return ((uint32_t)r << 16) | ((uint16_t)g << 8) | b;
}


/// Texture where to draw
typedef struct {
  uint8_t width;
  uint8_t height;
  pixel_t pixels[];
} __attribute__((__packed__)) texture_t;

/// Declare a texture with given size on the stack
#define TEXTURE_DECL(name,w,h)  \
    texture_t *name = __builtin_alloca(sizeof(texture_t) + (w) * (h) * sizeof(pixel_t)); \
    name->width = (w); \
    name->height = (h);

/// Declare a texture for a whole screen
#define SCREEN_TEXTURE_DECL(name)  TEXTURE_DECL(name,SCREEN_W,SCREEN_H)

/// Clear a texture with zeroes
inline void texture_clear(texture_t *tex) {
  memset(tex->pixels, 0, tex->width * tex->height * sizeof(*tex->pixels));
}

/// Pointer to a texture's pixel
#define TEXTURE_PIXEL(t,x,y)  (&(t)->pixels[(y) * (t)->width + (x)])

/// Rectangle, used for cropping
typedef struct {
  uint8_t x0, y0;
  uint8_t x1, y1;
} draw_rect_t;

/// Bound an area by to fit in a texture
inline draw_rect_t bound_to_texture(const texture_t *tex, int8_t x, int8_t y, uint8_t w, uint8_t h) {
  if(x >= (int8_t)tex->width || y > (int8_t)tex->height) {
    return (draw_rect_t){ 0, 0, 0, 0 };  // completely out of texture
  }
  return (draw_rect_t){
    .x0 = x < 0 ? -x : 0,
    .y0 = y < 0 ? -y : 0,
    .x1 = x + w <= tex->width ? w : tex->width - x,
    .y1 = y + h <= tex->height ? h : tex->height - y,
  };
}


/** @brief Display a screen of pixels on leds
 *
 * The texture size must be `(SCREEN_W, SCREEN_H)`.
 */
void display_screen(const texture_t *tex);


/// Draw pixels to a texture
void draw_pixels(texture_t *tex, int8_t x, int8_t y, uint8_t w, uint8_t h, const pixel_t *pixels);

/// Draw PROGMEM pixels to a texture
void draw_pixels_pgm(texture_t *tex, int8_t x, int8_t y, uint8_t w, uint8_t h, const pixel_t *pixels);

/// Blend gray map to a texture
void draw_pixels_color(texture_t *tex, int8_t x, int8_t y, uint8_t w, uint8_t h, const uint8_t *values, pixel_t color);

/// Blend PROGMEM gray map to a texture
void draw_pixels_color_pgm(texture_t *tex, int8_t x, int8_t y, uint8_t w, uint8_t h, const uint8_t *values, pixel_t color);

/// Draw a texture to a texture
inline void draw_texture(texture_t *dst, int8_t x, int8_t y, const texture_t *src) {
  draw_pixels(dst, x, y, src->width, src->height, src->pixels);
}


#define FONT_FIRST_CHAR ' '
#define FONT_LAST_CHAR 0x7e

/// Glyph data
typedef struct {
  uint8_t width;  ///< Character width
  uint16_t offset;  ///< Data offset (in font data)
} font_glyph_t;

/// Font data
typedef struct {
  /// Font height same for all characters)
  uint8_t height;
  /// Array of characters (in progmem)
  const font_glyph_t *glyphs;
  /// Raw font data (in progmem)
  const uint8_t *data;
} font_t;


/** @brief Draw a single character, return its width
 *
 * Glyph is cropped to texture boundaries.
 * If screen is NULL, only return the width to draw.
 */
uint8_t draw_char(texture_t *screen, const font_t *font, int8_t x, int8_t y, char c, pixel_t color);

/** @brief Draw text, return its width
 *
 * Text is cropped to texture boundaries
 * If screen is NULL, only return the width to draw.
 */
uint8_t draw_text(texture_t *screen, const font_t *font, int8_t x, int8_t y, const char *c, pixel_t color);

#endif
