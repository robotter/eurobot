#ifndef DRAW_H
#define DRAW_H

#include <stdint.h>
#include <string.h>
#include <avr/pgmspace.h>

#define SCREEN_W  12  // Screen width
#define SCREEN_H  10   // Screen height

/// A single screen pixel
typedef struct {
  uint8_t r;
  uint8_t g;
  uint8_t b;
} pixel_t;

/// Texture where to draw
typedef struct {
  uint8_t width;
  uint8_t height;
  pixel_t pixels[];
} __attribute__((__packed__)) texture_t;


/// Rectangle, used for cropping
typedef struct {
  uint8_t x0, y0;
  uint8_t x1, y1;
} draw_rect_t;

/// Blending a color in place
typedef void (*blend_color_t)(pixel_t *p, pixel_t color);
/// Blending a gray in place
typedef void (*blend_gray_t)(pixel_t *p, uint8_t gray);


#define RGB(r,g,b) ((pixel_t){(r),(g),(b)})
#define GRAY(v) ((pixel_t){(v),(v),(v)})


/// Declare a texture with given size on the stack
#define TEXTURE_DECL(name,w,h)  \
    texture_t *name = __builtin_alloca(sizeof(texture_t) + (w) * (h) * sizeof(pixel_t)); \
    name->width = (w); \
    name->height = (h);

/// Declare a texture for a whole screen
#define SCREEN_TEXTURE_DECL(name)  TEXTURE_DECL(name,SCREEN_W,SCREEN_H)

/// Pointer to a texture's pixel
#define TEXTURE_PIXEL(t,x,y)  (&(t)->pixels[(y) * (t)->width + (x)])


#define RECT(x0,y0,x1,y1)  ((draw_rect_t){(x0),(y0),(x1),(y1)})

const draw_rect_t screen_rect;


/// Bound an area by to fit in a texture
inline draw_rect_t bound_to_texture(const texture_t *tex, int8_t x, int8_t y, uint8_t w, uint8_t h) {
  if(x >= (int8_t)tex->width || y >= (int8_t)tex->height) {
    return RECT(0, 0, 0, 0);  // completely out of texture
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

/// Clear a texture with zeroes
inline void texture_clear(texture_t *tex) {
  memset(tex->pixels, 0, tex->width * tex->height * sizeof(*tex->pixels));
}


/// Draw pixels to a texture
void draw_pixels(texture_t *tex, int8_t x, int8_t y, uint8_t w, uint8_t h, const pixel_t *pixels);

/// Draw PROGMEM pixels to a texture
void draw_pixels_pgm(texture_t *tex, int8_t x, int8_t y, uint8_t w, uint8_t h, const pixel_t *pixels);

/// Blend gray map to a texture
void blend_pixels_gray(texture_t *tex, int8_t x, int8_t y, uint8_t w, uint8_t h, const uint8_t *values, blend_gray_t blender);

/// Blend PROGMEM gray map to a texture
void blend_pixels_gray_pgm(texture_t *tex, int8_t x, int8_t y, uint8_t w, uint8_t h, const uint8_t *values, blend_gray_t blender);

/// Draw a texture to a texture
inline void draw_texture(texture_t *dst, int8_t x, int8_t y, const texture_t *src) {
  draw_pixels(dst, x, y, src->width, src->height, src->pixels);
}

/// Draw a filled rectangle on a texture
void draw_rect(texture_t *tex, const draw_rect_t *rect, pixel_t color);

/** @brief Blend rectange a texture with given color, multiply
 *
 * @note \a rect must be within texture boundaries.
 */
void blend_rect(texture_t *tex, const draw_rect_t *rect, pixel_t color, blend_color_t blender);


/// Image data
typedef struct {
  uint8_t height;
  uint8_t width;
  /// Image pixels (in progmem)
  const pixel_t *data;
} image_t;


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


/// Compute the width of a character
uint8_t get_char_width(const font_t *font, char c);
/// Compute the width of a string
uint16_t get_text_width(const font_t *font, const char *text);

/** @brief Blend a single character, return its width
 *
 * Glyph is cropped to texture boundaries.
 */
uint8_t blend_char(texture_t *tex, const font_t *font, int8_t x, int8_t y, char c, blend_gray_t blender);

/** @brief Blend text, return its width
 *
 * Text is cropped to texture boundaries
 */
uint8_t blend_text(texture_t *tex, const font_t *font, int8_t x, int8_t y, const char *c, blend_gray_t blender);


/// Define an inline color blender
#define BLEND_COLOR(block_) ({ \
    void callback_(pixel_t *p, pixel_t c) block_ \
    callback_; \
})

/// Blend gray, set value
void blend_gray_set(pixel_t *p, uint8_t gray);
/// Blend gray, multiply
void blend_gray_mul(pixel_t *p, uint8_t gray);
/// Blend color, multiply
void blend_color_mul(pixel_t *p, pixel_t c);


/** @brief Helper to apply code on a rectangle
 *
 * Iterate on each pixel, define `x`, `y` and `p` (for the pixel), and execute
 * the block that follows the macro.
 */
#define FOREACH_RECT_PIXEL(tex,rect) \
  for(uint8_t y = (rect).y0; y < (rect).y1; y++) \
  for(uint8_t x = (rect).x0; x < (rect).x1; x++) \
  for(pixel_t *p = TEXTURE_PIXEL((tex), (x), (y)); p; p=0) \

#define FOREACH_PIXEL(tex) \
  for(uint8_t y = 0; y < (tex)->height; y++) \
  for(uint8_t x = 0; x < (tex)->width; x++) \
  for(pixel_t *p = TEXTURE_PIXEL((tex), (x), (y)); p; p=0) \

/// Scrolling text data
typedef struct {
  const font_t *font;
  const char *text;
  uint16_t text_width;
  int16_t pos;
  uint8_t pos_prescaler;
} scrolling_text_t;

/** @brief Initialize a scrolling text
 *
 * Text will be drew on the whole texture width.
 */
void scrolling_text_init(scrolling_text_t *scroll, const font_t *font, const char *text, uint8_t prescaler);
/// Draw scrolling text (repeated)
void scrolling_text_draw(const scrolling_text_t *scroll, texture_t *tex, int8_t y);
/// Scroll text (in any direction
void scrolling_text_scroll(scrolling_text_t *scroll, int8_t n);


#endif
