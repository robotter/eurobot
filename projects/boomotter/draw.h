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


/** @brief Display a screen of pixels on leds
 *
 * The texture size must be `(SCREEN_W, SCREEN_H)`.
 */
void display_screen(const texture_t *tex);


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
 * If screen is NULL, only return the width to draw.
 */
uint8_t draw_char(texture_t *screen, const font_t *font, int x, int y, char c, pixel_t color);

/** @brief Draw text, return its width
 *
 * If screen is NULL, only return the width to draw.
 */
uint8_t draw_text(texture_t *screen, const font_t *font, int x, int y, const char *c, pixel_t color);

#endif
