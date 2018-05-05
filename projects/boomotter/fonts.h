#ifndef FONTS_H
#define FONTS_H

#include <stdint.h>
#include <avr/pgmspace.h>
#include "screen.h"

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


/// Draw a single character, return its width
uint8_t draw_char(pixel_t *screen, const font_t *font, int x, int y, char c, pixel_t color);
/// Draw text, return its width
uint8_t draw_text(pixel_t *screen, const font_t *font, int x, int y, const char *c, pixel_t color);

#endif
