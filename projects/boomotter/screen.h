#ifndef SCREEN_H
#define SCREEN_H

#include <stdint.h>

#define UPPER_WIDTH   21
#define UPPER_HEIGHT  6
#define LOWER_WIDTH   11
#define LOWER_HEIGHT  10
#define SH (UPPER_HEIGHT+LOWER_HEIGHT)
#define SW (UPPER_WIDTH)
#define I(x,y) ((x) + (y)*SW)

/// A single screen pixel
typedef uint32_t pixel_t;
/// A whole screen, indexed by x/y directly
typedef pixel_t screen_t[UPPER_WIDTH * (UPPER_HEIGHT + LOWER_HEIGHT)];

/// Draw a screen of pixels
void screen_draw(const screen_t screen);

#endif
