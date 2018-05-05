#ifndef SCREEN_H_
#define SCREEN_H_

#include <stdint.h>

#define SCREEN_UW  21  // Screen upper width
#define SCREEN_UH  6   // Screen upper height
#define SCREEN_LW  11  // Screen lower width
#define SCREEN_LH  10  // Screen lower height
#define SCREEN_W  (SCREEN_UW)  // Screen total width
#define SCREEN_H  (SCREEN_UW+SCREEN_LH)  // Screen total height
// Coordinates to screen pixel index
#define SCREEN_PIXEL(x,y)  ((y) * SCREEN_W + (x))

/// A single screen pixel
typedef uint32_t pixel_t;
/// A whole screen, indexed by x/y directly
typedef pixel_t screen_t[SCREEN_W * SCREEN_H];

/// Draw a screen of pixels
void screen_draw(const screen_t screen);

#endif
