#ifndef LEDS_H
#define LEDS_H

#include <avarix/portpin.h>
#include "config.h"

#define LED_RGB_SET(rgb) do { \
  (rgb & 0b100) ? portpin_outset(&LED_R_PP) : portpin_outclr(&LED_R_PP); \
  (rgb & 0b010) ? portpin_outset(&LED_G_PP) : portpin_outclr(&LED_G_PP); \
  (rgb & 0b001) ? portpin_outset(&LED_B_PP) : portpin_outclr(&LED_B_PP); \
} while(0)

typedef enum {
  LED_COLOR_RED      = 0b100,
  LED_COLOR_GREEN    = 0b010,
  LED_COLOR_BLUE     = 0b001,
  LED_COLOR_YELLOW   = 0b110,
  LED_COLOR_MAGENTA  = 0b101,
  LED_COLOR_CYAN     = 0b011,
  LED_COLOR_WHITE    = 0b111,
  LED_COLOR_BLACK    = 0b000,
} ledrgb_color_t;


#endif
