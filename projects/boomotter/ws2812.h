#ifndef _WS2812_H_
#define _WS2812_H_

#include <avarix.h>

typedef uint32_t ws2812_pixel_t;

void ws2812_init(void);

void ws2812_sendPixel( ws2812_pixel_t pixel);

void ws2812_show(void);
#endif //_WS2812_H_
