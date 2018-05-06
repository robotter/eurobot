#ifndef _WS2812_H_
#define _WS2812_H_

#include <avarix.h>

void ws2812_init(void);

void ws2812_send_pixel(uint8_t r, uint8_t g, uint8_t b);

void ws2812_show(void);
#endif //_WS2812_H_
