#ifndef WS2812_H_
#define WS2812_H_

#include <stdint.h>

void ws2812_init(void);
void ws2812_send_pixel(uint8_t r, uint8_t g, uint8_t b);
void ws2812_show(void);

#endif
