#include <avarix.h>
#include <avarix/portpin.h>
#include <stdbool.h>
#include "clock_config.h"
#include "ws2812.h"


#define WS2812_DATA_PP  PORTPIN(B,2)


#define T1H  900  // Width of a 1 bit in ns
#define T1L  600  // Width of a 1 bit in ns

#define T0H  400  // Width of a 0 bit in ns
#define T0L  900  // Width of a 0 bit in ns

#define RES 7000  // Width of the low gap between bits to cause a frame to latch

// Note that this has to be SIGNED since we want to be able to check for
// negative values of derivatives
#define NS_PER_SEC (1000000000L)

#define CYCLES_PER_SEC (CLOCK_SOURCE_FREQ)

#define NS_PER_CYCLE (NS_PER_SEC / CYCLES_PER_SEC)

#define NS_TO_CYCLES(ns) ((ns) / NS_PER_CYCLE)

// Make sure we never have a delay less than zero
#define DELAY_CYCLES(n) ((n>0) ? __builtin_avr_delay_cycles(n) : __builtin_avr_delay_cycles(0))

void send_bit(bool) __attribute__ ((optimize(0)));

void send_bit(bool bit) {
  if(bit) {  // bit 1
    PORTB.OUTSET = _BV(2);
    DELAY_CYCLES(NS_TO_CYCLES(T1H) - 2);
    PORTB.OUTCLR = _BV(2);
    DELAY_CYCLES(NS_TO_CYCLES(T1L) - 10); // 1-bit gap less the overhead of the loop
  } else {  // bit 0
    PORTB.OUTSET = _BV(2);
    DELAY_CYCLES(NS_TO_CYCLES(T0H) - 5); // 0-bit width less overhead
    PORTB.OUTCLR = _BV(2);
    DELAY_CYCLES(NS_TO_CYCLES(T0L) - 50); // 0-bit gap less overhead of the loop
  }
}

void send_byte(unsigned char byte) {
  for(unsigned char bit = 0; bit < 8; bit++) {
    send_bit(byte >> 7);
    byte <<= 1;
  }
}

void ws2812_init(void)
{
  portpin_dirset(&WS2812_DATA_PP);
}

void ws2812_send_pixel(uint8_t r, uint8_t g, uint8_t b)
{
  send_byte(g);
  send_byte(r);
  send_byte(b);
}

void ws2812_show(void)
{
  DELAY_CYCLES(NS_TO_CYCLES(RES));
}

