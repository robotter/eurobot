#include <avarix.h>
#include <avarix/portpin.h>
#include <stdbool.h>
#include "clock_config.h"
#include "ws2812.h"


#define WS2812_DATA_PP  PORTPIN(A,0)


#define T1H  900    // Width of a 1 bit in ns
#define T1L  600    // Width of a 1 bit in ns

#define T0H  400    // Width of a 0 bit in ns
#define T0L  900    // Width of a 0 bit in ns

#define RES 7000    // Width of the low gap between bits to cause a frame to latch

#define NS_PER_SEC (1000000000L) // Note that this has to be SIGNED since we want to be able to check for negative values of derivatives

#define CYCLES_PER_SEC (CLOCK_SOURCE_FREQ)

#define NS_PER_CYCLE ( NS_PER_SEC / CYCLES_PER_SEC )

#define NS_TO_CYCLES(ns) ( (ns) / NS_PER_CYCLE )

#define DELAY_CYCLES(n) ( (n>0) ? __builtin_avr_delay_cycles( n ) : __builtin_avr_delay_cycles( 0 ) ) // Make sure we never have a delay less than zero

void sendBit(bool) __attribute__ ((optimize(0)));

void sendBit( bool bitVal ) {

  if ( bitVal ) {      // 1-bit

    portpin_outset(&WS2812_DATA_PP);

    DELAY_CYCLES( NS_TO_CYCLES( T1H ) - 2 );
    portpin_outclr(&WS2812_DATA_PP);

    DELAY_CYCLES( NS_TO_CYCLES( T1L ) - 10 ); // 1-bit gap less the overhead of the loop

  } else {             // 0-bit

    __asm__("cli");

    portpin_outset(&WS2812_DATA_PP);

    DELAY_CYCLES( NS_TO_CYCLES( T0H ) - 2 ); // 0-bit width less overhead
    portpin_outclr(&WS2812_DATA_PP);

    __asm__("sei");

    DELAY_CYCLES( NS_TO_CYCLES( T0L ) - 10 ); // 0-bit gap less overhead of the loop

  }


}

void sendByte( unsigned char byte ) {

  for( unsigned char bit = 0 ; bit < 8 ; bit++ ) {

    sendBit( byte>>7 ); 
    byte = byte << 1; 

  }
}

void ws2812_init(void)
{
  portpin_dirset(&WS2812_DATA_PP);
}

void ws2812_sendPixel( ws2812_pixel_t *pixel)
{
  sendByte(pixel->g); // Neopixel wants colors in green-then-red-then-blue order
  sendByte(pixel->r);
  sendByte(pixel->b);
}

void ws2812_show(void) 
{
  DELAY_CYCLES( NS_TO_CYCLES(RES) );
}
