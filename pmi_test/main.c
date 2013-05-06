#include <avarix.h>
#include <clock/clock.h>

#include <math.h>

int main(void) {

  clock_init();

  PORTQ.DIRSET = (1<<1)|(1<<2)|(1<<3);
  PORTQ.OUTCLR = (1<<1)|(1<<2)|(1<<3);

  // testing PWMs
  PORTH.DIRSET = 0xFF;
  PORTH.OUTSET = 0xFF;

  PORTE.DIRSET = _BV(2)|_BV(3);
  PORTE.OUTSET = _BV(2)|_BV(3);

  uint8_t lvla = 50;
  uint8_t lvlb = 50;
  double t = 0.0;
  while(1) {

    t+=0.01;
    lvla = 255*(0.5+0.5*cos(t));
    lvlb = 255*(0.5+0.5*sin(1.1*t));

    uint8_t x,y;
    x = _BV(3);
    y = _BV(1)|_BV(2);

    uint8_t i;
    for(i=0;i<255;i++) {
      PORTQ.OUT = (i < lvla) ? x : 0;
      PORTQ.OUT = (i < lvlb) ? y : 0;
    }
  }

  return 0;
}
