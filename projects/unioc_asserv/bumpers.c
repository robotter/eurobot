#if defined(GALIPETTE)

#include <string.h>
#include <avarix/portpin.h>
#include "bumpers.h"
#include "telemetry.h"

#define BUMPER_PIN_L  PORTPIN(B,0)
#define BUMPER_PIN_R  PORTPIN(B,2)

void bumpers_init(void)
{
  portpin_dirclr(&PORTPIN(B,0));
  //portpin_dirclr(&PORTPIN(B,1));
  portpin_dirclr(&PORTPIN(B,2));
  //portpin_dirclr(&PORTPIN(B,3));
  PORTB.PIN0CTRL = PORT_OPC_PULLUP_gc;
  //PORTB.PIN1CTRL = PORT_OPC_PULLUP_gc;
  PORTB.PIN2CTRL = PORT_OPC_PULLUP_gc;
  //PORTB.PIN3CTRL = PORT_OPC_PULLUP_gc;
}

void bumpers_update(void)
{
  TM_DL_BUMPERS(portpin_in(&BUMPER_PIN_L), portpin_in(&BUMPER_PIN_R));
}

bool bumpers_pushed(void)
{
  return portpin_in(&BUMPER_PIN_L) && portpin_in(&BUMPER_PIN_R);
}

bool bumper_left_pushed(void) {return portpin_in(&BUMPER_PIN_L);}
bool bumper_right_pushed(void){return portpin_in(&BUMPER_PIN_R);}
#endif
