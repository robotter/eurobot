#include <avarix.h>
#include <avarix/intlvl.h>
#include <avarix/portpin.h>
#include <uart/uart.h>
#include <clock/clock.h>
#include <rome/rome.h>
#include "leds.h"
#include <util/delay.h>
//#include "r3d2.h"

rome_intf_t rome_intf;

#if 0
static void rome_handler(rome_intf_t *intf, const rome_frame_t *frame)
{
  switch(frame->mid) {
    case ROME_MID_R3D2_SET_ROTATION:
      r3d2_set_rotation(frame->r3d2_set_rotation.speed_rpm,
        frame->r3d2_set_rotation.threshold_percent);
      rome_reply_ack(intf, frame);
      break;

    case ROME_MID_R3D2_SET_BLIND_SPOT:
      r3d2_set_blind_spot(
        1.0*frame->r3d2_set_blind_spot.begin/1000,
        1.0*frame->r3d2_set_blind_spot.end/1000);
      rome_reply_ack(intf, frame);
      break;
        
    default:
      break;
  }
}
#endif

int main(void)
{
  clock_init();

  //uart_init();

  INTLVL_ENABLE_ALL();
  __asm__("sei");

  portpin_dirset(&LED_RUN_PP);
  portpin_dirset(&LED_ERROR_PP);
  portpin_dirset(&LED_COM_PP);

  //r3d2_init();

  //rome_intf_init(&rome_intf);
  //rome_intf.uart = uartD0;
  //rome_intf.handler = rome_handler;

  //ROME_LOGF(&rome_intf, INFO, "RST.STATUS=%x booting...\n", RST.STATUS);
  //RST.STATUS = 0;

  uint8_t a=0;
  while(1) {
    switch (a){ 
      case 0:
        portpin_outclr(&LED_RUN_PP);
        portpin_outclr(&LED_ERROR_PP);
        portpin_outset(&LED_COM_PP);
        break;
      case 1:
        portpin_outclr(&LED_RUN_PP);
        portpin_outset(&LED_ERROR_PP);
        portpin_outclr(&LED_COM_PP);
        break;
      case 2:
        portpin_outset(&LED_RUN_PP);
        portpin_outclr(&LED_ERROR_PP);
        portpin_outclr(&LED_COM_PP);
        break;
    }
    a ++;
    a %= 3;
    _delay_ms(200);
    //rome_handle_input(&rome_intf);
    //r3d2_update();

  }

  while(1);
}

