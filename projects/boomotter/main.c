#include <avarix.h>
#include <avarix/intlvl.h>
#include <avarix/portpin.h>
#include <uart/uart.h>
#include <clock/clock.h>
#include <rome/rome.h>
#include "leds.h"
#include "dfplayer_mini.h"
#include "audio_amplifier.h"

rome_intf_t rome_intf;

static void rome_handler(rome_intf_t *intf, const rome_frame_t *frame)
{
  #if 0
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
#else

#endif
}

int main(void)
{
  portpin_dirset(&LED_RUN_PP);
  portpin_dirset(&LED_ERROR_PP);
  portpin_dirset(&LED_COM_PP);


  portpin_outset(&LED_COM_PP);
  portpin_outset(&LED_RUN_PP);
  portpin_outset(&LED_ERROR_PP);
  clock_init();
  portpin_outclr(&LED_COM_PP);

  uart_init();
  uart_fopen(uartC0);
  portpin_outclr(&LED_RUN_PP);

  INTLVL_ENABLE_ALL();
  __asm__("sei");

  rome_intf_init(&rome_intf);
  rome_intf.uart = uartC0;
  rome_intf.handler = rome_handler;

  ROME_LOGF(&rome_intf, INFO, "RST.STATUS=%x booting...\n", RST.STATUS);
  RST.STATUS = 0;

  portpin_outclr(&LED_ERROR_PP);

  dfplayer_init();
  amplifier_init();

  while(1) {
    rome_handle_input(&rome_intf);
    
    if (!dfplayer_is_busy())
      dfplayer_play();

    ROME_LOGF(&rome_intf, INFO, "RST.STATUS=%x booting...\n", RST.STATUS);

    portpin_outset(&LED_COM_PP);
    for(volatile uint32_t i = 0; i<= 320000; i++);
    amplifier_set_shutdown(0);
    amplifier_set_mute(0);

    portpin_outclr(&LED_COM_PP);
    for(volatile uint32_t i = 0; i<= 320000; i++);
  }

  while(1);
}

