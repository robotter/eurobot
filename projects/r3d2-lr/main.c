#include <avarix.h>
#include <avarix/intlvl.h>
#include <avarix/portpin.h>
#include <uart/uart.h>
#include <clock/clock.h>
#include <rome/rome.h>
#include <util/delay.h>
#include "leds.h"
#include "r3d2.h"
#include "config.h"

static rome_reader_t rome_reader;

static void rome_handler(const rome_frame_t *frame)
{
  switch(frame->mid) {
    case ROME_MID_R3D2_SET_ROTATION:
      r3d2_set_rotation(frame->r3d2_set_rotation.speed_rpm,
        frame->r3d2_set_rotation.threshold_percent);
      rome_reply_ack(UART_STRAT, frame);
      break;

    case ROME_MID_R3D2_SET_BLIND_SPOT:
      r3d2_set_blind_spot(
        1.0*frame->r3d2_set_blind_spot.begin/1000,
        1.0*frame->r3d2_set_blind_spot.end/1000);
      rome_reply_ack(UART_STRAT, frame);
      break;
        
    default:
      break;
  }
}

int main(void)
{
  clock_init();

  uart_init();

  INTLVL_ENABLE_ALL();
  __asm__("sei");

  portpin_dirset(&LED_RUN_PP);
  portpin_dirset(&LED_ERROR_PP);
  portpin_dirset(&LED_COM_PP);
  
  portpin_dirset(&LED_LR_PP);
  portpin_dirset(&LED_SR_PP);

  r3d2_init();

  rome_reader_init(&rome_reader, UART_STRAT);

  ROME_LOGF(UART_STRAT, INFO, "RST.STATUS=%x booting...\n", RST.STATUS);
  RST.STATUS = 0;

  for(;;) {
    const rome_frame_t *frame;
    while((frame = rome_reader_read(&rome_reader))) rome_handler(frame);
    r3d2_update();
  }
}

