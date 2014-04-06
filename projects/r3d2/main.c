#include <math.h>
#include <avarix/intlvl.h>
#include <avarix/portpin.h>
#include <clock/clock.h>
#include <uart/uart.h>
#include <timer/timer.h>
#include <rome/rome.h>
#include "r3d2.h"
#include "config.h"


static r3d2_data_t r3d2_data;
static rome_intf_t rome_intf;


static void rome_handler(rome_intf_t *intf, const rome_frame_t *frame)
{
  /*TODO
  switch(frame->mid) {
  }
  */
}


void update_data_cb(void)
{
  r3d2_update(&r3d2_data);
}

void send_rome_events_cb(void)
{
  // north east south west
  bool leds[4] = {false, false, false, false};

  uint8_t i = 0;
  for(i=0; i<r3d2_data.count; i++) {
    const r3d2_object_t *object = r3d2_data.objects+i;
    //TODO ROME_SEND_R3D2_DETECTED(&rome_intf, i, object->angle*1000, object->dist*1000);
    int8_t iangle = object->angle/M_PI_4;
    leds[((iangle+1)/2) % 4] = true;
  }

  if(leds[0]) portpin_outset(&LED_WEST_PP); else portpin_outclr(&LED_WEST_PP);
  if(leds[1]) portpin_outset(&LED_SOUTH_PP); else portpin_outclr(&LED_SOUTH_PP);
  if(leds[2]) portpin_outset(&LED_NORTH_PP); else portpin_outclr(&LED_NORTH_PP);
  if(leds[3]) portpin_outset(&LED_EAST_PP); else portpin_outclr(&LED_EAST_PP);
}


int main(void)
{
  clock_init();
  timer_init();
  uart_init();
  CPU_SREG |= CPU_I_bm;
  INTLVL_ENABLE_ALL();

  portpin_dirset(&LED_RUN_PP);
  portpin_dirset(&LED_ERROR_PP);
  portpin_dirset(&LED_COM_PP);
  portpin_dirset(&LED_WEST_PP);
  portpin_dirset(&LED_EAST_PP);
  portpin_dirset(&LED_NORTH_PP);
  portpin_dirset(&LED_SOUTH_PP);

  rome_intf_init(&rome_intf);
  rome_intf.uart = uartF1;
  rome_intf.handler = rome_handler;

  // init R3D2
  r3d2_init();
  r3d2_conf_load();
  r3d2_start();

  timer_set_callback(timerE0, 'A', TIMER_US_TO_TICKS(E0,20000), INTLVL_LO, update_data_cb);
  timer_set_callback(timerE0, 'B', TIMER_US_TO_TICKS(E0,200000), INTLVL_LO, send_rome_events_cb);

  // main loop
  for(;;) {
    rome_handle_input(&rome_intf);
  }
}

