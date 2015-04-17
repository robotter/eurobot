#include <math.h>
#include <avarix/intlvl.h>
#include <avarix/portpin.h>
#include <clock/clock.h>
#include <uart/uart.h>
#include <timer/timer.h>
#include <timer/uptime.h>
#include "led_config.h"
#include "com_config.h"
#include "rgb_led.h"
#include <rome/rome.h>
#include <stdio.h>

#include "proximity_color_sensor_fsm.h"

// ROME interfaces
rome_intf_t rome;

// ROME messages handler
void rome_handler(rome_intf_t *intf, const rome_frame_t *frame)
{
  switch(frame->mid) {

    case ROME_MID_COLOR_SENSOR_SET_COLOR_FILTER: {
      // TODO
      rome_reply_ack(intf, frame);
    } break;

    case ROME_MID_COLOR_SENSOR_SET_DIST_THRESHOLD: {
      uint16_t low_threshold = frame->color_sensor_set_dist_threshold.low_threshold;
      uint16_t high_threshold = frame->color_sensor_set_dist_threshold.high_threshold;
      uint8_t consecutive_detect_threshold = frame->color_sensor_set_dist_threshold.consecutive_detect_threshold;

      PCSFSM_SetTcs37725ProximityThreshold(low_threshold, high_threshold, consecutive_detect_threshold);
      rome_reply_ack(intf, frame);
    } break;

    default:
      break;
  }
}


int main(void)
{
  clock_init();
  timer_init();
  uart_init();
  uart_fopen(UART_COM);
  CPU_SREG |= CPU_I_bm;
  INTLVL_ENABLE_ALL();

  // leds port init
  PORTC.DIRSET = _BV(6)| _BV(7);

  PCSFSM_Init();
  
  //timer
  timer_init();
  uptime_init();

  //Initialize ROME
  rome_intf_init(&rome);
  rome.uart = UART_COM;
  rome.handler = rome_handler;

 // timer_set_callback(timerE2, 'A', TIMER_US_TO_TICKS(E0,20000), INTLVL_LO, update_data_cb);
 // timer_set_callback(timerE2, 'B', TIMER_US_TO_TICKS(E0,200000), INTLVL_LO, send_ppp_events_cb);

  uint32_t uptime = 0;;
  uint32_t color_sensor_uptime = 0;
  uint32_t telemetry_uptime = 0;

  // main loop
  for(;;) {
    uptime = uptime_us();
    if(uptime - color_sensor_uptime > 10000) {
      color_sensor_uptime = uptime;
      PCSFSM_Update();
    }


    uptime = uptime_us();
    if(uptime - telemetry_uptime >50000) {
      telemetry_uptime = uptime;
      ROME_SEND_COLOR_SENSOR_TM_DETECTION(&rome, PCCFSM_IsObjectDetected()==APDS9800_OBJECT_DETECTED, PCCFSM_GetObjectColor());
      ROME_SEND_COLOR_SENSOR_TM_DIST_DEBUG(&rome, PCSFSM_GetTcs37725ProxDistance(), PCCFSM_IsObjectDetected()==APDS9800_OBJECT_DETECTED, PCCFSM_GetObjectColor());
      }
  }
}

