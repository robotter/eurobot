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
  switch(frame->mid) {
    case ROME_MID_R3D2_CALIBRATE_ANGLE:
      r3d2_calibrate_angle(1000*frame->r3d2_calibrate_angle.a);
      break;
    case ROME_MID_R3D2_CALIBRATE_DIST:
      r3d2_calibrate_angle(frame->r3d2_calibrate_dist.d);
      break;
    case ROME_MID_R3D2_CONF_LOAD:
      r3d2_conf_load();
      break;
    case ROME_MID_R3D2_CONF_SAVE:
      r3d2_conf_save();
      break;
    case ROME_MID_R3D2_SET_MOTOR_SPEED:
      r3d2_set_motor_speed(frame->r3d2_set_motor_speed.speed);
      break;
    default:
      break;
  }
}

/// current time in microseconds
static volatile uint32_t uptime;

/// Get uptime value
uint32_t get_uptime_us(void)
{
  uint32_t tmp;
  INTLVL_DISABLE_ALL_BLOCK() {
    tmp = uptime;
  }
  return tmp;
}

/// Called on uptime timer tick
static void update_uptime(void)
{
  uptime += UPDATE_TICK_US;
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
  portpin_dirset(&LED_WEST_PP);
  portpin_dirset(&LED_EAST_PP);
  portpin_dirset(&LED_NORTH_PP);
  portpin_dirset(&LED_SOUTH_PP);

  rome_intf_init(&rome_intf);
  rome_intf.uart = UART_ROME;
  rome_intf.handler = rome_handler;

  // init R3D2
  r3d2_init();
  r3d2_conf_load();
  r3d2_start();

  timer_init();
  timer_set_callback(timerE0, 'A', TIMER_US_TO_TICKS(E0,UPDATE_TICK_US),
                     UPTIME_INTLVL, update_uptime);

  // main loop
  uint32_t t_capture = 0;
  uint32_t t_tm = 0;
  for(;;) {
    rome_handle_input(&rome_intf);
    uint32_t t = get_uptime_us();
    if(t > t_capture) {
      r3d2_update(&r3d2_data);
      t_capture = t + CAPTURE_PERIOD_US;
    }
    if(t > t_tm) {
      r3d2_telemetry(&rome_intf, &r3d2_data);
      t_capture = t + TELEMETRY_PERIOD_US;
    }
  }
}

