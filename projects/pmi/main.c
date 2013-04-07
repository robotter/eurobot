#include <avr/eeprom.h>
#include <avarix/portpin.h>
#include <avarix/intlvl.h>
#include <clock/clock.h>
#include <uart/uart.h>
#include <perlimpinpin/perlimpinpin.h>
#include <perlimpinpin/payload/system.h>
#include <perlimpinpin/payload/log.h>
#include <perlimpinpin/payload/room.h>
#include <timer/timer.h>
#include "position.h"
#include "trajectory.h"
#include "ramp.h"
#include "pid.h"
#include "motors.h"
#include "config.h"


extern ppp_payload_handler_t *ppp_filter(ppp_intf_t *intf);
extern void room_message_handler(ppp_intf_t *intf, room_payload_t *pl);

static ppp_intf_t pppintf;

position_t pos_man;
traj_t traj_man;
ramp_t ramp_dist;
ramp_t ramp_angle;
pid_t pid_dist;
pid_t pid_angle;

static EEMEM position_conf_t pos_man_conf;
static EEMEM traj_conf_t traj_man_conf;
static EEMEM ramp_conf_t ramp_dist_conf;
static EEMEM ramp_conf_t ramp_angle_conf;
static EEMEM pid_conf_t pid_dist_conf;
static EEMEM pid_conf_t pid_angle_conf;



void manage_control_system(void)
{
  // update encoders
  motors_update_encoders();

  // compute new position
  pos_set_encoder_values(&pos_man, motor_left_encoder_value(), motor_right_encoder_value());
  pos_do_computation(&pos_man);

  // compute trajectory
  traj_do_computation(&traj_man);

  // comupute consign filters
  ramp_set_consign(&ramp_dist, traj_get_d_output(&traj_man));
  ramp_set_consign(&ramp_angle, traj_get_a_output(&traj_man));
  ramp_do_computation(&ramp_dist);
  ramp_do_computation(&ramp_angle);

  // compute pid
  pid_set_consign(&pid_dist, ramp_get_output(&ramp_dist));
  pid_set_consign(&pid_angle, ramp_get_output(&ramp_angle));
  pid_set_feedback(&pid_dist, pos_get_d(&pos_man));
  pid_set_feedback(&pid_angle, pos_get_a(&pos_man));
  pid_do_computation(&pid_dist);
  pid_do_computation(&pid_angle);

  // compute motors consign
  double rcons = pid_get_output(&pid_dist) + pid_get_output(&pid_angle);
  double lcons = pid_get_output(&pid_dist) - pid_get_output(&pid_angle);

  motors_set_consign(lcons, rcons);
}


int main(void)
{
  clock_init();
  uart_init();
  uart_fopen(uartC1);
  INTLVL_ENABLE_ALL();

  // port pin configuration
  portpin_dirset(&AX12_DIR_PP);
  portpin_outset(&AX12_DIR_PP);  // required for SPI as master
  portpin_dirset(&LED_GREEN_PP);
  portpin_dirset(&LED_RED_PP);
  portpin_dirset(&LED_BLUE_PP);

  // PPP init
  pppintf.filter = ppp_filter;
  pppintf.uart = uartC1;
  pppintf.addr = PPP_ADDR;
  ppp_intf_init(&pppintf);
  room_set_message_handler(room_message_handler);
  // send a system RESET to signal that we have booted
  ppp_send_system_reset(&pppintf);

  // position system
  pos_init(&pos_man);
  pos_conf_load(&pos_man, &pos_man_conf);
  // control system
  ramp_init(&ramp_dist);
  ramp_conf_load(&ramp_dist, &ramp_dist_conf);
  ramp_init(&ramp_angle);
  ramp_conf_load(&ramp_angle, &ramp_angle_conf);
  pid_init(&pid_dist);
  pid_conf_load(&pid_dist, &pid_dist_conf);
  pid_init(&pid_angle);
  pid_conf_load(&pid_angle, &pid_angle_conf);
  // motors
  motors_init();
  // trajectory
  traj_init(&traj_man, &pos_man);
  traj_conf_load(&traj_man, &traj_man_conf);

  //TODO battery voltage on A3 (analogic input), multiply read value by 7.8

  timer_set_callback(timerC0, 'A', TIMER_US_TO_TICKS(C0,CONTROL_SYSTEM_PERIOD_US), CONTROL_SYSTEM_INTLVL, manage_control_system);

  /*
   * main loop
   *
   * There are two "threads", excluding low-level ones from modules:
   *  - main: just below, PPP (order handling, ...)
   *  - asserv: manage_control_system() call
   */
  for(;;) {
    ppp_intf_update(&pppintf);
  }
}


