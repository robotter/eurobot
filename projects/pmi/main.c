#include <avr/eeprom.h>
#include <avarix/portpin.h>
#include <avarix/intlvl.h>
#include <clock/clock.h>
#include <util/delay.h>
#include <uart/uart.h>
#include <perlimpinpin/perlimpinpin.h>
#include <perlimpinpin/payload/system.h>
#include <perlimpinpin/payload/log.h>
#include <perlimpinpin/payload/room.h>
#include <timer/timer.h>
#include <pwm/motor.h>
#include <i2c/i2c.h>
#include <string.h>
#include "position.h"
#include "trajectory.h"
#include "ramp.h"
#include "pid.h"
#include "motors.h"
#include "battery_monitor.h"
#include "config.h"


extern ppp_payload_handler_t *ppp_filter(ppp_intf_t *intf);
extern void room_message_handler(ppp_intf_t *intf, room_payload_t *pl);

ppp_intf_t pppintf;

position_t pos_man;
traj_t traj_man;
ramp_t ramp_dist;
ramp_t ramp_angle;
pid_t pid_dist;
pid_t pid_angle;

EEMEM position_conf_t pos_man_conf;
EEMEM traj_conf_t traj_man_conf;
EEMEM ramp_conf_t ramp_dist_conf;
EEMEM ramp_conf_t ramp_angle_conf;
EEMEM pid_conf_t pid_dist_conf;
EEMEM pid_conf_t pid_angle_conf;

volatile int16_t us_rangefinder_distances[2] = {-1, -1};

volatile bool match_is_over = false;
volatile bool match_started = false;
volatile bool debug_rangefinder_using_leds = false;

// Default configurations

static const position_conf_t pos_man_default_conf = {
  .left_wheel_ratio = 1,
  .right_wheel_ratio = -1,
  .tick_p_mm = 41.0,
  .tick_p_180deg = 8700.0,
};

static const ramp_conf_t ramp_dist_default_conf = {
  .a_max = 10000.0,
  .v_max = 50000.0,
};

static const ramp_conf_t ramp_angle_default_conf = {
  .a_max = 5000.0,
  .v_max = 10000.0,
};

static const pid_conf_t pid_dist_default_conf = {
  .kd = 0.5,
  .ki = 0.2,
  .kp = 5.0,
  .d_alpha = 1.0,
  .max_integral = 5000.0,
  .max_output = 30000.0,
};

static const pid_conf_t pid_angle_default_conf = {
  .kd = 0.5,
  .ki = 0.3,
  .kp = 5.0,
  .d_alpha = 1.0,
  .max_integral = 10000.0,
  .max_output = 30000.0,
};

static pwm_motor_t servos[5];

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


void monitor_battery(void)
{
  //TODO battery voltage on A3 (analogic input), multiply read value by 7.8
  battery_monitor_measure();
}


void monitor_rangefinders(void)
{
  uint8_t buf[2][3];
  i2cm_recv(i2cC, 0x70, buf[0], 3);
  i2cm_recv(i2cD, 0x70, buf[1], 3);
  int16_t dist[2] = {
    ((uint16_t)buf[0][2] << 8) + buf[0][1],
    ((uint16_t)buf[1][2] << 8) + buf[1][1],
  };
  (void)dist; //TODO

  int i;
  for(i=0;i<2;i++) {
    if(dist[i] < 0) 
      // on invalid distance (-1) bypass lowpass filter
      us_rangefinder_distances[i] = dist[i];
    else
      us_rangefinder_distances[i] = dist[i];// 0.6*us_rangefinder_distances[i] + dist[i]*0.4;
  }

  if(debug_rangefinder_using_leds) {
    if(us_rangefinder_distances[1] < 0) {
      portpin_outclr(&LED_GREEN_PP);
      portpin_outclr(&LED_BLUE_PP);
      portpin_outtgl(&LED_RED_PP);
    }
    else {
      portpin_outclr(&LED_RED_PP);
      if(us_rangefinder_distances[1] > 20) {
        portpin_outset(&LED_GREEN_PP);
        portpin_outclr(&LED_BLUE_PP);
      }
      else {
        portpin_outclr(&LED_GREEN_PP);
        portpin_outset(&LED_BLUE_PP);
      }
    }
  }
}

void main_update(void) {
  // called every MAIN_PERIOD_US
  
  // check 89s
  static int match_timer = 0; 
  if(match_started) {
    match_timer++;
    if(match_timer > 850) {
      PPP_LOGF(&pppintf, INFO, "MATCH IS OVER");
      match_is_over = true;
    }
  }
  // main loop update
  ppp_intf_update(&pppintf);
}

int main(void)
{
  clock_init();
  uart_init();
  uart_fopen(uartC1);
  timer_init();
  CPU_SREG |= CPU_I_bm;
  INTLVL_ENABLE_ALL();

  // port pin configuration
  portpin_dirset(&AX12_DIR_PP);
  portpin_outset(&AX12_DIR_PP);  // required for SPI as master
  portpin_dirset(&LED_GREEN_PP);
  portpin_dirset(&LED_RED_PP);
  portpin_dirset(&LED_BLUE_PP);

  // do a led cycling for one second
  // NOTE: it allows board reprogramming if something
  // fails, like full motor power right at the beginning.
  // Do not remove this code.
  uint8_t i;
  portpin_outset(&LED_BLUE_PP);
  for(i=0;i<5;i++) {
    portpin_outtgl(&LED_GREEN_PP);
    _delay_ms(100);
    portpin_outtgl(&LED_BLUE_PP);
    _delay_ms(100);
  }
  portpin_outclr(&LED_BLUE_PP);
  portpin_outclr(&LED_GREEN_PP);
  portpin_outclr(&LED_RED_PP);

  // PPP init
  pppintf.filter = ppp_filter;
  pppintf.uart = uartC1;
  pppintf.addr = PPP_ADDR;
  ppp_intf_init(&pppintf);
  room_set_message_handler(room_message_handler);
  // send a system RESET to signal that we have booted
  ppp_send_system_reset(&pppintf);

  // i2c init, for rangefinders
  i2c_init();

  // battery monitoring 
  battery_monitor_init();

  portpin_outset(&LED_RED_PP);
  portpin_outset(&LED_GREEN_PP);
  portpin_outset(&LED_BLUE_PP);

  // battery checking
  int32_t sum = 0;
  int it;
  const int nmeasures = 10;
  for(it=0;it<nmeasures;it++) {
    sum += battery_monitor_measure();
  }
  if(sum/nmeasures < BATTERY_MONITORING_LOW_VOLTAGE_DECIVOLTS) {
    // do a BATTERY LOW GLOW and wait indefinitely here
    double t = 0.0;
    portpin_outclr(&LED_RED_PP);
    portpin_outclr(&LED_GREEN_PP);
    portpin_outclr(&LED_BLUE_PP);
    for(;;) {
      t+=0.015;
      const int N = 100;
      for(it=0;it<N;it++) {
        if(50*(cos(t)+1.0) < it) {
          portpin_outset(&LED_RED_PP);
        }
        else {
          portpin_outclr(&LED_RED_PP);
        }
      }
    }
  }

  // position system
  pos_init(&pos_man);
  pos_conf_load(&pos_man, &pos_man_conf, &pos_man_default_conf);
  // control system
  ramp_init(&ramp_dist);
  ramp_conf_load(&ramp_dist, &ramp_dist_conf, &ramp_dist_default_conf);
  ramp_init(&ramp_angle);
  ramp_conf_load(&ramp_angle, &ramp_angle_conf, &ramp_angle_default_conf);
  pid_init(&pid_dist);
  pid_conf_load(&pid_dist, &pid_dist_conf, &pid_dist_default_conf);
  pid_init(&pid_angle);
  pid_conf_load(&pid_angle, &pid_angle_conf, &pid_angle_default_conf);
  // motors
  motors_init();
  // trajectory
  traj_init(&traj_man, &pos_man);
  traj_conf_load(&traj_man, &traj_man_conf);

  // servos
  pwm_servo_init(&servos[0], (TC0_t*)&SERVO_ANA_0_TC, SERVO_ANA_0_CH);
  pwm_servo_init(&servos[1], (TC0_t*)&SERVO_ANA_1_TC, SERVO_ANA_1_CH);
  pwm_servo_init(&servos[2], (TC0_t*)&SERVO_ANA_2_TC, SERVO_ANA_2_CH);
  pwm_servo_init(&servos[3], (TC0_t*)&SERVO_ANA_3_TC, SERVO_ANA_3_CH);
  pwm_servo_init(&servos[4], (TC0_t*)&SERVO_ANA_4_TC, SERVO_ANA_4_CH);

  timer_set_callback(timerC0, 'A', TIMER_US_TO_TICKS(C0,CONTROL_SYSTEM_PERIOD_US), CONTROL_SYSTEM_INTLVL, manage_control_system);
  timer_set_callback(timerC0, 'B', TIMER_US_TO_TICKS(C0,BATTERY_MONITORING_PERIOD_US), BATTERY_MONITORING_INTLVL, monitor_battery);
  timer_set_callback(timerC0, 'C', TIMER_US_TO_TICKS(C0,RANGEFINDERS_MONITORING_PERIOD_US), RANGEFINDERS_MONITORING_INTLVL, monitor_rangefinders);
  timer_set_callback(timerC0, 'D', TIMER_US_TO_TICKS(C0,MAIN_PERIOD_US), MAIN_INTLVL, main_update);

  /*
   * main loop
   *
   */
  
  pwm_motor_set(&servos[4], ((int32_t)600*32767 / 20000));
  pwm_motor_set(&servos[1], ((int32_t)1500*32767 / 20000));

  _delay_ms(500);

  portpin_outclr(&LED_GREEN_PP);
  portpin_outclr(&LED_RED_PP);
  portpin_outclr(&LED_BLUE_PP);
  double tt = 0.0;
  while(!match_started) {
    tt += 0.003;
    int i;
    const int N = 50;
    for(i=0;i<N;i++) {
      if(0.5*N*(1.0+cos(tt)) < i )
        portpin_outset(&LED_GREEN_PP);
      else
        portpin_outclr(&LED_GREEN_PP);
    }
    portpin_outset(&LED_GREEN_PP);
  }

  debug_rangefinder_using_leds = true;

  bool blocked = false;
  traj_goto_xy(&traj_man, pos_mm_to_tick(&pos_man,1000), 0.0);
  double t = 0;
  while(!(traj_done(&traj_man) && !blocked)) {
    if(match_is_over) 
      break;
    int16_t d = us_rangefinder_distances[1];
    if((d > 0)&&(d < 20)) {
      pwm_motor_set(&servos[4], ((int32_t)(1500 - 450*(1.0+cos(t)) )*32767 / 20000));
      pwm_motor_set(&servos[1], ((int32_t)(550  + 450*(1.0+sin(t)) )*32767 / 20000));
      t+=0.1;
      ramp_reset(&ramp_dist, traj_man.d_cur);
      ramp_reset(&ramp_angle, traj_man.a_cur);
      traj_goto_d(&traj_man, 0);
      blocked = true;
    } else {
      pwm_motor_set(&servos[4], ((int32_t)600*32767 / 20000));
      pwm_motor_set(&servos[1], ((int32_t)1500*32767 / 20000));

      traj_goto_xy(&traj_man, pos_mm_to_tick(&pos_man,1000), 0.0);
      blocked = false;
    }
    _delay_ms(10);
  }

  motors_brake(true);
  timer_set_callback(timerC0, 'A', TIMER_US_TO_TICKS(C0,CONTROL_SYSTEM_PERIOD_US), CONTROL_SYSTEM_INTLVL, NULL);
  motors_set_consign(0,0);

  for(;;) {
    PPP_LOGF(&pppintf, INFO, "MATCH OVER, I'M DONE WITH THAT SHIT !");
    _delay_ms(1000);
  }
}


