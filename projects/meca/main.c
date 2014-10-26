#include <avr/io.h>
#include <avarix.h>
#include <avarix/intlvl.h>
#include <avarix/portpin.h>
#include <clock/clock.h>
#include <util/delay.h>
#include <uart/uart.h>
#include <ax12/ax12.h>
#include <timer/timer.h>
#include <math.h>
#include <rome/rome.h>
#include "config.h"
#include "arm.h"
#include "barometer.h"
#include "telemetry.h"
#include "pumps_valves.h"

// match timer   
bool match_timer_counting = false;
int32_t match_timer_ms = -1;

// ROME interface
rome_intf_t rome;
// ROME messages handler
void rome_handler(rome_intf_t *intf, const rome_frame_t *frame) {
  switch(frame->mid) {
    case ROME_MID_START_TIMER: {
      uint8_t fid = frame->start_timer.fid;
      match_timer_counting = true;
      ROME_SEND_ACK(intf,fid);
      break;
    }

    case ROME_MID_MECA_SET_ARM: {
      uint8_t fid = frame->meca_set_arm.fid;
      int16_t upper = frame->meca_set_arm.upper;
      int16_t elbow = frame->meca_set_arm.elbow;
      int16_t wrist = frame->meca_set_arm.wrist;

      arm_set_position(A_ELBOW, elbow);
      arm_set_position(A_WRIST, wrist);
      arm_set_position(A_UPPER, upper);

      ROME_SEND_ACK(intf, fid);
      break;
    }

    case ROME_MID_MECA_SET_PUMP: {
      uint8_t fid = frame->meca_set_pump.fid;
      uint8_t n = frame->meca_set_pump.n;
      uint8_t active = frame->meca_set_pump.active;
      
      switch(n) {
        case 0: pump_valves_activate_pump_A(active); break;
        case 1: pump_valves_activate_pump_B(active); break;
        default: break;
      }
 
      ROME_SEND_ACK(intf, fid);
      break;
    }

    case ROME_MID_MECA_SET_SUCKER: {
      uint8_t fid = frame->meca_set_sucker.fid;
      uint8_t n = frame->meca_set_sucker.n;
      uint8_t active = frame->meca_set_sucker.active;

      switch(n) {
        case 0: pump_valves_activate_valve_A(active); break;
        case 1: pump_valves_activate_valve_B(active); break;
        default: break;
      }
         
      ROME_SEND_ACK(intf, fid);
      break;
    }

    case ROME_MID_MECA_SET_POWER: {
      uint8_t fid = frame->meca_set_power.fid;
      uint8_t active = frame->meca_set_power.active;

      if(active)
        portpin_outset(&LED_ERROR_PP);
      else
        portpin_outclr(&LED_ERROR_PP);
      arm_activate_power(active);

      ROME_SEND_ACK(intf, fid);
      break;
    }

    case ROME_MID_MECA_SET_SERVO: {
      uint8_t fid = frame->meca_set_servo.fid;
      uint8_t n = frame->meca_set_servo.n;
      int16_t position = frame->meca_set_servo.position;

      arm_set_external_servo(n,position);

      ROME_SEND_ACK(intf, fid);
    }

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

//NOTE: ax12 methods MUST NOT be called with UART interrupts blocked, and from
// a single "thread". Just call them from the main thread and you'll be safe.

/** @brief Number of sent bytes to drop on read
 *
 * Sent bytes are also received and need to be dropped.
 */
static volatile uint8_t ax12_nsent = 0;

/// Switch UART line for AX-12 module
static void ax12_set_state(ax12_state_t state)
{
  USART_t *const usart = uart_get_usart(UART_AX12);
  if(state == AX12_STATE_WRITE) {
    ax12_nsent = 0;
    while(uart_recv_nowait(UART_AX12) != -1) ;
    portpin_dirset(&PORTPIN_TXDN(usart));
    portpin_outset(&AX12_DIR_PP);
  } else {
    while(ax12_nsent > 0) {
      int c;
      for(int wdog=0; wdog<1000; wdog++) {
        if((c = uart_recv_nowait(UART_AX12)) != -1)
          break;
      }
      ax12_nsent--;
    }
    portpin_dirclr(&PORTPIN_TXDN(usart));
    portpin_outclr(&AX12_DIR_PP);
  }
}

/// Send a character to AX-12
static int8_t ax12_send_char(uint8_t c)
{
  uart_send(UART_AX12, c);
  ax12_nsent++;
  return 0;
}

/// Receive a character from AX-12
static int ax12_recv_char(void)
{
  uint32_t tend = get_uptime_us() + AX12_TIMEOUT_US;
  for(;;) {
    int c = uart_recv_nowait(UART_AX12);
    if(c != -1) {
      return c;
    }
    if(tend <= get_uptime_us()) {
      return -1; // timeout
    }
  }
}

ax12_t ax12 = {
  .send = ax12_send_char,
  .recv = ax12_recv_char,
  .set_state = ax12_set_state,
};

/// Called on uptime timer tick
void update(void)
{
  INTLVL_DISABLE_ALL_BLOCK() {
    uptime += UPDATE_TICK_US;
  }
}

void update_match_timer(void)
{
  // update match timer
  // check match timer
  if(match_timer_ms > 1000*(int32_t)MATCH_DURATION_SECS) {
    // out of time
    arm_activate_power(false);
    // turn off all pumps, open valves
    pump_valves_activate_pump_A(false);
    pump_valves_activate_pump_B(false);
    pump_valves_activate_valve_A(true);
    pump_valves_activate_valve_B(true);
  }
  else {
    // update match timer
    if(match_timer_counting) {
      match_timer_ms += UPDATE_MATCH_TIMER_TICK_US/1000;
    }
  }
  // downlink match timer telemetry
  TM_DL_MATCH_TIMER(match_timer_ms/1000);
}

int main(void)
{
  clock_init();

  // initialize leds
  portpin_dirset(&LED_RUN_PP);
  portpin_dirset(&LED_ERROR_PP);
  portpin_dirset(&LED_COM_PP);
  portpin_dirset(&LED_AN_PP(0));
  portpin_dirset(&LED_AN_PP(1));
  portpin_dirset(&LED_AN_PP(2));
  portpin_dirset(&LED_AN_PP(3));

  portpin_outset(&LED_RUN_PP);
  portpin_outset(&LED_ERROR_PP);
  portpin_outset(&LED_COM_PP);
  _delay_ms(500);
  portpin_outclr(&LED_RUN_PP);
  portpin_outclr(&LED_ERROR_PP);
  portpin_outclr(&LED_COM_PP);

  // initialize uarts
  uart_init();
  uart_fopen(UART_PPP);

  // intialize barometers
  barometer_t baro0, baro1;
  // ADCA4
  barometer_init(&baro0, &ADCA, ADC_CH_MUXPOS_PIN4_gc);
  // ADCA5
  barometer_init(&baro1, &ADCA, ADC_CH_MUXPOS_PIN5_gc);

  // initialize pumps
  pump_valves_init();

  // initialize arm
  arm_init();

  INTLVL_ENABLE_ALL();
  __asm__("sei");

  // timer
  timer_init();
  timer_set_callback(timerE0, 'A', TIMER_US_TO_TICKS(E0,UPDATE_TICK_US), UPTIME_INTLVL, update);
  timer_set_callback(timerE0, 'B', TIMER_US_TO_TICKS(E0,UPDATE_MATCH_TIMER_TICK_US), UPTIME_INTLVL, update_match_timer);

  // Initialize ROME
  rome_intf_init(&rome);
  rome.uart = UART_PPP;
  rome.handler = rome_handler;
 
  // start arm calibration procedure
  arm_start_calibration();

  uint32_t luptime = UINT32_MAX;
  uint32_t lluptime = UINT32_MAX;

  arm_set_external_servo(S_LEFT, 120);
  arm_set_external_servo(S_RIGHT, -120);

  uint32_t t = 0;
  bool setted_up = false;
  // main loop
  for(;;) {
    
    // debug info
    PORTA.OUT = (t++)/1000;

    // update arm every 100 ms
    uptime = get_uptime_us();
    if(uptime - luptime > UPDATE_ARM_US) {
      luptime = uptime;
      // update arm
      arm_update();

      // update barometers
      uint16_t a = barometer_get_pressure(&baro0);
      uint16_t b = barometer_get_pressure(&baro1);
      TM_DL_SUCKERS(a < 250, b < 250);
    }

    if(!setted_up && arm_is_running()) {
      setted_up = true;
      arm_set_position(A_UPPER, -10000);
      arm_set_position(A_ELBOW, 400);
      arm_set_position(A_WRIST, -200);
    }

    // update rome every 100 ms
    uptime = get_uptime_us();
    if(uptime - lluptime > 100000) {
      lluptime = uptime;
      portpin_outset(&LED_COM_PP);
      rome_handle_input(&rome);
      portpin_outclr(&LED_COM_PP);
    }
  }
}
