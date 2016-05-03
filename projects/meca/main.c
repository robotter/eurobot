#include <avr/io.h>
#include <avarix.h>
#include <avarix/intlvl.h>
#include <avarix/portpin.h>
#include <clock/clock.h>
#include <util/delay.h>
#include <uart/uart.h>
#include <ax12/ax12.h>
#include <timer/timer.h>
#include <timer/uptime.h>
#include <math.h>
#include <rome/rome.h>
#include "config.h"
#include "telemetry.h"
#include <pwm/motor.h>
#include "servos.h"

// match timer
bool match_started = false;
int32_t match_timer_ms = -1;

// ROME interface
rome_intf_t rome_strat;
struct {
  rome_intf_t left,right;
}rome_color_sensor;

//sand roller
void sand_roller_off(void) { servo_set(0,1500); }
void sand_roller_on(void)  { servo_set(0,1800); }

void rome_strat_handler(rome_intf_t *intf, const rome_frame_t *frame)
{

  switch(frame->mid) {
    case ROME_MID_START_TIMER: {
      match_started = true;
      rome_reply_ack(intf, frame);
    } break;

    case ROME_MID_MECA_SET_SERVO: {
        servo_set(frame->asserv_set_servo.id,frame->asserv_set_servo.value);
        rome_reply_ack(intf, frame);
    } break;

    case ROME_MID_MECA_SET_POWER: {
      uint8_t active = frame->meca_set_power.active;
      if(active) {
        portpin_outset(&LED_ERROR_PP);
      } else {
        portpin_outclr(&LED_ERROR_PP);
      }
      rome_reply_ack(intf, frame);
    } break;

    case ROME_MID_MECA_SET_SAND_ROLLER: {
      uint8_t active = frame->meca_set_sand_roller.active;
      if(active)
        sand_roller_on();
      else
        sand_roller_off();
    }
    default:
      break;
  }
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

    // unlock IRQs
    INTLVL_ENABLE_BLOCK(UART_INTLVL) {
      while(uart_recv_nowait(UART_AX12) != -1) ;
    }
    portpin_dirset(&PORTPIN_TXDN(usart));
    portpin_outset(&AX12_DIR_PP);
  } else {
    INTLVL_ENABLE_BLOCK(UART_INTLVL) {
      while(ax12_nsent > 0) {
        int c;
        for(int wdog=0; wdog<1000; wdog++) {
          if((c = uart_recv_nowait(UART_AX12)) != -1)
            break;
        }
        ax12_nsent--;
      }
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
  uint32_t tend = uptime_us() + AX12_TIMEOUT_US;
  for(;;) {
    int c = uart_recv_nowait(UART_AX12);
    if(c != -1) {
      return c;
    }
    if(tend <= uptime_us()) {
      return -1; // timeout
    }
  }
}

ax12_t ax12 = {
  .send = ax12_send_char,
  .recv = ax12_recv_char,
  .set_state = ax12_set_state,
};

static void open_store(void){
  ax12_write_word(&ax12, SE_AX12_STORE_ID, AX12_ADDR_MOVING_SPEED_L,  0x05F);
  ax12_write_byte(&ax12, SE_AX12_STORE_ID, AX12_ADDR_TORQUE_ENABLE,   0x01);
  ax12_write_word(&ax12, SE_AX12_STORE_ID, AX12_ADDR_GOAL_POSITION_L, SE_STORE_OPEN);
}

static void close_store(void){
  ax12_write_word(&ax12, SE_AX12_STORE_ID, AX12_ADDR_MOVING_SPEED_L,  0x1FF);
  ax12_write_byte(&ax12, SE_AX12_STORE_ID, AX12_ADDR_TORQUE_ENABLE,   0x01);
  ax12_write_word(&ax12, SE_AX12_STORE_ID, AX12_ADDR_GOAL_POSITION_L, SE_STORE_CLOSE);
}

void update_match_timer(void)
{
  // update match timer
  if(match_started) {
    match_timer_ms += UPDATE_MATCH_TIMER_TICK_US/1000;
  }

  // downlink match timer telemetry
  TM_DL_MATCH_TIMER(match_timer_ms/1000);
}

int main(void)
{
  clock_init();

  // timer
  timer_init();

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
  portpin_dirset(&AX12_DIR_PP);

  servos_init();

  sand_roller_off();

  INTLVL_ENABLE_ALL();
  __asm__("sei");

  //uptime
  uptime_init();
  TIMER_SET_CALLBACK_US(E0, 'B', UPDATE_MATCH_TIMER_TICK_US, INTLVL_HI, update_match_timer);

  // Initialize ROME
  rome_intf_init(&rome_strat);
  rome_strat.uart = UART_PPP;
  rome_strat.handler = rome_strat_handler;
 
  uint32_t luptime = UINT32_MAX;
  //uint32_t t = 0;

  close_store();

  // main loop
  for(;;) {
    
    // debug info
    //PORTA.OUT = (t++)/1000;
    uint32_t uptime = uptime_us();

    // update rome every 100 ms
    uptime = uptime_us();
    if(uptime - luptime > 100000) {
      luptime = uptime;
      portpin_outtgl(&LED_COM_PP);
      rome_handle_input(&rome_strat);
    }

    // check match timer
    if(match_timer_ms > 1000*(int32_t)MATCH_DURATION_SECS) {
      sand_roller_off();
    }

    //check store deploy timer
    if(match_timer_ms > 1000*(int32_t)STORE_DEPLOY_TIME_SECS) {
      open_store();
    }
  }
}

