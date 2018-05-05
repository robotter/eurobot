#include <avarix.h>
#include <avarix/intlvl.h>
#include <avarix/portpin.h>
#include <clock/clock.h>
#include <util/delay.h>
#include <uart/uart.h>
#include <ax12/ax12.h>
#include <timer/timer.h>
#include <timer/uptime.h>
#include <idle/idle.h>
#include <rome/rome.h>
#include "config.h"
#include <pwm/motor.h>
#include "servos.h"
#include "cylinder.h"
#include "jevois_cam.h"

// match timer
bool match_started = false;
uint32_t match_timer_ms = 0;

// ROME interfaces
rome_intf_t rome_strat;
rome_intf_t rome_jevois;

jevois_cam_t cam;

void rome_strat_handler(rome_intf_t *intf, const rome_frame_t *frame)
{
  switch(frame->mid) {
    case ROME_MID_START_TIMER: {
      match_started = true;
      rome_reply_ack(intf, frame);
    } break;

    case ROME_MID_MECA_CMD: {
      ROME_LOGF(&rome_strat, DEBUG, "MECA: cmd %d",frame->meca_cmd.cmd);
      switch(frame->meca_cmd.cmd) {
        case ROME_ENUM_MECA_COMMAND_CHECK_EMPTY:
          cylinder_check_empty();
          break;
        case ROME_ENUM_MECA_COMMAND_PREPARE_LOAD_WATER:
          cylinder_load_water(false);
          break;
        case ROME_ENUM_MECA_COMMAND_LOAD_WATER:
          cylinder_load_water(true);
          break;
        case ROME_ENUM_MECA_COMMAND_PREPARE_THROW_WATERTOWER:
          cylinder_throw_watertower(false);
          break;
        case ROME_ENUM_MECA_COMMAND_THROW_WATERTOWER:
          cylinder_throw_watertower(true);
          break;
        case ROME_ENUM_MECA_COMMAND_PREPARE_TRASH_TREATMENT:
          cylinder_trash_treatment(false);
          break;
        case ROME_ENUM_MECA_COMMAND_TRASH_TREATMENT:
          cylinder_trash_treatment(true);
          break;
        case ROME_ENUM_MECA_COMMAND_TRASH_BEGINMATCH:
          cylinder_trash_beginmatch();
          break;
        case ROME_ENUM_MECA_COMMAND_THROW_OFFCUP:
          cylinder_throw_offcup();
          break;
        case ROME_ENUM_MECA_COMMAND_NONE:
        default:
          break;
      }
      rome_reply_ack(intf, frame);
    } break;

    case ROME_MID_MECA_SET_POWER: {
      uint8_t active = frame->meca_set_power.active;
      if(active) {
        portpin_outset(&LED_RUN_PP);
      } else {
        portpin_outclr(&LED_RUN_PP);
      }
      rome_reply_ack(intf, frame);
    } break;

    case ROME_MID_MECA_SET_ROBOT_COLOR: {
      uint8_t green = frame->meca_set_robot_color.green;
      if(green) {
        cylinder_set_robot_color(ROME_ENUM_JEVOIS_COLOR_GREEN);
      } else {
        cylinder_set_robot_color(ROME_ENUM_JEVOIS_COLOR_ORANGE);
      }
      rome_reply_ack(intf, frame);
    } break;
    case ROME_MID_MECA_SET_THROW_POWER:
      cylinder_set_throw_power(frame->meca_set_throw_power.pwr);
      rome_reply_ack(intf, frame);
      break;
    case ROME_MID_MECA_SET_TRASH_POWER:
      cylinder_set_throw_power(frame->meca_set_trash_power.pwr);
      rome_reply_ack(intf, frame);
      break;
    default:
      break;
  }
}

void rome_jevois_handler(rome_intf_t *intf, const rome_frame_t *frame)
{
  jevois_cam_process_rome(&cam, frame);

  //send to strat (and then paddock) one message every 100ms
  static uint32_t lmt = 0;
  if (uptime_us() - lmt > 100000){
    rome_send(&rome_strat, frame);
    lmt = uptime_us();
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
        for(int wdog=0; wdog<1000; wdog++) {
          if(uart_recv_nowait(UART_AX12) != -1)
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

void update_match_timer(void)
{
  if(!match_started) {
    return;
  }

  match_timer_ms += UPDATE_MATCH_TIMER_TICK_US/1000;

  if(match_timer_ms > 1000 * (uint32_t)MATCH_DURATION_SECS) {
    portpin_outset(&LED_AN_PP(0));
    cylinder_shutdown();
  }
}

void update_rome_strat(void)
{
  portpin_outtgl(&LED_COM_PP);
  rome_handle_input(&rome_strat);
}

void update_rome_jevois(void)
{
  rome_handle_input(&rome_jevois);
}

void send_telemetry(void)
{
  ROME_SEND_MECA_TM_STATE(&rome_strat,cylinder_get_tm_state());
  ROME_SEND_MECA_TM_OPTIMAL_EMPTYING_MOVE(&rome_strat,cylinder_get_tm_optimal_move());
  ROME_SEND_MECA_TM_CYLINDER_STATE(&rome_strat,
    CYLINDER_NB_POS,
    cylinder_count_empty_slots(),
    cylinder_count_good_water(),
    cylinder_count_bad_water(),
    cylinder.ball_color,
    CYLINDER_NB_POS);
  //convert ax12 angle to milli radians
  int16_t a = ( cylinder_get_position() - cylinder_get_position_zero() )
    *(300./1023.*M_PI/180.*1000.);
  ROME_SEND_MECA_TM_CYLINDER_POSITION(&rome_strat, a );

  ROME_SEND_MECA_TM_MATCH_TIMER(&rome_strat, match_timer_ms/1000);
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
  uart_fopen(UART_STRAT);
  portpin_dirset(&AX12_DIR_PP);

  servos_init();

  // Initialize jevois camera
  jevois_cam_init(&cam);

  INTLVL_ENABLE_ALL();
  __asm__("sei");

  uptime_init();
  TIMER_SET_CALLBACK_US(E0, 'B', UPDATE_MATCH_TIMER_TICK_US, INTLVL_HI, update_match_timer);

  idle_set_callback(rome_strat_update, update_rome_strat);
  idle_set_callback(rome_telemetry, send_telemetry);
  idle_set_callback(rome_jevois_update, update_rome_jevois);
  idle_set_callback(cylinder_update, cylinder_update);

  cylinder_init();

  // Initialize ROME
  rome_intf_init(&rome_strat);
  rome_strat.uart = UART_STRAT;
  rome_strat.handler = rome_strat_handler;

  rome_intf_init(&rome_jevois);
  rome_jevois.uart = UART_JEVOIS;
  rome_jevois.handler = rome_jevois_handler;

  for(;;) {
    idle();
  }
}

