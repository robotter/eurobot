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
#include "arms.h"
#include "jevois_cam.h"

#define ROME_DEVICE  ROME_ENUM_DEVICE_GALIPEUR_MECA


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
        case ROME_ENUM_MECA_COMMAND_TAKE_ATOMS:
          arm_take_atoms(frame->meca_cmd.side ? arm_l : arm_r);
          break;
        case ROME_ENUM_MECA_COMMAND_RELEASE_ATOMS:
          arm_release_atoms(frame->meca_cmd.side ? arm_l : arm_r);
          break;
        case ROME_ENUM_MECA_COMMAND_ELEVATOR_UP:
          arm_elevator_up(frame->meca_cmd.side ? arm_l : arm_r);
          break;
        case ROME_ENUM_MECA_COMMAND_ELEVATOR_DOWN:
          arm_elevator_down(frame->meca_cmd.side ? arm_l : arm_r);
          break;
        case ROME_ENUM_MECA_COMMAND_NONE:
        default:
          break;
      }
      rome_reply_ack(intf, frame);
    } break;

    //case ROME_MID_MECA_SET_THROW_POWER:
    //  cylinder_set_throw_power(frame->meca_set_throw_power.pwr);
    //  rome_reply_ack(intf, frame);
    //  break;

    case ROME_MID_MECA_SET_POWER: {
      uint8_t active = frame->meca_set_power.active;
      if(active) {
        portpin_outset(&LED_RUN_PP);
      } else {
        portpin_outclr(&LED_RUN_PP);
      }
      rome_reply_ack(intf, frame);
    } break;

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

#if 0 //no ax12 needed on galipeur this year

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
#endif //AX12

void update_match_timer(void)
{
  if(!match_started) {
    return;
  }

  match_timer_ms += UPDATE_MATCH_TIMER_TICK_US/1000;

  if(match_timer_ms > 1000 * (uint32_t)MATCH_DURATION_SECS) {
    portpin_outset(&LED_AN_PP(0));
    arms_shutdown();
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
  ROME_SEND_MECA_TM_STATE(&rome_strat,arms_get_tm_state());
  ROME_SEND_MECA_TM_ARMS_STATE(&rome_strat,
    arm_l.up,
    arm_r.up,
    arm_l.atoms,
    arm_r.atoms);
  ROME_SEND_TM_MATCH_TIMER(&rome_strat, ROME_DEVICE, match_timer_ms/1000);
}

void arm_l_update(void){ arm_update(arm_l); };
void arm_r_update(void){ arm_update(arm_r); };

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
  //portpin_dirset(&AX12_DIR_PP);

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
  idle_set_callback(arm_l_update, arm_l_update);
  idle_set_callback(arm_r_update, arm_r_update);

  arms_init();

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

