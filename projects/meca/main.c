/*
 *  Copyright RobOtter (2016)
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
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
#include "arms.h"
#include "servo_hat.h"
#include <i2c/i2c.h>

#define ROME_DEVICE  ROME_ENUM_DEVICE_GALIPEUR_MECA


// match timer
bool match_started = false;
uint32_t match_timer_ms = 0;

// ROME interfaces
rome_reader_t rome_strat;

void rome_strat_handler(const rome_frame_t *frame)
{
  switch(frame->mid) {
    case ROME_MID_START_TIMER: {
      match_started = true;
      arm_deploy_wings(&arm_l);
      arm_deploy_wings(&arm_r);
      rome_reply_ack(UART_STRAT, frame);
    } break;

    case ROME_MID_MECA_GRAB_WINGS: {
      ROME_LOGF(UART_STRAT, DEBUG, "MECA: %s grab wings", SIDE_NAME(frame->meca_grab_wings.side));
      arm_grab_wings(SIDE_ARM(frame->meca_grab_wings.side));
      rome_reply_ack(UART_STRAT, frame);
      break;
    }

    case ROME_MID_MECA_DEPLOY_WINGS: {
      ROME_LOGF(UART_STRAT, DEBUG, "MECA: %s deploy wings", SIDE_NAME(frame->meca_deploy_wings.side));
      arm_deploy_wings(SIDE_ARM(frame->meca_deploy_wings.side));
      rome_reply_ack(UART_STRAT, frame);
      break;
    }

    case ROME_MID_MECA_FOLD_WINGS: {
      ROME_LOGF(UART_STRAT, DEBUG, "MECA: %s fold wings", SIDE_NAME(frame->meca_fold_wings.side));
      arm_fold_wings(SIDE_ARM(frame->meca_fold_wings.side));
      rome_reply_ack(UART_STRAT, frame);
      break;
    }

    case ROME_MID_MECA_TAKE_CANS: {
      ROME_LOGF(UART_STRAT, DEBUG, "MECA: %s take cans", SIDE_NAME(frame->meca_take_cans.side));
      arm_take_cans(SIDE_ARM(frame->meca_take_cans.side));
      rome_reply_ack(UART_STRAT, frame);
      break;
    }

    case ROME_MID_MECA_RELEASE_CANS: {
      ROME_LOGF(UART_STRAT, DEBUG, "MECA: %s release cans", SIDE_NAME(frame->meca_release_cans.side));
      arm_release_cans(SIDE_ARM(frame->meca_release_cans.side));
      rome_reply_ack(UART_STRAT, frame);
      break;
    }

    case ROME_MID_MECA_MOVE_ELEVATOR: {
      ROME_LOGF(UART_STRAT, DEBUG, "MECA: %s move elevator to %u", SIDE_NAME(frame->meca_move_elevator.side), frame->meca_move_elevator.pos_mm);
      arm_elevator_move(SIDE_ARM(frame->meca_move_elevator.side), ARM_MM_TO_STEPS(frame->meca_move_elevator.side, frame->meca_move_elevator.pos_mm));
      rome_reply_ack(UART_STRAT, frame);
    } break;

    case ROME_MID_MECA_SHUTDOWN_ELEVATOR: {
      ROME_LOGF(UART_STRAT, DEBUG, "MECA: %s shutdown elevator", SIDE_NAME(frame->meca_shutdown_elevator.side));
      arm_elevator_shutdown(SIDE_ARM(frame->meca_shutdown_elevator.side));
      rome_reply_ack(UART_STRAT, frame);
      break;
    }

    case ROME_MID_MECA_SET_POWER: {
      if(frame->meca_set_power.active) {
        portpin_outset(&LED_RUN_PP);
      } else {
        portpin_outclr(&LED_RUN_PP);
      }
      rome_reply_ack(UART_STRAT, frame);
    } break;

    default:
      break;
  }
}

// Handle input from all ROME interfaces
void update_rome_interfaces(void)
{
  portpin_outtgl(&LED_COM_PP);
  rome_reader_handle_input(&rome_strat, rome_strat_handler);
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
    portpin_outset(&LED_AN_PP(3));
    arms_shutdown();
  }
}

void send_telemetry(void)
{
  ROME_SEND_MECA_TM_STATE(UART_STRAT, arms_get_tm_state());
  ROME_SEND_MECA_TM_ARMS_STATE(UART_STRAT,
    arm_l.elevator.pos_known ? (int16_t)ARM_STEPS_TO_MM(arm_l.side, arm_l.elevator.pos) : -1,
    arm_r.elevator.pos_known ? (int16_t)ARM_STEPS_TO_MM(arm_l.side, arm_r.elevator.pos) : -1);
  ROME_SEND_TM_MATCH_TIMER(UART_STRAT, ROME_DEVICE, match_timer_ms/1000);
}

void arm_l_update(void){ arm_update(&arm_l); };
void arm_r_update(void){ arm_update(&arm_r); };

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

  i2c_init();

  // initialize uarts
  uart_init();
  uart_fopen(UART_STRAT);

  INTLVL_ENABLE_ALL();
  __asm__("sei");

  uptime_init();
  TIMER_SET_CALLBACK_US(E0, 'B', UPDATE_MATCH_TIMER_TICK_US, INTLVL_HI, update_match_timer);

  idle_set_callback(rome_update, update_rome_interfaces);
  idle_set_callback(rome_telemetry, send_telemetry);
  idle_set_callback(arm_l_update, arm_l_update);
  idle_set_callback(arm_r_update, arm_r_update);

  arms_init();

  // Initialize ROME
  rome_reader_init(&rome_strat, UART_STRAT);

  for(;;) {
    idle();
  }
}

