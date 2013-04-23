#include <avr/io.h>
#include <avarix/intlvl.h>
#include <clock/clock.h>
#include <uart/uart.h>
#include <ax12/ax12.h>
#include <pwm/motor.h>
#include <timer/timer.h>
#include <perlimpinpin/perlimpinpin.h>
#include <perlimpinpin/payload/system.h>
#include <perlimpinpin/payload/room.h>
#include <perlimpinpin/payload/log.h>
#include "cake.h"
#include "config.h"


static pwm_motor_t pwm_balloon;
static ppp_intf_t pppintf;


/// current time in microseconds
volatile uint32_t uptime;

/// Called on uptime timer tick
void uptime_update(void)
{
  uptime += UPTIME_TICK_US;
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
  USART_t *usart = uart_get_usart(UART_AX12);
  if(state == AX12_STATE_WRITE) {
    ax12_nsent = 0;
    while(uart_recv_nowait(UART_AX12) != -1) ;
    usart->CTRLB |= USART_TXEN_bm;
    portpin_outset(&AX12_DIR_PP);
  } else {
    while(ax12_nsent > 0) {
      while(uart_recv_nowait(UART_AX12) == -1) ;
      ax12_nsent--;
    }
    portpin_outclr(&AX12_DIR_PP);
    //usart->CTRLB &= ~USART_TXEN_bm;
    PORTC.DIRCLR = _BV(7);

    while(!(usart->STATUS & USART_DREIF_bm)) ;
    usart->CTRLB &= ~USART_TXEN_bm;
    while(!(usart->STATUS & USART_TXCIF_bm)) ;
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
  uint32_t tend = uptime + AX12_TIMEOUT_US;
  for(;;) {
    int c = uart_recv_nowait(UART_AX12);
    if(c != -1) {
      if(ax12_nsent == 0) {
        return c;
      }
      ax12_nsent--;
    }
    if(tend <= uptime) {
      return -1; // timeout
    }
  }
}


static ax12_t ax12 = {
  .send = ax12_send_char,
  .recv = ax12_recv_char,
  .set_state = ax12_set_state,
};



ppp_payload_handler_t *ppp_filter(ppp_intf_t *intf)
{
  if(intf->rstate.header.dst != 0xFF && intf->rstate.header.dst != intf->addr) {
    return NULL;
  }
  switch(intf->rstate.header.pltype) {
    case PPP_TYPE_SYSTEM:
      return ppp_payload_handler_system;
    case PPP_TYPE_ROOM:
      return ppp_payload_handler_room;
    default:
      return NULL;
  }
}


void room_message_handler(ppp_intf_t *intf, room_payload_t *pl)
{
  switch(pl->mid) {
    // AX-12
    case ROOM_MID_AX12_READ_BYTE: {
      uint8_t data = 0xFF;
      uint8_t error = ax12_read_byte(&ax12, pl->ax12_read_byte.id, pl->ax12_read_byte.addr, &data);
      ROOM_REPLY_AX12_READ_BYTE(intf, pl, data, error);
    } break;
    case ROOM_MID_AX12_READ_WORD: {
      uint16_t data = 0xFFFF;
      uint8_t error = ax12_read_word(&ax12, pl->ax12_read_word.id, pl->ax12_read_word.addr, &data);
      ROOM_REPLY_AX12_READ_BYTE(intf, pl, data, error);
    } break;
    case ROOM_MID_AX12_WRITE_BYTE: {
      uint8_t error = ax12_write_byte(&ax12, pl->ax12_write_byte.id, pl->ax12_write_byte.addr, pl->ax12_write_byte.data);
      ROOM_REPLY_AX12_WRITE_BYTE(intf, pl, error);
    } break;
    case ROOM_MID_AX12_WRITE_WORD: {
      uint8_t error = ax12_write_word(&ax12, pl->ax12_write_word.id, pl->ax12_write_word.addr, pl->ax12_write_word.data);
      ROOM_REPLY_AX12_WRITE_WORD(intf, pl, error);
    } break;
    case ROOM_MID_AX12_MOVE: {
      // GOAL_POSITION and MOVING_SPEED are low-endian and contiguous
      // it allows to directly use the ROOM payload buffer
      ax12_write_mem(&ax12, pl->ax12_move.id, AX12_ADDR_GOAL_POSITION_L, 4, (uint8_t*)&pl->ax12_move.pos);
      ROOM_REPLY_AX12_MOVE(intf, pl);
    } break;
    case ROOM_MID_AX12_STATE: {
      uint16_t pos = 0xFFFF;
      ax12_read_word(&ax12, pl->ax12_state.id, AX12_ADDR_PRESENT_POSITION_L, &pos);
      uint8_t moving = 0;
      ax12_read_byte(&ax12, pl->ax12_state.id, AX12_ADDR_MOVING, &moving);
      ROOM_REPLY_AX12_STATE(intf, pl, pos, moving);
    } break;

    // rule specific
    case ROOM_MID_MECA_SET_CAKE_ANGLE: {
      cake_set_angle(pl->meca_set_cake_angle.a);
      ROOM_REPLY_MECA_SET_CAKE_ANGLE(intf, pl);
    } break;
    case ROOM_MID_MECA_GET_CAKE_ANGLE: {
      ROOM_REPLY_MECA_GET_CAKE_ANGLE(intf, pl, cake_get_angle());
    } break;
    case ROOM_MID_MECA_BALLOON_TAP: {
      pwm_motor_set(&pwm_balloon, pl->meca_balloon_tap.open ? SERVO_BALLOON_OPEN_POS : SERVO_BALLOON_CLOSE_POS);
      ROOM_REPLY_MECA_BALLOON_TAP(intf, pl);
    } break;

    default:
      PPP_LOGF(intf, INFO, "unexpected ROOM message: %u", pl->mid);
      break;
  }
}


int main(void)
{
  clock_init();
  timer_init();
  uart_init();
  uart_fopen(UART_PPP);
  CPU_SREG |= CPU_I_bm;
  INTLVL_ENABLE_ALL();

  // timer
  timer_set_callback(timerC0, 'A', TIMER_US_TO_TICKS(C0,UPTIME_TICK_US), INTLVL_LO, uptime_update);

  // init PPP
  pppintf.filter = ppp_filter;
  pppintf.uart = UART_PPP;
  pppintf.addr = PPP_ADDR;

  ppp_intf_init(&pppintf);
  room_set_message_handler(room_message_handler);
  // send a system RESET to signal that we have booted
  ppp_send_system_reset(&pppintf);

  // balloon servo, for the funny action
  pwm_servo_init(&pwm_balloon, &TCD0, 'A');
  pwm_motor_set(&pwm_balloon, SERVO_BALLOON_CLOSE_POS);

  // init AX-12
  portpin_dirset(&AX12_DIR_PP);
  portpin_outclr(&AX12_DIR_PP);

  // main loop
  for(;;) {
    ppp_intf_update(&pppintf);
  }
}

