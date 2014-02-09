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
#include <perlimpinpin/perlimpinpin.h>
#include <perlimpinpin/payload/system.h>
#include <perlimpinpin/payload/room.h>
#include <perlimpinpin/payload/log.h>
#include "config.h"
#include "arm.h"

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
      while((c = uart_recv_nowait(UART_AX12)) == -1) ;
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
  uptime += UPDATE_TICK_US;
}

// Perlimpinpin
ppp_intf_t pppintf;

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
  portpin_outtgl(&LED_COM_PP);
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

  arm_init();

  portpin_dirset(&LED_RUN_PP);
  portpin_dirset(&LED_ERROR_PP);
  portpin_dirset(&LED_COM_PP);
  portpin_outclr(&LED_RUN_PP);
  portpin_outclr(&LED_ERROR_PP);
  portpin_outclr(&LED_COM_PP);

  // init PPP
  /*
  pppintf.filter = ppp_filter;
  pppintf.uart = UART_PPP;
  pppintf.addr = PPP_ADDR;

  ppp_intf_init(&pppintf);
  room_set_message_handler(room_message_handler);
  // send a system RESET to signal that we have booted
  ppp_send_system_reset(&pppintf);
  */

  // timer
  timer_set_callback(timerF0, 'A', TIMER_US_TO_TICKS(F0,UPDATE_TICK_US), UPTIME_INTLVL, update);

  printf("**REBOOT**");
 
  // startup arm calibration
  arm_start_calibration();

  // main loop
  double t = 0.0;
  int idx = 0;
  for(;;) {
    arm_update();

    static int _ds = 0; _ds++;
    if((_ds%1000) == 0) {
      t+= 0.1;

      if(arm_is_running()) {
        idx++;
        uint8_t addr = idx/15;

        if(addr > 5)
          addr = 5;
        printf("addr = %d\n",addr);
        const int32_t consign[][3] = {
            {0, 0, 0},
            {-1780, 267, 60},
            {-5290, 197, 36},
            {-1780, 267, 60},
            {-5602, -310, -220},
            {-12146, -510, 127},
            };
        
        arm_set_position(A_UPPER, consign[addr][0]);
        arm_set_position(A_ELBOW, consign[addr][1]);
        arm_set_position(A_WRIST, consign[addr][2]);

        /*
        arm_activate_debug();
        arm_debug_t debug;
        arm_get_debug(&debug);
        printf("%ld %d %d\n",
          debug.upper,
          debug.elbow,
          debug.wrist);
        */


        // 0 0 0 
        // -1780 267 60
        // -5290 197 36
        // -1780 267 60
        // -5602 -310 -220
        // -12146 -510 127
      }


      portpin_outtgl(&LED_COM_PP);
    }

    //ppp_intf_update(&pppintf);
    portpin_outtgl(&LED_RUN_PP);
  }
}

