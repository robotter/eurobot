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

// ROME interface
rome_intf_t rome;
// ROME messages handler
void rome_handler(rome_intf_t *intf, const rome_frame_t *frame) {
  switch(frame->mid) {
    
    case ROME_MID_MECA_SET_ARM: {
      int32_t upper = frame->meca_set_arm.upper;
      int32_t elbow = frame->meca_set_arm.elbow;
      int32_t wrist = frame->meca_set_arm.wrist;
      arm_set_position(A_UPPER, upper);
      arm_set_position(A_ELBOW, elbow);
      arm_set_position(A_WRIST, wrist);
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

int main(void)
{
  clock_init();

  uart_init();
  uart_fopen(UART_PPP);

  // intialize barometers
  barometer_t baro0, baro1;
  // ADCA4
  barometer_init(&baro0, &ADCA, ADC_CH_MUXPOS_PIN4_gc);
  // ADCA5
  barometer_init(&baro1, &ADCA, ADC_CH_MUXPOS_PIN5_gc);

  arm_init();

  portpin_dirset(&LED_RUN_PP);
  portpin_dirset(&LED_ERROR_PP);
  portpin_dirset(&LED_COM_PP);
  portpin_outclr(&LED_RUN_PP);
  portpin_outclr(&LED_ERROR_PP);
  portpin_outclr(&LED_COM_PP);

  printf("**REBOOT**\n");

  INTLVL_ENABLE_ALL();
  __asm__("sei");


  // timer
  timer_init();
  timer_set_callback(timerE0, 'A', TIMER_US_TO_TICKS(E0,UPDATE_TICK_US), UPTIME_INTLVL, update);

  // start arm calibration procedure
  arm_start_calibration();

  int32_t luptime = uptime;

    // main loop
  for(;;) {

    // update arm every 1 ms
    if(uptime - luptime > 1000) {
      luptime = uptime;
      arm_update();
    }
    if(arm_is_running()) {
      arm_set_position(A_UPPER, 4000);
      arm_set_position(A_ELBOW, -300);
    }

  }
}

