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
#include "acm.h"
#include "config.h"


static pwm_motor_t pwm_balloon;


/// current time in microseconds
static volatile uint32_t uptime;

/// Called on uptime timer tick
void uptime_update(void)
{
  uptime += UPTIME_TICK_US;
}

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


static ax12_t ax12 = {
  .send = ax12_send_char,
  .recv = ax12_recv_char,
  .set_state = ax12_set_state,
};


// arm motors
static pwm_motor_t arm_motors[4];

static void arm_motor0_pwm_set(int16_t val)
{
  if(val >= 0) {
    portpin_outclr(&MOTOR0_SIGN_PP);
    pwm_motor_set(arm_motors+0, val);
  } else {
    portpin_outset(&MOTOR0_SIGN_PP);
    pwm_motor_set(arm_motors+0, PWM_MOTOR_MAX + val);
  }
}

static void arm_motor1_pwm_set(int16_t val)
{
  if(val >= 0) {
    portpin_outclr(&MOTOR1_SIGN_PP);
    pwm_motor_set(arm_motors+1, val);
  } else {
    portpin_outset(&MOTOR1_SIGN_PP);
    pwm_motor_set(arm_motors+1, PWM_MOTOR_MAX + val);
  }
}

static void arm_motor2_pwm_set(int16_t val)
{
  if(val >= 0) {
    portpin_outclr(&MOTOR2_SIGN_PP);
    pwm_motor_set(arm_motors+2, val);
  } else {
    portpin_outset(&MOTOR2_SIGN_PP);
    pwm_motor_set(arm_motors+2, PWM_MOTOR_MAX + val);
  }
}

static void arm_motor3_pwm_set(int16_t val)
{
  if(val >= 0) {
    portpin_outclr(&MOTOR3_SIGN_PP);
    pwm_motor_set(arm_motors+3, val);
  } else {
    portpin_outset(&MOTOR3_SIGN_PP);
    pwm_motor_set(arm_motors+3, PWM_MOTOR_MAX + val);
  }
}


// caking
aeat_t cake_enc;
acm_t acm = {
  .ax12 = &ax12,
  .encoder = &cake_enc,
  .set_second_lvl_motor_pwm = arm_motor3_pwm_set,
  .set_first_lvl_left_motor_pwm = arm_motor0_pwm_set,
  .set_first_lvl_right_motor_pwm = arm_motor2_pwm_set,
};

static void cake_encoder_update(void)
{
  aeat_update(&cake_enc);
}


// Perlimpinpin

static ppp_intf_t pppintf;


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
    case ROOM_MID_MECA_BALLOON_TAP: {
      pwm_motor_set(&pwm_balloon, pl->meca_balloon_tap.open ? SERVO_BALLOON_OPEN_POS : SERVO_BALLOON_CLOSE_POS);
      ROOM_REPLY_MECA_BALLOON_TAP(intf, pl);
    } break;
    case ROOM_MID_MECA_SET_ARM_MODE: {
      acm_set_arm_mode(&acm, pl->meca_set_arm_mode.mode);
      ROOM_REPLY_MECA_SET_ARM_MODE(intf, pl);
    } break;
    case ROOM_MID_ROBOT_COLOR: {
      if(pl->robot_color.color == 1) {
        acm.robot_color = ACM_RED;
        acm.cake_stall_side = ACM_RED;
      }
      else if(pl->robot_color.color == 2) {
        acm.robot_color = ACM_BLUE;
        acm.cake_stall_side = ACM_BLUE;
      }
      ROOM_REPLY_ROBOT_COLOR(intf, pl);
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

  portpin_dirset(&LED_RUN_PP);
  portpin_dirset(&LED_ERROR_PP);
  portpin_dirset(&LED_COM_PP);
  portpin_outclr(&LED_RUN_PP);
  portpin_outclr(&LED_ERROR_PP);
  portpin_outclr(&LED_COM_PP);

  // init PPP
  pppintf.filter = ppp_filter;
  pppintf.uart = UART_PPP;
  pppintf.addr = PPP_ADDR;

  ppp_intf_init(&pppintf);
  room_set_message_handler(room_message_handler);
  // send a system RESET to signal that we have booted
  ppp_send_system_reset(&pppintf);

  // balloon servo, for the funny action
  pwm_servo_init(&pwm_balloon, &SERVO_BALLOON_TC, SERVO_BALLOON_TC_CH);
  pwm_motor_set(&pwm_balloon, SERVO_BALLOON_CLOSE_POS);

  // init AX-12
  portpin_dirset(&AX12_DIR_PP);
  portpin_outclr(&AX12_DIR_PP);
  portpin_dirset(&PORTPIN_TXDN(uart_get_usart(UART_AX12)));

  // init arm motors
  portpin_dirset(&MOTOR0_SIGN_PP);
  portpin_dirset(&MOTOR1_SIGN_PP);
  portpin_dirset(&MOTOR2_SIGN_PP);
  portpin_dirset(&MOTOR3_SIGN_PP);
  pwm_motor_init(arm_motors+0, (TC0_t*)&MOTOR0_TC, MOTOR0_TC_CH, 0);
  pwm_motor_init(arm_motors+2, (TC0_t*)&MOTOR1_TC, MOTOR1_TC_CH, 0);
  pwm_motor_init(arm_motors+1, (TC0_t*)&MOTOR2_TC, MOTOR2_TC_CH, 0);
  pwm_motor_init(arm_motors+3, (TC0_t*)&MOTOR3_TC, MOTOR3_TC_CH, 0);
  pwm_motor_set_frequency(arm_motors+0, 5000);
  pwm_motor_set_frequency(arm_motors+1, 5000);
  pwm_motor_set_frequency(arm_motors+2, 5000);
  pwm_motor_set_frequency(arm_motors+3, 5000);
  pwm_motor_set(arm_motors+0, 0);
  pwm_motor_set(arm_motors+1, 0);
  pwm_motor_set(arm_motors+2, 0);
  pwm_motor_set(arm_motors+3, 0);
  (void)arm_motor1_pwm_set; // not used

  // init cake encoder
  // dirset/outset MUST BE DONE before aeat init
  portpin_dirset(&CAKE_ENCODER_CS_PP);
  portpin_outset(&CAKE_ENCODER_CS_PP);
  aeat_spi_init();
  aeat_init(&cake_enc, CAKE_ENCODER_CS_PP);

  // init caking management
  acm_init(&acm);

  // timer
  timer_set_callback(timerF0, 'A', TIMER_US_TO_TICKS(F0,UPTIME_TICK_US), UPTIME_INTLVL, uptime_update);
  timer_set_callback(timerF0, 'B', TIMER_US_TO_TICKS(F0,CAKE_ENCODER_UPDATE_PERIOD_US), CAKE_ENCODER_INTLVL, cake_encoder_update);
  aeat_update(&cake_enc);

  // main loop
  for(;;) {
    ppp_intf_update(&pppintf);
    acm_update(&acm);
  }
}

