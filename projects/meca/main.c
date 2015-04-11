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
#include "arm.h"
#include "barometer.h"
#include "telemetry.h"
#include "spot_elevator.h"

// match timer
bool match_timer_counting = false;
int32_t match_timer_ms = -1;

// ROME interface
rome_intf_t rome_strat;
struct {
  rome_intf_t left,right;
}rome_color_sensor;

// robot color
robot_color_t robot_color = COLOR_GREEN; // TODO

// spot_elevator structure
spot_elevator_t l_spot_elevator = 
{
  .claw_ax12_addr = SE_LEFT_AX12_CLAW_ID,
  .elevator_ax12_addr = SE_LEFT_AX12_ELEVATOR_ID,
  .claw_ax12_positions = {SE_LEFT_CLAW_OPENED, SE_LEFT_CLAW_CLOSED_FOR_SPOT, SE_LEFT_CLAW_CLOSED_FOR_BULB},
  .elevator_ax12_positions = {SE_LEFT_ELEVATOR_UP, SE_LEFT_ELEVATOR_DOWN_WAIT_SPOT, SE_LEFT_ELEVATOR_DOWN_WAIT_BULB},
};
spot_elevator_t r_spot_elevator; // right spot elevator

// color sensor structure
struct {
  struct {
    robot_color_t color;
    bool detected;
  } left,right;
} color_sensor = {
  .left =  {.color = COLOR_UNDEFINED, .detected = false},
  .right = {.color = COLOR_UNDEFINED, .detected = false},
};

// ROME messages handler
void rome_color_sensor_handler(rome_intf_t *intf, const rome_frame_t *frame)
{
  (void)intf;
  switch(frame->mid) {
    case ROME_MID_COLOR_SENSOR_TM_DETECTION: {
      bool detected = frame->color_sensor_tm_detection.detected;
      uint8_t color = frame->color_sensor_tm_detection.color;

      if(intf == &rome_color_sensor.left) {
        // left sensor
        color_sensor.left.color = color;
        color_sensor.left.detected = detected;
      }
      else if(intf == &rome_color_sensor.right) {
        // right sensor
        color_sensor.right.color = color;
        color_sensor.right.detected = detected;
      }
      else {
        // XXX
        ROME_LOGF(&rome_strat, DEBUG, "Invalid interface %p", intf);
      }

    } break;

    default:
      break;
  }

  // forward
  rome_send(&rome_strat, frame);
}

void rome_strat_handler(rome_intf_t *intf, const rome_frame_t *frame)
{
  switch(frame->mid) {
    case ROME_MID_START_TIMER: {
      match_timer_counting = true;
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

    case ROME_MID_MECA_SET_SERVO: {
      arm_set_external_servo(frame->meca_set_servo.n, frame->meca_set_servo.position);
      rome_reply_ack(intf, frame);
    } break;

    case ROME_MID_MECA_PICK_ONE_SPOT: {
      switch(frame->meca_pick_one_spot.n)
      {
        case 0:
          spot_elevator_automatic_spot_stacking(&l_spot_elevator);
          break;
        case 1:
          spot_elevator_automatic_spot_stacking(&r_spot_elevator);
          break;
        default :
          break;
      }
      rome_reply_ack(intf, frame);
    } break;

    case ROME_MID_MECA_RELEASE_SPOT_STACK: {
      switch(frame->meca_release_spot_stack.n)
      {
        case 0:
          spot_elevator_discharge_spot_stack(&l_spot_elevator);
          break;
        case 1:
          spot_elevator_discharge_spot_stack(&r_spot_elevator);
          break;
        default :
          break;
      }
      rome_reply_ack(intf, frame);
    } break;

    case ROME_MID_MECA_PICK_BULB: {
      switch(frame->meca_pick_bulb.n)
      {
        case 0:
          spot_elevator_pick_bulb(&l_spot_elevator);
          break;
        case 1:
          spot_elevator_pick_bulb(&r_spot_elevator);
          break;
        default :
          break;
      }
      rome_reply_ack(intf, frame);
    } break;

    case ROME_MID_MECA_PREPARE_FOR_ONBOARD_BULB: {
      switch(frame->meca_prepare_for_onboard_bulb.n)
      {
        case 0:
          spot_elevator_prepare_for_onboard_bulb(&l_spot_elevator);
          break;
        case 1:
          spot_elevator_prepare_for_onboard_bulb(&r_spot_elevator);
          break;
        default :
          break;
      }
      rome_reply_ack(intf, frame);
    } break;

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

void update_match_timer(void)
{
  // update match timer
  // check match timer
  if(match_timer_ms > 1000*(int32_t)MATCH_DURATION_SECS) {
    // out of time
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

bool is_spot_present_l(void)
{
  return color_sensor.left.detected;
}

bool is_spot_present_r(void)
{
  return color_sensor.right.detected;
}

robot_color_t get_spot_color_l(void)
{
  return color_sensor.left.color; 
}

robot_color_t get_spot_color_r(void)
{
  return color_sensor.right.color; 
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
  portpin_dirset(&AX12_DIR_PP);

  // intialize barometers
  barometer_t baro0, baro1;
  // ADCA4
  barometer_init(&baro0, &ADCA, ADC_CH_MUXPOS_PIN4_gc);
  // ADCA5
  barometer_init(&baro1, &ADCA, ADC_CH_MUXPOS_PIN5_gc);

  // initialize spot elevator
  spot_elevator_init(&l_spot_elevator);
  spot_elevator_set_claw_ax12_addr(&l_spot_elevator, SE_LEFT_AX12_CLAW_ID);
  spot_elevator_set_elevator_ax12_addr(&l_spot_elevator, SE_LEFT_AX12_ELEVATOR_ID);
  spot_elevator_set_is_spot_present_fn(&l_spot_elevator, is_spot_present_l);
  spot_elevator_set_get_spot_color_fn(&l_spot_elevator, get_spot_color_l);

  spot_elevator_init(&r_spot_elevator);
  spot_elevator_set_claw_ax12_addr(&r_spot_elevator, SE_RIGHT_AX12_CLAW_ID);
  spot_elevator_set_elevator_ax12_addr(&r_spot_elevator, SE_RIGHT_AX12_ELEVATOR_ID);
  
  INTLVL_ENABLE_ALL();
  __asm__("sei");

  // timer
  timer_init();
  uptime_init();
  TIMER_SET_CALLBACK_US(E0, 'B', UPDATE_MATCH_TIMER_TICK_US, INTLVL_HI, update_match_timer);

  // Initialize ROME
  rome_intf_init(&rome_strat);
  rome_strat.uart = UART_PPP;
  rome_strat.handler = rome_strat_handler;
 
  rome_intf_init(&rome_color_sensor.right);
  #warning "DEFINE ME !"
  rome_color_sensor.left.uart = UART_PPP;
  rome_color_sensor.left.handler = rome_color_sensor_handler;

  rome_intf_init(&rome_color_sensor.left);
  #warning "DEFINE ME !"
  rome_color_sensor.right.uart = UART_PPP;
  rome_color_sensor.right.handler = rome_color_sensor_handler;

  uint32_t luptime = UINT32_MAX;
  uint32_t lluptime = UINT32_MAX;
  uint32_t spot_elevator_uptime = UINT32_MAX;

  arm_set_external_servo(S_LEFT, 120);
  arm_set_external_servo(S_RIGHT, -120);

  uint32_t t = 0;

  spot_elevator_set_enable(&l_spot_elevator, true);
  spot_elevator_automatic_spot_stacking(&l_spot_elevator);
  // main loop
  for(;;) {
    
    // debug info
    PORTA.OUT = (t++)/1000;
    uint32_t uptime = uptime_us();
    (void)lluptime;
    (void)luptime;

    // update rome every 100 ms
    uptime = uptime_us();
    if(uptime - lluptime > 100000) {
      lluptime = uptime;
      portpin_outset(&LED_COM_PP);
      rome_handle_input(&rome_strat);
      rome_handle_input(&rome_color_sensor.left);
      rome_handle_input(&rome_color_sensor.right);
      portpin_outclr(&LED_COM_PP);
    }
    
    // update spot elevator every 10 ms
    uptime = uptime_us();
    if(uptime - spot_elevator_uptime > 100000) {
      spot_elevator_uptime = uptime;
      spot_elevator_manage(&l_spot_elevator);
      //spot_elevator_manage(&r_spot_elevator);
    }
  }
}

