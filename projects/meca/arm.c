#include "arm.h"
#include "config.h"
#include <avarix.h>
#include <avarix/portpin.h>
#include <uart/uart.h>
#include <ax12/ax12.h>

#define MOTOR_SCALE (24.0/16.0)

#define UPPER_ARM_POSITION_OFFSET 12000
#define LOWER_ARM_SPEED 0x50
#define EXT_SERVO_SPEED 0x100

typedef enum {
  LA_LEFT = 0x11,
  LA_FRONT = 0x12,
  LA_RIGHT = 0x13,
}lower_arm_t;

extern ax12_t ax12;

typedef enum {
  ARM_STATE_INIT = 0,
  ARM_STATE_CALIBRATION_STARTING,
  ARM_STATE_CALIBRATION_STARTING_WAITING,
  ARM_STATE_CALIBRATION_IN_POSITION,
  ARM_STATE_RUNNING,

}arm_internal_state_t;

typedef struct {
  int32_t consign;
  uint8_t in_position;
}arm_consign_t;

// -- public functions --

/** @brief Initialize arm */
void arm_init() {

  // init AX-12
  portpin_dirset(&AX12_DIR_PP);
  portpin_outclr(&AX12_DIR_PP);
  portpin_dirset(&PORTPIN_TXDN(uart_get_usart(UART_AX12)));
}

void arm_set_external_servo(external_servo_t n, int16_t position) {

  ax12_addr_t addr;
  switch(n) {
    case S_LEFT:  addr = LA_LEFT; break;
    case S_FRONT: addr = LA_FRONT; break;
    case S_RIGHT: addr = LA_RIGHT; break;
    default: return;
  }

  ax12_write_byte(&ax12, addr, AX12_ADDR_TORQUE_ENABLE, 0x01);
  ax12_write_word(&ax12, addr, AX12_ADDR_GOAL_POSITION_L, 0x1FF + position);
  ax12_write_word(&ax12, addr, AX12_ADDR_MOVING_SPEED_L, EXT_SERVO_SPEED);
}
