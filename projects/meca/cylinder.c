#include <avr/io.h>
#include <avarix.h>
#include <avarix/portpin.h>
#include <ax12/ax12.h>
#include "config.h"
#include "servos.h"
#include <clock/clock.h>
#include <util/delay.h>
#include "cylinder.h"

extern ax12_t ax12;

cylinder_t cylinder;

#define RETRY_AND_UPDATE(fcall) while(!(fcall)) { \
  portpin_outtgl(&LED_AN_PP(0)); \
  _delay_ms(10); \
  }

// return ax12 load ranging from 0 to 1023, or -1 on error
static int16_t _get_ax12_load(ax12_addr_t address) {
  uint16_t load;
  uint8_t rv = ax12_read_word(&ax12, address, AX12_ADDR_PRESENT_LOAD_L, &load);

  if(rv == 0) {
    return -1;
  }

  // bit 10 is the load direction, remove it
  return load & 0x3ff;
}

static bool ax12_cylinder_set_posmode(void){
  uint8_t rv = 0;
  ax12_addr_t address = SE_AX12_CYLINDER_ID;
  uint16_t read_pos = 0;
  ax12_read_word(&ax12,
                  SE_AX12_CYLINDER_ID,
                  AX12_ADDR_PRESENT_POSITION_L,
                  &read_pos);
  rv |= ax12_write_word(&ax12, address, AX12_ADDR_CW_ANGLE_LIMIT_L,  0x000);
  rv |= ax12_write_word(&ax12, address, AX12_ADDR_CCW_ANGLE_LIMIT_L, 0x3FF);
  rv |= ax12_write_word(&ax12, address, AX12_ADDR_MOVING_SPEED_L, 0x0FF);
  rv |= ax12_write_word(&ax12, address, AX12_ADDR_GOAL_POSITION_L, read_pos);
  return rv == 0;
}

static bool ax12_cylinder_move(uint16_t position, uint16_t speed){
  uint8_t rv = 0;
  ax12_addr_t address = SE_AX12_CYLINDER_ID;
  rv |= ax12_write_word(&ax12, address, AX12_ADDR_MOVING_SPEED_L,  speed);
  rv |= ax12_write_byte(&ax12, address, AX12_ADDR_TORQUE_ENABLE,   0x01);
  rv |= ax12_write_word(&ax12, address, AX12_ADDR_GOAL_POSITION_L, position);

  return rv == 0;
}

static bool ax12_cylinder_rotate(uint16_t speed){
  uint8_t rv = 0;
  ax12_addr_t address = SE_AX12_CYLINDER_ID;
  rv |= ax12_write_word(&ax12, address, AX12_ADDR_CW_ANGLE_LIMIT_L,  0x000);
  rv |= ax12_write_word(&ax12, address, AX12_ADDR_CCW_ANGLE_LIMIT_L,  0x000);
  rv |= ax12_write_byte(&ax12, address, AX12_ADDR_TORQUE_ENABLE,   0x01);
  rv |= ax12_write_word(&ax12, address, AX12_ADDR_MOVING_SPEED_L, speed);

  return !(rv & AX12_ERROR_INVALID_PACKET);
}

static bool ax12_cylinder_in_position(uint16_t position){

  uint16_t read_pos = 0;
  ax12_read_word(&ax12,
                  SE_AX12_CYLINDER_ID,
                  AX12_ADDR_PRESENT_POSITION_L,
                  &read_pos);
  //int16_t read_load = _get_ax12_load(SE_AX12_CYLINDER_ID);
  (void) _get_ax12_load;

  int16_t diff = (int16_t)position - (int16_t) read_pos;
  if ( ((diff >=0) && (diff < 5)) ||  ((diff < 0) && (diff >- 5)) )
    return true;
  else
    return false;
}


static void turbine_off(void) { servo_set(1,1300); }
static void turbine_full(void){ servo_set(1,2500); }
//static void turbine_low(void){ servo_set(1,1600); }

static void balleater_off(void) { servo_set(0,2455); }
static void balleater_on(void)  { servo_set(0,2000); }
//static void balleater_reverse(void) { servo_set(0,2500); }

void cylinder_shutdown(void){
  turbine_off();
  balleater_off();
}

void cylinder_init(void){
  turbine_off();
  balleater_off();
  cylinder.state = CYLINDER_INIT;
}

void cylinder_update(void){
  #if 1
  turbine_full();

  (void) ax12_cylinder_set_posmode;
  (void) ax12_cylinder_move;
  (void) ax12_cylinder_rotate;
  (void) ax12_cylinder_in_position;

  switch (cylinder.state){
    case CYLINDER_INIT:
      ax12_cylinder_set_posmode();
      cylinder.balls_loaded = 0;
      //cylinder.state = CYLINDER_TAKEBALLS;
      break;

    case CYLINDER_TAKEBALLS:
      balleater_off();
      uint16_t cylpos = SE_CYLINDER_MIN+cylinder.balls_loaded*(SE_CYLINDER_MAX - SE_CYLINDER_MIN)/7;
      if (ax12_cylinder_move(cylpos, 0x12F)){
        if (ax12_cylinder_in_position(cylpos)){
          cylinder.balls_loaded++;
          balleater_on();
          portpin_outtgl(&LED_AN_PP(2));
          _delay_ms(1500);
        }
      }
      if (cylinder.balls_loaded > 7){
        balleater_off();
        cylinder.state = CYLINDER_IS_FULL;
      }
      break;

    case CYLINDER_IS_FULL:
      //no break;

    case CYLINDER_THROWBALLS:
      RETRY_AND_UPDATE(ax12_cylinder_rotate(0x15F));
      turbine_full();
      _delay_ms(10000);
      turbine_off();
      RETRY_AND_UPDATE(ax12_cylinder_rotate(0));
      cylinder.state = CYLINDER_INIT;
      break;

    case CYLINDER_TRASHBALLS:
      break;

    default:
      break;
  }

  #else

  RETRY_AND_UPDATE(servo_cylinder_set_posmode());
  portpin_outset(&LED_AN_PP(2));
  RETRY_AND_UPDATE(move_cylinder(SE_CYLINDER_MIN,0x17F));
  portpin_outtgl(&LED_AN_PP(3));
  RETRY_AND_UPDATE(cylinder_in_position(SE_CYLINDER_MIN));
  portpin_outtgl(&LED_AN_PP(2));

  //move cylinder to the other positions
  int i=0;
  while (i<= 7){
    servo_set(0,2455);
    uint16_t cylpos = SE_CYLINDER_MIN+i*(SE_CYLINDER_MAX - SE_CYLINDER_MIN)/7;
    if ( move_cylinder(cylpos, 0x12F)){
      if (cylinder_in_position(cylpos)){
        i++;
        portpin_outtgl(&LED_AN_PP(2));
        servo_set(0,2000);
        _delay_ms(1500);
      }
    }
  }
  portpin_outset(&LED_AN_PP(1));
  portpin_outclr(&LED_AN_PP(2));
  portpin_outclr(&LED_AN_PP(3));
  //turn off ball eater
  servo_set(0,2455);

  //make cylinder rotate continuously
  RETRY_AND_UPDATE(cylinder_rotate(0x15F));

  portpin_outtgl(&LED_AN_PP(1));

  //turn on turbine
  servo_set(1,2500);

  _delay_ms(10000);
  //stop turbine
  servo_set(1,1300);

  //stop cylinder
  RETRY_AND_UPDATE(cylinder_rotate(0));
  portpin_outtgl(&LED_AN_PP(2));
  _delay_ms(1000);
  RETRY_AND_UPDATE(servo_cylinder_set_posmode());
  _delay_ms(1000);

  portpin_outtgl(&LED_AN_PP(3));
  #endif
}
