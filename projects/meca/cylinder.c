#include <avr/io.h>
#include <avarix.h>
#include <avarix/portpin.h>
#include <ax12/ax12.h>
#include "config.h"
#include "servos.h"
#include <clock/clock.h>
#include <util/delay.h>
#include "cylinder.h"
#include "jevois_cam.h"
#include <rome/rome.h>
#include <timer/uptime.h>

extern ax12_t ax12;

extern rome_intf_t rome_strat;

extern jevois_cam_t cam;

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
  rv |= ax12_write_word(&ax12, address, AX12_ADDR_CW_ANGLE_LIMIT_L,  0x000);
  rv |= ax12_write_word(&ax12, address, AX12_ADDR_CCW_ANGLE_LIMIT_L, 0x3FF);
  rv |= ax12_write_word(&ax12, address, AX12_ADDR_MOVING_SPEED_L, 0x0FF);
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

static uint16_t ax12_angle_from_position(uint8_t i) {
  return SE_CYLINDER_MIN+i*(SE_CYLINDER_MAX - SE_CYLINDER_MIN)/7;
}

static bool cylinder_in_position(uint8_t position) {
  return ax12_cylinder_in_position(ax12_angle_from_position(position));
}

static bool cylinder_goto_position(uint8_t position){
  cylinder.position = position % 8;
  return ax12_cylinder_move(ax12_angle_from_position(position), 0x3ff);
}

static void cylinder_increment(void) {
  // shift cw
  cylinder.position++;
  cylinder.position %= 8;
}

static bool cylinder_move(void) {
  // send command to ax12
  return cylinder_goto_position(cylinder.position);
}

static bool cylinder_move_done(void) {
  return cylinder_in_position(cylinder.position);
}

void cylinder_init(void){
  turbine_off();
  balleater_off();
  cylinder.state = CYLINDER_INIT;
  cylinder.balls_loaded = 0;
}

void cylinder_update(void){
  (void)ax12_cylinder_set_posmode;
  (void)ax12_cylinder_move;
  (void)ax12_cylinder_in_position;
  (void)ax12_cylinder_rotate;


  #if 1

  switch (cylinder.state){
    case CYLINDER_INIT:
      ax12_cylinder_set_posmode();
      cylinder_goto_position(0);
      cylinder.state = CYLINDER_BALLEATER_PRE_TAKE;
      cylinder.moving_ts = 0;
      cylinder.drop_ts = 0;
      break;
/*
    case CYLINDER_CHECK_BALLS:
      uint16_t cylpos = SE_CYLINDER_MIN+cylinder.balls_loaded*(SE_CYLINDER_MAX - SE_CYLINDER_MIN)/7;
      if (ax12_cylinder_move(cylpos, 0x12F)){
        if (ax12_cylinder_in_position(cylpos)){
          if (jevois_cam_get_cylinder_color(&cam) != JEVOIS_COLOR_NONE){
          cylinder.state = CYLINDER_EATBALL;
        }
      }
*/
    case CYLINDER_BALLEATER_PRE_TAKE:
      balleater_on();
      if (jevois_cam_is_valid(&cam) && jevois_cam_get_entry_color(&cam) != JEVOIS_COLOR_NONE){
        portpin_outset(&LED_AN_PP(2));
        balleater_off();
        cylinder.state = CYLINDER_FIND_EMPTY;
      }
      break;

    case CYLINDER_FIND_EMPTY: {
      bool cylinder_empty =
        jevois_cam_is_valid(&cam) && jevois_cam_get_cylinder_color(&cam) == JEVOIS_COLOR_NONE;


      bool delay = cylinder.moving_ts + 200000 < uptime_us();
      if(!delay)
        break;
      // cylinder position empty lets roll
      if(cylinder_empty) {
        cylinder.drop_ts = uptime_us();
        cylinder.state = CYLINDER_BALLEATER_TAKE;
        break;
      }

      cylinder_increment();
      cylinder.state = CYLINDER_FIND_EMPTY_ORDER_MOVING;
      // no break
    }

    case CYLINDER_FIND_EMPTY_ORDER_MOVING: {
      // shift to next position
      if(!cylinder_move())
        break;
      cylinder.state = CYLINDER_FIND_EMPTY_MOVING;
      break;
    }

    case CYLINDER_FIND_EMPTY_MOVING: {
      if(cylinder_move_done()) {
        cylinder.state = CYLINDER_FIND_EMPTY;
        cylinder.moving_ts = uptime_us();
      }
      break;
    }

    case CYLINDER_BALLEATER_TAKE: {
      balleater_on();


      bool entry_empty =
        jevois_cam_is_valid(&cam) && jevois_cam_get_entry_height(&cam) < 150;
      bool cylinder_present =
        jevois_cam_is_valid(&cam) && jevois_cam_get_cylinder_color(&cam) != JEVOIS_COLOR_NONE;
      if(entry_empty && cylinder_present) {
        balleater_off();
        cylinder.balls_loaded ++;
        if (cylinder.balls_loaded > 6){
          cylinder.state = CYLINDER_THROWBALLS;
          break;
        }
        cylinder.state = CYLINDER_BALLEATER_POST_TAKE;
        break;
      }
      break;
    }

    case CYLINDER_BALLEATER_POST_TAKE:{
      //wait for ball to drop
      bool delay = cylinder.drop_ts + 500000 < uptime_us();
      if(!delay)
        break;

      cylinder.state = CYLINDER_BALLEATER_PRE_TAKE;
      break;
    }
/*
    case CYLINDER_TAKEBALLS:
      if (ax12_cylinder_move(cylpos, 0x12F)){
        if (ax12_cylinder_in_position(cylpos)){
          if (jevois_cam_is_valid() && jevois_cam_get_cylinder_color != JEVOIs_COLOR_NONE){
            balleater_on();
        }
      }


      balleater_off();
      uint16_t cylpos = SE_CYLINDER_MIN+cylinder.balls_loaded*(SE_CYLINDER_MAX - SE_CYLINDER_MIN)/7;
      if (ax12_cylinder_move(cylpos, 0x12F)){
        if (ax12_cylinder_in_position(cylpos)){
          balleater_on();
          portpin_outtgl(&LED_AN_PP(2));
          if (jevois_cam_is_valid()&&
            cylinder.balls_loaded++;
        }
      }
      if (cylinder.balls_loaded > 7){
        balleater_off();
        cylinder.state = CYLINDER_IS_FULL;
      }
      break;

    case CYLINDER_IS_FULL:
      //no break;
*/
    case CYLINDER_THROWBALLS:
      cylinder_goto_position(5);
      if (!cylinder_move_done())
        break;
      turbine_full();
      _delay_ms(1000);
      RETRY_AND_UPDATE(ax12_cylinder_rotate(0x15F));
      _delay_ms(8000);
      turbine_off();
      RETRY_AND_UPDATE(ax12_cylinder_rotate(0));
      cylinder.state = CYLINDER_INIT;
      cylinder.balls_loaded = 0;
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
