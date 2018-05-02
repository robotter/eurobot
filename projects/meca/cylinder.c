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
#include <timer/uptime.h>

extern ax12_t ax12;

extern rome_intf_t rome_strat;

extern jevois_cam_t cam;

cylinder_t cylinder;
const uint16_t balleater_pos[CYLINDER_NB_POS] = CYLINDER_BALLEATER_POS;
const uint16_t turbine_pos[CYLINDER_NB_POS]   = CYLINDER_TURBINE_POS;


#define RETRY_AND_UPDATE(fcall) while(!(fcall)) { \
  portpin_outtgl(&LED_AN_PP(0)); \
  _delay_ms(10); \
  }

#ifdef USE_AX12
// return ax12 load ranging from 0 to 1023, or -1 on error
int16_t _get_ax12_load(uint8_t address) {
  uint16_t load;
  uint8_t rv = ax12_read_word(&ax12, address, AX12_ADDR_PRESENT_LOAD_L, &load);

  if(rv == 0) {
    return -1;
  }

  // bit 10 is the load direction, remove it
  return load & 0x3ff;
}

bool ax12_cylinder_set_posmode(void){
  uint8_t rv = 0;
  uint8_t address = CYLINDER_AX12_ID;
  rv |= ax12_write_word(&ax12, address, AX12_ADDR_CW_ANGLE_LIMIT_L,  0x000);
  rv |= ax12_write_word(&ax12, address, AX12_ADDR_CCW_ANGLE_LIMIT_L, 0x3FF);
  rv |= ax12_write_word(&ax12, address, AX12_ADDR_MOVING_SPEED_L, 0x0FF);

  return !(rv & AX12_ERROR_INVALID_PACKET);
}

bool ax12_cylinder_move(uint16_t position, uint16_t speed){
  uint8_t rv = 0;
  uint8_t address = CYLINDER_AX12_ID;
  rv |= ax12_write_word(&ax12, address, AX12_ADDR_MOVING_SPEED_L,  speed);
  rv |= ax12_write_byte(&ax12, address, AX12_ADDR_TORQUE_ENABLE,   0x01);
  rv |= ax12_write_word(&ax12, address, AX12_ADDR_GOAL_POSITION_L, position);

  return !(rv & AX12_ERROR_INVALID_PACKET);
}

bool ax12_cylinder_rotate(uint16_t speed){
  uint8_t rv = 0;
  uint8_t address = CYLINDER_AX12_ID;
  rv |= ax12_write_word(&ax12, address, AX12_ADDR_CW_ANGLE_LIMIT_L,  0x000);
  rv |= ax12_write_word(&ax12, address, AX12_ADDR_CCW_ANGLE_LIMIT_L,  0x000);
  rv |= ax12_write_byte(&ax12, address, AX12_ADDR_TORQUE_ENABLE,   0x01);
  rv |= ax12_write_word(&ax12, address, AX12_ADDR_MOVING_SPEED_L, speed);

  return !(rv & AX12_ERROR_INVALID_PACKET);
}

bool ax12_cylinder_in_position(uint16_t position){

  uint16_t read_pos = 0;
  ax12_read_word(&ax12,
                  CYLINDER_AX12_ID,
                  AX12_ADDR_PRESENT_POSITION_L,
                  &read_pos);
  //int16_t read_load = _get_ax12_load(CYLINDER_AX12_ID);

  int16_t diff = (int16_t)position - (int16_t) read_pos;
  if ( ((diff >=0) && (diff < CYLINDER_AX12_ARRIVED_WINDOW)) ||  ((diff < 0) && (diff >- CYLINDER_AX12_ARRIVED_WINDOW)) )
    return true;
  else
    return false;
}

#endif

#ifdef USE_MX12
// return mx12 load ranging from 0 to 1023, or -1 on error
int16_t _get_mx12_load(uint8_t address) {
  uint16_t load;
  uint8_t rv = ax12_read_word(&ax12, address, MX12_ADDR_PRESENT_LOAD_L, &load);

  if(rv == 0) {
    return -1;
  }

  // bit 10 is the load direction, remove it
  return load & 0x3ff;
}

bool mx12_cylinder_set_posmode(void){
  uint8_t rv = 0;
  uint8_t address = CYLINDER_MX12_ID;
  rv |= ax12_write_word(&ax12, address, MX12_ADDR_CW_ANGLE_LIMIT_L,  0x000);
  rv |= ax12_write_word(&ax12, address, MX12_ADDR_CCW_ANGLE_LIMIT_L, 0x3FF);
  rv |= ax12_write_word(&ax12, address, MX12_ADDR_MOVING_SPEED_L, 0x0FF);

  return !(rv & AX12_ERROR_INVALID_PACKET);
}

bool mx12_cylinder_rotate(uint16_t speed){
  uint8_t rv = 0;
  uint8_t address = CYLINDER_MX12_ID;
  rv |= ax12_write_word(&ax12, address, MX12_ADDR_CW_ANGLE_LIMIT_L,  0x000);
  rv |= ax12_write_word(&ax12, address, MX12_ADDR_CCW_ANGLE_LIMIT_L,  0x000);
  rv |= ax12_write_byte(&ax12, address, MX12_ADDR_TORQUE_ENABLE,   0x01);
  rv |= ax12_write_word(&ax12, address, MX12_ADDR_MOVING_SPEED_L, speed);

  return !(rv & AX12_ERROR_INVALID_PACKET);
}

bool mx12_cylinder_move(uint16_t position, uint16_t speed){
  uint8_t rv = 0;
  uint8_t address = CYLINDER_MX12_ID;
  rv |= ax12_write_word(&ax12, address, MX12_ADDR_MOVING_SPEED_L,  speed);
  rv |= ax12_write_byte(&ax12, address, MX12_ADDR_TORQUE_ENABLE,   0x01);
  rv |= ax12_write_word(&ax12, address, MX12_ADDR_GOAL_POSITION_L, position);

  return !(rv & AX12_ERROR_INVALID_PACKET);
}

bool mx12_cylinder_in_position(uint16_t position){

  uint16_t read_pos = 0;
  ax12_read_word(&ax12,
                  CYLINDER_MX12_ID,
                  MX12_ADDR_PRESENT_POSITION_L,
                  &read_pos);
  //int16_t read_load = _get_mx12_load(CYLINDER_MX12_ID);
  (void) _get_mx12_load;

  int16_t diff = (int16_t)position - (int16_t) read_pos;
  if ( ((diff >=0) && (diff < 5)) ||  ((diff < 0) && (diff >- 5)) )
    return true;
  else
    return false;
}
#endif
void deflector_on(void) { servo_set(2,1800); }
void deflector_off(void) { servo_set(2,3000); }

void turbine_off(void) { deflector_on(); servo_set(1,1300); }
void turbine_full(void){ deflector_off(); servo_set(1,2500); }
void turbine_watertower(void){ deflector_off(); servo_set(1,cylinder.throw_power); }
void turbine_treatment(void){  deflector_on();  servo_set(1,cylinder.trash_power); }
void turbine_low(void){ deflector_off(); servo_set(1,1650); }

void balleater_off(void) { servo_set(0,2455); }
void balleater_on(void)  { servo_set(0,2000); }
void balleater_reverse(void) { servo_set(0,2500); }



void cylinder_shutdown(void){
  turbine_off();
  balleater_off();
#ifdef USE_AX12
  ax12_write_byte(&ax12, CYLINDER_AX12_ID, AX12_ADDR_TORQUE_ENABLE,   0x01);
#endif
#ifdef USE_MX12
  ax12_write_byte(&ax12, CYLINDER_MX12_ID, MX12_ADDR_TORQUE_ENABLE,   0x01);
#endif
}

bool cylinder_goto_position(uint16_t position){
#ifdef USE_AX12
  return ax12_cylinder_move(position, 0x3ff);
#endif
#ifdef USE_MX12
  return mx12_cylinder_move(position, 0x3ff);
#endif
}

void cylinder_position_increment(void) {
  // shift cw
  cylinder.position++;
  cylinder.position %= CYLINDER_NB_POS;
}

bool cylinder_balleater_move(void) {
  // send command to servo
  return cylinder_goto_position(balleater_pos[cylinder.position]);
}

bool cylinder_balleater_move_done(void) {
#ifdef USE_AX12
  return ax12_cylinder_in_position(balleater_pos[cylinder.position]);
#endif
#ifdef USE_MX12
  return mx12_cylinder_in_position(balleater_pos[cylinder.position]);
#endif
}

bool cylinder_turbine_move(void) {
  // send command to servo
  return cylinder_goto_position(turbine_pos[cylinder.position]);
}

bool cylinder_turbine_move_done(void) {
#ifdef USE_AX12
  return ax12_cylinder_in_position(turbine_pos[cylinder.position]);
#endif
#ifdef USE_MX12
  return mx12_cylinder_in_position(turbine_pos[cylinder.position]);
#endif
}

void set_current_slot_color(rome_enum_jevois_color_t jvc){
  cylinder.ball_color[cylinder.position] = jvc;
}

rome_enum_jevois_color_t get_current_slot_color(void){
  return cylinder.ball_color[cylinder.position];
}

int8_t cylinder_get_next_color_slot(rome_enum_jevois_color_t jvc){
  if (jvc != cylinder.robot_color){
    for( int8_t i = 0; i < CYLINDER_NB_POS; i++){
      if (cylinder.ball_color[i] == ROME_ENUM_JEVOIS_COLOR_NONE)
        return i;
    }
  }
  else{
    for( int8_t i = CYLINDER_NB_POS -1; i > 0; i--){
      if (cylinder.ball_color[i] == ROME_ENUM_JEVOIS_COLOR_NONE)
        return i;
    }
  }
  return 0;
}

//static rome_enum_jevois_color_t get_current_slot_color(void){
//  return cylinder.ball_color[cylinder.position];
//}

bool cylinder_is_full(void){
  return (cylinder_count_empty_slots() == 0);
}

void cylinder_reset_next_move_ball_colors(void){
  for (uint8_t i = 0; i <= cylinder.next_move.length; i++)
    cylinder.ball_color[(cylinder.next_move.begin+i)%CYLINDER_NB_POS] = ROME_ENUM_JEVOIS_COLOR_NONE;
}

void cylinder_reset_ball_colors(void){
  for (uint8_t i = 0; i < CYLINDER_NB_POS; i++)
    cylinder.ball_color[i] = ROME_ENUM_JEVOIS_COLOR_NONE;
}

rome_enum_jevois_color_t _opposite_color(rome_enum_jevois_color_t color){
  if (color == ROME_ENUM_JEVOIS_COLOR_ORANGE)
    return ROME_ENUM_JEVOIS_COLOR_GREEN;

  if (color == ROME_ENUM_JEVOIS_COLOR_GREEN)
    return ROME_ENUM_JEVOIS_COLOR_ORANGE;

  return ROME_ENUM_JEVOIS_COLOR_NONE;
}

cylinder_move_t next_emptying_move(rome_enum_jevois_color_t color){
  uint8_t idx = 0;
  uint8_t i   = 0;
  uint8_t j   = 0;
  uint8_t dist = 0;
  cylinder_move_t max_move;
  max_move.begin = 0;
  max_move.end = 0;
  max_move.length = 0;
  while (idx < CYLINDER_NB_POS){
    //search for the beginning of a color row
    if (cylinder.ball_color[idx] == color){
      //search for the end of a color row
      i = idx;
      j = idx;
      while( cylinder.ball_color[i] != _opposite_color(color)){
        if (cylinder.ball_color[i] == color)
          j = i;
        i++;
        i %= CYLINDER_NB_POS;
        if (i == idx)
          break;
      }
      //measure color row length, and store the longest
      if (j != 2){
        if (j >= idx)
          dist = j - idx + 1;
        else
          dist = j+CYLINDER_NB_POS - idx + 1;
        if (dist > max_move.length){
          max_move.length = dist;
          max_move.begin = idx;
          max_move.end = j;
        }
      }
    }

    idx ++;
    if (idx == 2)
      idx ++;
  }

  ROME_LOGF(&rome_strat, DEBUG,"meca next (%u) move : %u from %u to %u", color, max_move.length, max_move.begin, max_move.end);

  return max_move;
}

void cylinder_set_idle(void){
  cylinder.state = CYLINDER_IDLE;
  cylinder.tm_optimal_move = ROME_ENUM_EMPTYING_MOVE_NONE;
}

void cylinder_init(void){
  turbine_off();
  balleater_off();
  deflector_on();
  cylinder.state = CYLINDER_INIT;
  cylinder.robot_color = ROME_ENUM_JEVOIS_COLOR_GREEN;
  cylinder.moving_ts = uptime_us();
  ax12_cylinder_set_posmode();
  cylinder_reset_ball_colors();
  cylinder.tm_state = ROME_ENUM_MECA_STATE_BUSY;
  cylinder.tm_optimal_move = ROME_ENUM_EMPTYING_MOVE_NONE;
  //set default turbine power
  cylinder.throw_power = 2100;
  cylinder.trash_power = 1800;
}

void cylinder_update(void){
  static cylinder_state_t os = 1;
  static uint32_t lsct = 0;
  if (os != cylinder.state){
    ROME_LOGF(&rome_strat, DEBUG,"meca state : %u",cylinder.state);
    ROME_LOGF(&rome_strat, DEBUG,"meca position : %u",cylinder.position);
    ROME_LOGF(&rome_strat, DEBUG,"meca balls : %u %u %u %u %u %u %u %u",
      cylinder.ball_color[0],
      cylinder.ball_color[1],
      cylinder.ball_color[2],
      cylinder.ball_color[3],
      cylinder.ball_color[4],
      cylinder.ball_color[5],
      cylinder.ball_color[6],
      cylinder.ball_color[7]);
    //if state changed, the meca is doing something and isn't blocked
    //idle and init stages are the only exeptions
    if (cylinder.state != CYLINDER_IDLE && cylinder.state != CYLINDER_INIT)
      lsct = uptime_us();
  }
  os = cylinder.state;

  if(uptime_us() - lsct > MECA_TIMEOUT_US)
    cylinder_set_idle();

  switch (cylinder.state){
    case CYLINDER_INIT:{
      if (!jevois_cam_is_valid(&cam))
        break;
      bool delay = cylinder.moving_ts + CYLINDER_TURBINE_BOOT_DELAY_US < uptime_us();
      if(!delay)
        break;
      turbine_off();
      balleater_off();
      cylinder.moving_ts = 0;
      cylinder.drop_ts = 0;
      cylinder_set_idle();
      break;
    }

    case CYLINDER_CHECK_EMPTY_PREPARE:
      cylinder.position = 0;
      cylinder_reset_ball_colors();
      cylinder.state = CYLINDER_CHECK_EMPTY_ORDER_MOVING;
      break;

    case CYLINDER_CHECK_EMPTY:{
      bool delay = cylinder.moving_ts + CYLINDER_MOVING_DELAY_US < uptime_us();
      if(!delay)
        break;
      if (!jevois_cam_is_valid(&cam))
        break;
      set_current_slot_color(jevois_cam_get_cylinder_color(&cam));

      if (cylinder.position == CYLINDER_NB_POS - 1 ){
        cylinder_set_idle();
        break;
      }

      // shift to next position
      cylinder_position_increment();
      cylinder.state = CYLINDER_CHECK_EMPTY_ORDER_MOVING;
      break;
    }

    case CYLINDER_CHECK_EMPTY_ORDER_MOVING:
      if(!cylinder_balleater_move())
        break;
      cylinder.state = CYLINDER_CHECK_EMPTY_MOVING;
      break;

    case CYLINDER_CHECK_EMPTY_MOVING:
      if(cylinder_balleater_move_done()) {
        cylinder.state = CYLINDER_CHECK_EMPTY;
        cylinder.moving_ts = uptime_us();
      }
      break;

    case CYLINDER_BALLEATER_PRE_TAKE:
      balleater_on();
      cylinder.eating_ts = uptime_us();
      cylinder.state = CYLINDER_BALLEATER_WAIT_EATING;
      //no break;

    case CYLINDER_BALLEATER_WAIT_EATING:{
      bool delay = cylinder.eating_ts + CYLINDER_EATING_TIMEOUT_US > uptime_us();
      if(!delay){
        cylinder_set_idle();
        balleater_off();
        break;
      }
      //wait for a valid camera frame
      if (!jevois_cam_is_valid(&cam))
        break;
      if (jevois_cam_get_entry_color(&cam) != ROME_ENUM_JEVOIS_COLOR_NONE){
        cylinder.position = cylinder_get_next_color_slot(jevois_cam_get_entry_color(&cam));
        cylinder.state = CYLINDER_EATING_FIND_EMPTY_ORDER_MOVING;
        balleater_off();
      }
      break;
    }

    case CYLINDER_EATING_FIND_EMPTY: {
      //wait for a valid camera frame
      if (!jevois_cam_is_valid(&cam))
        break;

      //if there is no ball in the eater, go taking one
      if (jevois_cam_get_entry_color(&cam) == ROME_ENUM_JEVOIS_COLOR_NONE)
        cylinder.state = CYLINDER_BALLEATER_PRE_TAKE;

      bool delay = cylinder.moving_ts + CYLINDER_MOVING_DELAY_US < uptime_us();
      if(!delay)
        break;

      bool cylinder_empty = jevois_cam_get_cylinder_color(&cam) == ROME_ENUM_JEVOIS_COLOR_NONE;

      // cylinder position empty lets roll
      if(cylinder_empty) {
        set_current_slot_color(ROME_ENUM_JEVOIS_COLOR_NONE);
        cylinder.drop_ts = uptime_us();
        cylinder.state = CYLINDER_BALLEATER_TAKE;
        break;
      }
      //cylinder not empty :(
      //update current slot color in case the stored information was false ...
      set_current_slot_color(jevois_cam_get_cylinder_color(&cam));
      //... and check if there still are empty spots
      if (cylinder_is_full()){
        cylinder_set_idle();
        break;
      }

      // shift to next empty spot
      cylinder.position = cylinder_get_next_color_slot(jevois_cam_get_entry_color(&cam));
      cylinder.state = CYLINDER_EATING_FIND_EMPTY_ORDER_MOVING;
      // no break
    }

    case CYLINDER_TAKEBALLS:
      //first ball in dispenser should be a good one
      cylinder.position = cylinder_get_next_color_slot(cylinder.robot_color);
      cylinder.state = CYLINDER_EATING_FIND_EMPTY_ORDER_MOVING;
      //no break

    case CYLINDER_EATING_FIND_EMPTY_ORDER_MOVING:
      if(!cylinder_balleater_move())
        break;
      cylinder.state = CYLINDER_EATING_FIND_EMPTY_MOVING;
      break;

    case CYLINDER_EATING_FIND_EMPTY_MOVING:
      if(cylinder_balleater_move_done()) {
        cylinder.moving_ts = uptime_us();
        if (cylinder.exec)
          cylinder.state = CYLINDER_EATING_FIND_EMPTY;
        else
          cylinder_set_idle();
      }
      break;

    case CYLINDER_BALLEATER_TAKE: {
      balleater_on();

      //wait for a valid camera frame
      if (!jevois_cam_is_valid(&cam))
        break;

      bool entry_empty = jevois_cam_get_entry_height(&cam) < 150;
      bool cylinder_present = jevois_cam_get_cylinder_color(&cam) != ROME_ENUM_JEVOIS_COLOR_NONE;

      if(entry_empty && cylinder_present) {
        balleater_off();
        set_current_slot_color(jevois_cam_get_cylinder_color(&cam));
        cylinder.state = CYLINDER_BALLEATER_POST_TAKE;
        break;
      }

      break;
    }

    case CYLINDER_BALLEATER_POST_TAKE:{
      //wait for ball to drop
      bool delay = cylinder.drop_ts + CYLINDER_BALL_DROP_DELAY_US < uptime_us();
      if(!delay)
        break;

      //if all balls are loaded stop taking balls
      if (cylinder_is_full()){
        cylinder_set_idle();
        break;
      }
      cylinder.state = CYLINDER_BALLEATER_PRE_TAKE;
      break;
    }

    case CYLINDER_THROWBALLS:
    case CYLINDER_THROWBALLS_PREPARE_ORDER_MOVE:
      cylinder.position = cylinder.next_move.begin;
      if (!cylinder_turbine_move())
        break;
      cylinder.state = CYLINDER_THROWBALLS_PREPARE_MOVING;
      //no break
    case CYLINDER_THROWBALLS_PREPARE_MOVING:
      if (!cylinder_turbine_move_done())
        break;
      if (!cylinder.exec){
        cylinder_set_idle();
        break;
      }
      //switch on turbine depending on the throw mode
      switch (cylinder.throw_mode){
        case CYLINDER_THROW_WATERTOWER:
          turbine_watertower();
          break;
        case CYLINDER_THROW_TREATMENT:
          turbine_treatment();
          break;
        case CYLINDER_THROW_OFFCUP:
          turbine_full();
          break;
        default:
          turbine_treatment();
          break;
      }
      //prepare the end move position of the cylinder
      cylinder.position = cylinder.next_move.end;
      cylinder.state = CYLINDER_THROWBALLS_ORDER_TURN;
      cylinder.throw_ts = uptime_us();
      //no break

    case CYLINDER_THROWBALLS_ORDER_TURN:{
      bool delay = cylinder.throw_ts + CYLINDER_TURBINE_STOP_DELAY_US < uptime_us();
      if (!delay)
        break;
      if(!ax12_cylinder_rotate(CYLINDER_WHEELMODE_SPEED))
        break;
      cylinder.state = CYLINDER_THROWBALLS_TURNING;
      break;
    }

    case CYLINDER_THROWBALLS_TURNING:
      if(!cylinder_turbine_move_done())
        break;
      if(!ax12_cylinder_rotate(0))
        break;
      turbine_off();
      cylinder.state = CYLINDER_THROWBALLS_STOPPING;
      cylinder.throw_ts = uptime_us();
      //no break

    case CYLINDER_THROWBALLS_STOPPING:{
      bool delay = cylinder.throw_ts + CYLINDER_TURBINE_STOP_DELAY_US < uptime_us();
      if (!delay)
        break;
      ax12_cylinder_set_posmode();
      if (cylinder.throw_mode == CYLINDER_THROW_WATERTOWER
        ||cylinder.throw_mode == CYLINDER_THROW_TREATMENT)
        cylinder_reset_next_move_ball_colors();
      else
        cylinder_reset_ball_colors();
      cylinder_set_idle();
      break;
    }

    case CYLINDER_IDLE:
      cylinder.tm_state = ROME_ENUM_MECA_STATE_READY;

      //compute next move options
      if (cylinder.tm_optimal_move == ROME_ENUM_EMPTYING_MOVE_NONE){
        cylinder_move_t watertower_move = next_emptying_move(cylinder.robot_color);
        cylinder_move_t treatment_move =  next_emptying_move(_opposite_color(cylinder.robot_color));
        if (watertower_move.length > treatment_move.length)
          cylinder.tm_optimal_move = ROME_ENUM_EMPTYING_MOVE_WATERTOWER;
        else
          cylinder.tm_optimal_move = ROME_ENUM_EMPTYING_MOVE_TREATMENT;
      }
      break;

    default:
      ROME_LOGF(&rome_strat, DEBUG,"meca default case : %d",cylinder.state);
      break;
  }
}

void cylinder_check_empty(void){
  if (cylinder.state == CYLINDER_IDLE){
    cylinder.state = CYLINDER_CHECK_EMPTY_PREPARE;
    cylinder.tm_state = ROME_ENUM_MECA_STATE_GROUND_CLEAR;
  }
}

void cylinder_load_water(bool exec){
  cylinder.exec = exec;
  if (cylinder.state == CYLINDER_IDLE){
    cylinder.state = CYLINDER_TAKEBALLS;
    if (exec)
      cylinder.tm_state = ROME_ENUM_MECA_STATE_BUSY;
    else
      cylinder.tm_state = ROME_ENUM_MECA_STATE_GROUND_CLEAR;
  }
}

void cylinder_throw_watertower(bool exec){
  cylinder.exec = exec;
  if (cylinder.state == CYLINDER_IDLE){
    cylinder.next_move = next_emptying_move(cylinder.robot_color);
    if (cylinder.next_move.length != 0){
      cylinder.state = CYLINDER_THROWBALLS;
      cylinder.throw_mode = CYLINDER_THROW_WATERTOWER;
      if (exec)
        cylinder.tm_state = ROME_ENUM_MECA_STATE_BUSY;
      else
        cylinder.tm_state = ROME_ENUM_MECA_STATE_GROUND_CLEAR;
    }
  }
}

void cylinder_trash_treatment(bool exec){
  cylinder.exec = exec;
  if (cylinder.state == CYLINDER_IDLE){
    cylinder.next_move = next_emptying_move(_opposite_color(cylinder.robot_color));
    if (cylinder.next_move.length != 0){
      cylinder.state = CYLINDER_THROWBALLS;
      cylinder.throw_mode = CYLINDER_THROW_TREATMENT;
      if (exec)
        cylinder.tm_state = ROME_ENUM_MECA_STATE_BUSY;
      else
        cylinder.tm_state = ROME_ENUM_MECA_STATE_GROUND_CLEAR;
    }
  }
}

void cylinder_trash_beginmatch(void){
  cylinder.exec = true;
  if (cylinder.state == CYLINDER_IDLE){
    cylinder.state = CYLINDER_THROWBALLS;
    cylinder.next_move.begin = 2;
    cylinder.next_move.end = CYLINDER_NB_POS - 1;
    cylinder.throw_mode = CYLINDER_THROW_NONE;
    cylinder.tm_state = ROME_ENUM_MECA_STATE_BUSY;
  }

}

void cylinder_throw_offcup(void){
  cylinder.exec = true;
  if (cylinder.state == CYLINDER_IDLE){
    cylinder.next_move.begin = 2;
    cylinder.next_move.end = CYLINDER_NB_POS - 1;
    cylinder.state = CYLINDER_THROWBALLS;
    cylinder.throw_mode = CYLINDER_THROW_OFFCUP;
    cylinder.tm_state = ROME_ENUM_MECA_STATE_GROUND_CLEAR;
  }
}

uint8_t cylinder_count_empty_slots(void){
  uint8_t balls_loaded = 0;
  for (uint8_t i = 0; i < CYLINDER_NB_POS; i++){
    if (cylinder.ball_color[i] == ROME_ENUM_JEVOIS_COLOR_NONE)
      balls_loaded ++;
  }
  return balls_loaded;
}

uint8_t cylinder_count_good_water(void){
  uint8_t balls_loaded = 0;
  for (uint8_t i = 0; i < CYLINDER_NB_POS; i++){
    if (cylinder.ball_color[i] == cylinder.robot_color)
      balls_loaded ++;
  }
  return balls_loaded;
}

uint8_t cylinder_count_bad_water(void){
  uint8_t balls_loaded = 0;
  for (uint8_t i = 0; i < CYLINDER_NB_POS; i++){
    if (cylinder.ball_color[i] == _opposite_color(cylinder.robot_color))
      balls_loaded ++;
  }
  return balls_loaded;
}

uint8_t cylinder_get_tm_state(void){
  return cylinder.tm_state;
}

uint8_t cylinder_get_tm_optimal_move(void){
  return cylinder.tm_optimal_move;
}

void cylinder_set_robot_color(rome_enum_jevois_color_t color){
  cylinder.robot_color = color;
}

uint16_t cylinder_get_position_zero(void){
  return balleater_pos[0];
}

uint16_t cylinder_get_position(void){
  if (cylinder.state < CYLINDER_THROWBALLS)
    return balleater_pos[cylinder.position];
  else
    return turbine_pos[cylinder.position];
}

void cylinder_set_throw_power(uint16_t pwr){
  if (pwr < 2500)
    cylinder.throw_power = pwr;
}

void cylinder_set_trash_power(uint16_t pwr){
  if (pwr < 2500)
    cylinder.trash_power = pwr;
}
