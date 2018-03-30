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
const uint16_t balleater_pos[CYLINDER_NB_POS] = CYLINDER_BALLEATER_POS;
const uint16_t turbine_pos[CYLINDER_NB_POS]   = CYLINDER_TURBINE_POS;


#define RETRY_AND_UPDATE(fcall) while(!(fcall)) { \
  portpin_outtgl(&LED_AN_PP(0)); \
  _delay_ms(10); \
  }

#ifdef USE_AX12
// return ax12 load ranging from 0 to 1023, or -1 on error
static int16_t _get_ax12_load(uint8_t address) {
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
  uint8_t address = CYLINDER_AX12_ID;
  rv |= ax12_write_word(&ax12, address, AX12_ADDR_CW_ANGLE_LIMIT_L,  0x000);
  rv |= ax12_write_word(&ax12, address, AX12_ADDR_CCW_ANGLE_LIMIT_L, 0x3FF);
  rv |= ax12_write_word(&ax12, address, AX12_ADDR_MOVING_SPEED_L, 0x0FF);

  return !(rv & AX12_ERROR_INVALID_PACKET);
}

static bool ax12_cylinder_move(uint16_t position, uint16_t speed){
  uint8_t rv = 0;
  uint8_t address = CYLINDER_AX12_ID;
  rv |= ax12_write_word(&ax12, address, AX12_ADDR_MOVING_SPEED_L,  speed);
  rv |= ax12_write_byte(&ax12, address, AX12_ADDR_TORQUE_ENABLE,   0x01);
  rv |= ax12_write_word(&ax12, address, AX12_ADDR_GOAL_POSITION_L, position);

  return !(rv & AX12_ERROR_INVALID_PACKET);
}

static bool ax12_cylinder_rotate(uint16_t speed){
  uint8_t rv = 0;
  uint8_t address = CYLINDER_AX12_ID;
  rv |= ax12_write_word(&ax12, address, AX12_ADDR_CW_ANGLE_LIMIT_L,  0x000);
  rv |= ax12_write_word(&ax12, address, AX12_ADDR_CCW_ANGLE_LIMIT_L,  0x000);
  rv |= ax12_write_byte(&ax12, address, AX12_ADDR_TORQUE_ENABLE,   0x01);
  rv |= ax12_write_word(&ax12, address, AX12_ADDR_MOVING_SPEED_L, speed);

  return !(rv & AX12_ERROR_INVALID_PACKET);
}

static bool ax12_cylinder_in_position(uint16_t position){

  uint16_t read_pos = 0;
  ax12_read_word(&ax12,
                  CYLINDER_AX12_ID,
                  AX12_ADDR_PRESENT_POSITION_L,
                  &read_pos);
  //int16_t read_load = _get_ax12_load(CYLINDER_AX12_ID);
  (void) _get_ax12_load;

  int16_t diff = (int16_t)position - (int16_t) read_pos;
  if ( ((diff >=0) && (diff < 5)) ||  ((diff < 0) && (diff >- 5)) )
    return true;
  else
    return false;
}
#endif

#ifdef USE_MX12
// return mx12 load ranging from 0 to 1023, or -1 on error
static int16_t _get_mx12_load(uint8_t address) {
  uint16_t load;
  uint8_t rv = ax12_read_word(&ax12, address, MX12_ADDR_PRESENT_LOAD_L, &load);

  if(rv == 0) {
    return -1;
  }

  // bit 10 is the load direction, remove it
  return load & 0x3ff;
}

/*
static bool mx12_cylinder_set_posmode(void){
  uint8_t rv = 0;
  uint8_t address = CYLINDER_MX12_ID;
  rv |= ax12_write_word(&ax12, address, MX12_ADDR_CW_ANGLE_LIMIT_L,  0x000);
  rv |= ax12_write_word(&ax12, address, MX12_ADDR_CCW_ANGLE_LIMIT_L, 0x3FF);
  rv |= ax12_write_word(&ax12, address, MX12_ADDR_MOVING_SPEED_L, 0x0FF);

  return !(rv & AX12_ERROR_INVALID_PACKET);
}

static bool mx12_cylinder_rotate(uint16_t speed){
  uint8_t rv = 0;
  uint8_t address = CYLINDER_MX12_ID;
  rv |= ax12_write_word(&ax12, address, MX12_ADDR_CW_ANGLE_LIMIT_L,  0x000);
  rv |= ax12_write_word(&ax12, address, MX12_ADDR_CCW_ANGLE_LIMIT_L,  0x000);
  rv |= ax12_write_byte(&ax12, address, MX12_ADDR_TORQUE_ENABLE,   0x01);
  rv |= ax12_write_word(&ax12, address, MX12_ADDR_MOVING_SPEED_L, speed);

  return !(rv & AX12_ERROR_INVALID_PACKET);
}
*/
static bool mx12_cylinder_move(uint16_t position, uint16_t speed){
  uint8_t rv = 0;
  uint8_t address = CYLINDER_MX12_ID;
  rv |= ax12_write_word(&ax12, address, MX12_ADDR_MOVING_SPEED_L,  speed);
  rv |= ax12_write_byte(&ax12, address, MX12_ADDR_TORQUE_ENABLE,   0x01);
  rv |= ax12_write_word(&ax12, address, MX12_ADDR_GOAL_POSITION_L, position);

  return !(rv & AX12_ERROR_INVALID_PACKET);
}

static bool mx12_cylinder_in_position(uint16_t position){

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

static void turbine_off(void) { servo_set(1,1300); }
static void turbine_full(void){ servo_set(1,1900); }
//static void turbine_low(void){ servo_set(1,1650); }

static void balleater_off(void) { servo_set(0,2455); }
static void balleater_on(void)  { servo_set(0,2000); }
//static void balleater_reverse(void) { servo_set(0,2500); }

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

static bool cylinder_goto_position(uint16_t position){
#ifdef USE_AX12
  return ax12_cylinder_move(position, 0x3ff);
#endif
#ifdef USE_MX12
  return mx12_cylinder_move(position, 0x3ff);
#endif
}

static void cylinder_position_increment(void) {
  // shift cw
  cylinder.position++;
  cylinder.position %= CYLINDER_NB_POS;
}

static bool cylinder_balleater_move(void) {
  // send command to servo
  return cylinder_goto_position(balleater_pos[cylinder.position]);
}

static bool cylinder_balleater_move_done(void) {
#ifdef USE_AX12
  return ax12_cylinder_in_position(balleater_pos[cylinder.position]);
#endif
#ifdef USE_MX12
  return mx12_cylinder_in_position(balleater_pos[cylinder.position]);
#endif
}

static bool cylinder_turbine_move(void) {
  // send command to servo
  return cylinder_goto_position(turbine_pos[cylinder.position]);
}

static bool cylinder_turbine_move_done(void) {
#ifdef USE_AX12
  return ax12_cylinder_in_position(turbine_pos[cylinder.position]);
#endif
#ifdef USE_MX12
  return mx12_cylinder_in_position(turbine_pos[cylinder.position]);
#endif
}

static void set_current_slot_color(jevois_color_t jvc){
  cylinder.ball_color[cylinder.position] = jvc;
}

static uint8_t cylinder_get_next_color_slot(jevois_color_t jvc){
  uint8_t i = 0;
  bool first_slot_found = false;
  for( i=0; i < CYLINDER_NB_POS; i++){
    if (cylinder.ball_color[i] == jvc)
      first_slot_found = true;
    else
      if (first_slot_found)
        return i;
  }

  if (jvc == cylinder.robot_color)
    return 0;
  else
    return 4;
}

//static jevois_color_t get_current_slot_color(void){
//  return cylinder.ball_color[cylinder.position];
//}

static bool cylinder_is_full(void){
  uint8_t balls_loaded = 0;
  for (uint8_t i = 0; i < CYLINDER_NB_POS; i++){
    if (cylinder.ball_color[i] != JEVOIS_COLOR_NONE)
      balls_loaded ++;
  }
  return (balls_loaded == CYLINDER_NB_POS);
}

static void cylinder_reset_ball_colors(void){
  for (uint8_t i = 0; i < CYLINDER_NB_POS; i++)
    cylinder.ball_color[i] = JEVOIS_COLOR_NONE;
}
void cylinder_init(void){
  turbine_off();
  balleater_off();
  cylinder.state = CYLINDER_INIT;
  cylinder.robot_color = JEVOIS_COLOR_GREEN;
  cylinder.moving_ts = uptime_us();
  ax12_cylinder_set_posmode();
  cylinder_reset_ball_colors();
}


void cylinder_update(void){
  static cylinder_state_t os = 1;
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
  }
  os = cylinder.state;

  switch (cylinder.state){
    case CYLINDER_INIT:{
      bool delay = cylinder.moving_ts + CYLINDER_TURBINE_BOOT_DELAY_US < uptime_us();
      if(!delay)
        break;
      turbine_off();
      balleater_off();
      //cylinder.position = 0;
      //cylinder_balleater_move();
      #if 0
      cylinder.state = CYLINDER_IS_FULL;
      #else
      cylinder.state = CYLINDER_CHECK_EMPTY_PREPARE;
      #endif
      cylinder.moving_ts = 0;
      cylinder.drop_ts = 0;
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

      if (cylinder_is_full()){
        cylinder.state = CYLINDER_IS_FULL;
        break;
      }
      if (cylinder.position == CYLINDER_NB_POS - 1 ){
        cylinder.state = CYLINDER_BALLEATER_PRE_TAKE;
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
      //wait for a valid camera frame
      if (!jevois_cam_is_valid(&cam))
        break;
      if (jevois_cam_get_entry_color(&cam) != JEVOIS_COLOR_NONE){
        cylinder.position = cylinder_get_next_color_slot(jevois_cam_get_entry_color(&cam));
        cylinder.state = CYLINDER_EATING_FIND_EMPTY_ORDER_MOVING;
        balleater_off();
      }
      break;

    case CYLINDER_EATING_FIND_EMPTY: {
      //wait for a valid camera frame
      if (!jevois_cam_is_valid(&cam))
        break;

      //if there is no ball in the eater, go taking one
      if (jevois_cam_get_entry_color(&cam) == JEVOIS_COLOR_NONE)
        cylinder.state = CYLINDER_BALLEATER_PRE_TAKE;

      bool delay = cylinder.moving_ts + CYLINDER_MOVING_DELAY_US < uptime_us();
      if(!delay)
        break;

      bool cylinder_empty =
        jevois_cam_is_valid(&cam) && jevois_cam_get_cylinder_color(&cam) == JEVOIS_COLOR_NONE;

      // cylinder position empty lets roll
      if(cylinder_empty) {
        set_current_slot_color(JEVOIS_COLOR_NONE);
        cylinder.drop_ts = uptime_us();
        cylinder.state = CYLINDER_BALLEATER_TAKE;
        break;
      }

      // shift to next position
      cylinder_position_increment();
      cylinder.state = CYLINDER_EATING_FIND_EMPTY_ORDER_MOVING;
      // no break
    }

    case CYLINDER_EATING_FIND_EMPTY_ORDER_MOVING:
      if(!cylinder_balleater_move())
        break;
      cylinder.state = CYLINDER_EATING_FIND_EMPTY_MOVING;
      break;

    case CYLINDER_EATING_FIND_EMPTY_MOVING:
      if(cylinder_balleater_move_done()) {
        cylinder.state = CYLINDER_EATING_FIND_EMPTY;
        cylinder.moving_ts = uptime_us();
      }
      break;

    case CYLINDER_BALLEATER_TAKE: {
      balleater_on();

      //wait for a valid camera frame
      if (!jevois_cam_is_valid(&cam))
        break;

      bool entry_empty = jevois_cam_get_entry_height(&cam) < 150;
      bool cylinder_present = jevois_cam_get_cylinder_color(&cam) != JEVOIS_COLOR_NONE;

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

      //if all balls are loaded, trow them
      if (cylinder_is_full()){
        cylinder.state = CYLINDER_IS_FULL;
        break;
      }
      cylinder.state = CYLINDER_BALLEATER_PRE_TAKE;
      break;
    }
    case CYLINDER_IS_FULL:
    case CYLINDER_THROWBALLS_WHEELMODE_PREPARE_ORDER_MOVE:
      cylinder.position = 2;
      if (!cylinder_turbine_move())
        break;
      cylinder.state = CYLINDER_THROWBALLS_WHEELMODE_PREPARE_MOVING;
      //no break
    case CYLINDER_THROWBALLS_WHEELMODE_PREPARE_MOVING:
      if (!cylinder_turbine_move_done())
        break;
      turbine_full();
      cylinder.state = CYLINDER_THROWBALLS_WHEELMODE;
      cylinder.throw_ts = uptime_us();
      //no break

    case CYLINDER_THROWBALLS_WHEELMODE:{
      bool delay = cylinder.throw_ts + CYLINDER_TURBINE_STOP_DELAY_US < uptime_us();
      if (!delay)
        break;
      if(!ax12_cylinder_rotate(0x15F))
        break;
      cylinder.state = CYLINDER_THROWBALLS_WHEELMODE_TURNING;
      cylinder.position = 7;
      break;
    }
    case CYLINDER_THROWBALLS_WHEELMODE_TURNING:
      if(!cylinder_turbine_move_done())
        break;
      turbine_off();
      ax12_cylinder_set_posmode();
      cylinder_reset_ball_colors();
      cylinder.state = CYLINDER_CHECK_EMPTY_PREPARE;
      break;

    case CYLINDER_THROWBALLS_PREPARE:
      cylinder.position = 2;
      if(!cylinder_turbine_move())
        break;
      cylinder.state = CYLINDER_THROWBALLS_PREPARE_MOVING;
      break;

    case CYLINDER_THROWBALLS_PREPARE_MOVING:
      if(cylinder_turbine_move_done()) {
        //cylinder.moving_ts = uptime_us();
        cylinder.position = 0;
        cylinder.state = CYLINDER_THROWBALLS_ORDER_MOVING;
      }
      break;

    case CYLINDER_THROWBALLS:{
      //bool delay = cylinder.moving_ts + CYLINDER_MOVING_DELAY_US < uptime_us();
      //if(!delay)
      //  break;

      turbine_full();
      if (0){//cylinder_current_slot_empty()){
        cylinder_position_increment();
        if (cylinder.position == 2)
          cylinder_position_increment();
        cylinder.state = CYLINDER_THROWBALLS_ORDER_MOVING;
        break;
      }
      cylinder.throw_ts = uptime_us();
      cylinder.state = CYLINDER_THROWBALLS_WAIT_FLYING;
      //no break
    }

    case CYLINDER_THROWBALLS_WAIT_FLYING:{
      bool delay = cylinder.throw_ts + CYLINDER_BALL_FLYING_DELAY_US < uptime_us();
      if(!delay)
        break;
      turbine_off();
      set_current_slot_color(JEVOIS_COLOR_NONE);
      if (cylinder_is_full()){
        cylinder.state = CYLINDER_IDLE;
        break;
      }
      //else wait for turbine to stop before moving cylinder
      cylinder.throw_ts = uptime_us();
      cylinder.state = CYLINDER_THROWBALLS_WAIT_TURBINE_STOP;
      //no break;
    }

    case CYLINDER_THROWBALLS_WAIT_TURBINE_STOP:{
      bool delay = cylinder.throw_ts + CYLINDER_TURBINE_STOP_DELAY_US < uptime_us();
      if(!delay)
        break;

      cylinder_position_increment();
      if (cylinder.position == 2)
        cylinder_position_increment();
      cylinder.state = CYLINDER_THROWBALLS_ORDER_MOVING;
      //no break
    }

    case CYLINDER_THROWBALLS_ORDER_MOVING:
      if(!cylinder_turbine_move())
        break;
      turbine_off();
      cylinder.state = CYLINDER_THROWBALLS_MOVING;
      break;

    case CYLINDER_THROWBALLS_MOVING:
      if(cylinder_turbine_move_done()) {
        cylinder.state = CYLINDER_THROWBALLS;
        cylinder.moving_ts = uptime_us();
      }
      break;

    case CYLINDER_IDLE:
      cylinder.state = CYLINDER_CHECK_EMPTY_PREPARE;
      break;

    default:
      ROME_LOGF(&rome_strat, DEBUG,"meca default case : %d",cylinder.state);
      break;
  }
}
