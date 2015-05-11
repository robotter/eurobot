#include "spot_elevator.h"
#include <ax12/ax12.h>
#include <pwm/motor.h>
#include "color_defs.h"
#include <avarix.h>
#include <stdlib.h>
#include <rome/rome.h>
#include "config.h"

extern ax12_t ax12;

extern robot_color_t robot_color;

extern rome_intf_t rome_strat;

#define AX12_ELEVATOR_POS_ERROR 50
#define AX12_CLAW_POS_ERROR 10

// return ax12 load ranging from 0 to 1023, or -1 on error
static int16_t _get_ax12_load(ax12_addr_t address) {
  uint16_t load;
  uint8_t rv = ax12_read_word(&ax12, address, AX12_ADDR_PRESENT_LOAD_L, &load);

  if(rv&AX12_ERROR_INVALID_PACKET) {
    return -1;
  }

  // bit 10 is the load direction, remove it
  return load & 0x3ff;
}

// set the claw ax12 to one of the _claw_ax12_positions_t enum positions
static bool _set_claw_ax12(spot_elevator_t *elevator, _claw_ax12_positions_t pos)
{
  uint8_t rv = 0;
  ax12_addr_t address = elevator->claw_ax12_addr;
  rv |= ax12_write_word(&ax12, address, AX12_ADDR_MOVING_SPEED_L, 0x1FF);
  rv |= ax12_write_byte(&ax12, address, AX12_ADDR_TORQUE_ENABLE, 0x01);
  rv |= ax12_write_word(&ax12, address, AX12_ADDR_TORQUE_LIMIT_L, 0x150);
  rv |= ax12_write_word(&ax12, address, AX12_ADDR_GOAL_POSITION_L,
    elevator->claw_ax12_positions[pos]);

  return !(rv&AX12_ERROR_INVALID_PACKET);
}


static bool _set_elevator_ax12(spot_elevator_t *elevator, _elevator_ax12_positions_t pos, 
                        _elevator_ax12_positions_t speed)
{
  uint8_t rv = 0;
  ax12_addr_t address = elevator->elevator_ax12_addr;
  rv |= ax12_write_word(&ax12, address, AX12_ADDR_MOVING_SPEED_L, speed);
  rv |= ax12_write_byte(&ax12, address, AX12_ADDR_TORQUE_ENABLE, 0x01);
  rv |= ax12_write_word(&ax12, address, AX12_ADDR_TORQUE_LIMIT_L, 0x3ff);
  rv |= ax12_write_word(&ax12, address, AX12_ADDR_GOAL_POSITION_L,
    elevator->elevator_ax12_positions[pos]);

  return !(rv&AX12_ERROR_INVALID_PACKET);
}

bool _is_claw_ax12_in_position(spot_elevator_t *elevator, _elevator_ax12_positions_t pos)
{
  uint16_t read_pos = 0;

  ax12_read_word(&ax12,
                  elevator->claw_ax12_addr,
                  AX12_ADDR_PRESENT_POSITION_L,
                  &read_pos);
  int16_t read_load = _get_ax12_load(elevator->claw_ax12_addr);

  uint16_t consign = elevator->claw_ax12_positions[pos];
  int16_t diff = (int16_t)consign - (int16_t) read_pos;

  if (read_load > 0) {
    // load too high, set position to current pos and declare claw in position
    if (read_load > 200)
      elevator->claw_blocked_cnt ++;
    else
      elevator->claw_blocked_cnt = 0;
  
    if (elevator->claw_blocked_cnt > 10){
      uint16_t newpos = (read_pos + 2*consign)/3;
      ax12_write_word(&ax12, elevator->claw_ax12_addr, AX12_ADDR_GOAL_POSITION_L,newpos);
      elevator->claw_blocked_cnt = 0;
      return true;
      }
  }

  if ( ((diff >=0) && (diff < AX12_CLAW_POS_ERROR)) ||  ((diff < 0) && (diff >- AX12_CLAW_POS_ERROR)) )
  {
    elevator->claw_blocked_cnt = 0;
    return true;
  }
  else
  {
    return false;
  }

}

bool _is_elevator_ax12_in_position(spot_elevator_t *elevator, _elevator_ax12_positions_t pos)
{
  uint16_t read_pos = 0;

  ax12_read_word(&ax12,
                  elevator->elevator_ax12_addr,
                  AX12_ADDR_PRESENT_POSITION_L,
                  &read_pos);
  int16_t read_load = _get_ax12_load(elevator->elevator_ax12_addr);

  uint16_t consign = elevator->elevator_ax12_positions[pos];
  int16_t diff = (int16_t)consign - (int16_t) read_pos;

  if (read_load > 0) {
    // load too high, set position to current pos and declare claw in position
    if (read_load > 800)
      elevator->elev_blocked_cnt ++;
    else
      elevator->elev_blocked_cnt = 0;
  
    if (elevator->elev_blocked_cnt > 10){
      elevator->elev_blocked_cnt = 0;
      return true;
      }
  }

  if ( ((diff >=0) && (diff < AX12_ELEVATOR_POS_ERROR)) ||  ((diff < 0) && (diff >- AX12_ELEVATOR_POS_ERROR)) )
  {
    elevator->elev_blocked_cnt = 0;
    return true;
  }
  else
  {
    return false;
  }
}

void _spipe_close(spot_elevator_t *elevator){
  pwm_motor_set(&elevator->spipe_servo, elevator->spipe_close);
}

void _spipe_open(spot_elevator_t *elevator){
  pwm_motor_set(&elevator->spipe_servo, elevator->spipe_open);
}

// public functions
void spot_elevator_init(spot_elevator_t *elevator)
{
  if (elevator!= NULL)
  {
    elevator->sm_state = SESM_INIT;
    elevator->is_spot_present = NULL;
    elevator->get_spot_color = NULL;
    elevator->is_active = false;
    elevator->nb_spots = 0;
    elevator->tm_state = SESM_TM_S(READY);
    elevator->spipe_state = SESM_SPIPE_CLOSE;
  }
}

void spot_elevator_set_claw_ax12_addr(spot_elevator_t *elevator, ax12_addr_t addr)
{
  if (elevator!= NULL)
  {
    elevator->claw_ax12_addr = addr; 
  }
}

void spot_elevator_set_elevator_ax12_addr(spot_elevator_t *elevator, ax12_addr_t addr)
{
  if (elevator!= NULL)
  {
    elevator->elevator_ax12_addr = addr;
  }
}

void spot_elevator_init_spipe_servo(spot_elevator_t *elevator, char channel, int16_t open_pwm_us, int16_t close_pwm_us){
  pwm_servo_init(&elevator->spipe_servo, SE_SERVO_TUBE_PORT, channel);
  elevator->spipe_open  = open_pwm_us;
  elevator->spipe_close = close_pwm_us;
  _spipe_open(elevator);
}

void spot_elevator_set_is_spot_present_fn(spot_elevator_t *elevator, bool (*is_spot_present)(void))
{
  if (elevator!= NULL)
  {
    elevator->is_spot_present = is_spot_present;
  }
}

void spot_elevator_set_get_spot_color_fn(spot_elevator_t *elevator, robot_color_t(*get_spot_color)(void))
{
  if (elevator!= NULL)
  {
    elevator->get_spot_color = get_spot_color;
  }
}

void spot_elevator_manage(spot_elevator_t *elevator)
{
  if ((elevator != NULL) && 
      (elevator->is_active))
  {
    //static int lstate = INT16_MAX;
    //if(lstate != elevator->sm_state) {
    //  ROME_LOGF(&rome_strat, DEBUG,"state %u", elevator->sm_state);
    //}
    //lstate = elevator->sm_state;
    // each state uses a non blocking function that returns the next state of the state machine
    switch(elevator->sm_state)
    {
      case SESM_INIT:
        ax12_write_word(&ax12,
                  elevator->claw_ax12_addr,
                  AX12_ADDR_TORQUE_LIMIT_L,
                  0x0060);
 
        ax12_write_word(&ax12,
                  elevator->claw_ax12_addr,
                  AX12_ADDR_TORQUE_LIMIT_L,
                  0x0060);

        elevator->nb_spots = 0;
        elevator->sm_state = SESM_INACTIVE;
        break;

      case SESM_INACTIVE:
        elevator->tm_state = SESM_TM_S(READY);
        //waiting state, do noting
        break;

      case SESM_PREPARE_CLAW_FOR_BULB_INIT:
      case SESM_PREPARE_CLAW_FOR_BULB_CLAW:
        _spipe_close(elevator);
        if(_set_claw_ax12(elevator, CLAW_OPENED))
          elevator->sm_state = SESM_PREPARE_CLAW_FOR_BULB_CLAW_WAIT;
        break;

      case SESM_PREPARE_CLAW_FOR_BULB_CLAW_WAIT:
        if(_is_claw_ax12_in_position(elevator, CLAW_OPENED))
          elevator->sm_state = SESM_PREPARE_CLAW_FOR_BULB_ELEV;
        break;

      case SESM_PREPARE_CLAW_FOR_BULB_ELEV:
        if(_set_elevator_ax12(elevator, ELEVATOR_DOWN_WAITING_BULB, ELEVATOR_FAST))
          elevator->sm_state = SESM_PREPARE_CLAW_FOR_BULB_ELEV_WAIT;
        break;

      case SESM_PREPARE_CLAW_FOR_BULB_ELEV_WAIT:
        if(_is_elevator_ax12_in_position(elevator, ELEVATOR_DOWN_WAITING_BULB))
          elevator->sm_state = SESM_INACTIVE;
        break;

      case SESM_PREPARE_SPOT_INIT:
      case SESM_PREPARE_SPOT_LIFT_UP_ELEVATOR:
        _spipe_close(elevator);
        if(_set_elevator_ax12(elevator, ELEVATOR_UP, ELEVATOR_FAST))
          elevator->sm_state = SESM_PREPARE_SPOT_WAIT_ELEVATOR_UP;
        break;

      case SESM_PREPARE_SPOT_WAIT_ELEVATOR_UP:
        if(_is_elevator_ax12_in_position(elevator, ELEVATOR_UP))
          elevator->sm_state = SESM_INACTIVE;
        break;

      case SESM_PICK_SPOT_INIT:
      case SESM_PICK_SPOT_PRE_LIFT_UP_ELEVATOR:
        _spipe_open(elevator);
        if(_set_elevator_ax12(elevator, ELEVATOR_UP, ELEVATOR_FAST))
          elevator->sm_state = SESM_PICK_SPOT_PRE_WAIT_ELEVATOR_UP;
        break;

      case SESM_PICK_SPOT_PRE_WAIT_ELEVATOR_UP:
        if(_is_elevator_ax12_in_position(elevator, ELEVATOR_UP))
          elevator->sm_state = SESM_PICK_SPOT_CHECK_SPOT_PRESENCE;
        break;

      case SESM_PICK_SPOT_CHECK_SPOT_PRESENCE:
        if (elevator->is_spot_present == NULL){
          elevator->sm_state = SESM_INACTIVE; 
          break;
        }
        if (elevator->is_spot_present())
          elevator->sm_state = SESM_PICK_SPOT_CHECK_SPOT_COLOR;
        break;

      case  SESM_PICK_SPOT_CHECK_SPOT_COLOR:
        if (elevator->get_spot_color == NULL){
          elevator->sm_state = SESM_INACTIVE; 
          break;
        }
        if(elevator->get_spot_color()){
          //for first spot, gently put the bulb on it
          if(elevator->nb_spots == 0)
            elevator->sm_state = SESM_PICK_SPOT_LIFT_DOWN_ELEVATOR_PUT_BULB;
          else
            elevator->sm_state = SESM_PICK_SPOT_OPEN_CLAW;
        }
        break;

      case SESM_PICK_SPOT_LIFT_DOWN_ELEVATOR_PUT_BULB:
        if(_set_elevator_ax12(elevator, ELEVATOR_DOWN_PUT_BULB, ELEVATOR_FAST))
          elevator->sm_state = SESM_PICK_SPOT_LIFT_DOWN_ELEVATOR_PUT_BULB_WAIT;
        break;

      case SESM_PICK_SPOT_LIFT_DOWN_ELEVATOR_PUT_BULB_WAIT:
        if(_is_elevator_ax12_in_position(elevator, ELEVATOR_DOWN_PUT_BULB))
          elevator->sm_state = SESM_PICK_SPOT_OPEN_CLAW;
        break;

      case SESM_PICK_SPOT_OPEN_CLAW: 
        if(_set_claw_ax12(elevator, CLAW_OPENED))
          elevator->sm_state = SESM_PICK_SPOT_WAIT_CLAW_OPENED; 
        break;

      case SESM_PICK_SPOT_WAIT_CLAW_OPENED:
        if (_is_claw_ax12_in_position(elevator, CLAW_OPENED))
          elevator->sm_state = SESM_PICK_SPOT_LIFT_DOWN_ELEVATOR;
        break;

      case SESM_PICK_SPOT_LIFT_DOWN_ELEVATOR:
        if(_set_elevator_ax12(elevator, ELEVATOR_DOWN_WAITING_SPOT, ELEVATOR_FAST))
          elevator->sm_state = SESM_PICK_SPOT_WAIT_ELEVATOR_DOWN;
        break;

      case SESM_PICK_SPOT_WAIT_ELEVATOR_DOWN:
        if (_is_elevator_ax12_in_position(elevator, ELEVATOR_DOWN_WAITING_SPOT))
          elevator->sm_state = SESM_PICK_SPOT_CLOSE_CLAW;
        break;

      case SESM_PICK_SPOT_CLOSE_CLAW:
        if(_set_claw_ax12(elevator, CLAW_CLOSED_FOR_SPOT))
          elevator->sm_state = SESM_PICK_SPOT_WAIT_CLAW_CLOSED;
        break;

      case SESM_PICK_SPOT_WAIT_CLAW_CLOSED:
        if(_is_claw_ax12_in_position(elevator, CLAW_CLOSED_FOR_SPOT))
          elevator->sm_state = SESM_PICK_SPOT_POST_LIFT_UP_ELEVATOR;
        break;

      case SESM_PICK_SPOT_POST_LIFT_UP_ELEVATOR:
        _spipe_open(elevator);
        elevator->nb_spots ++;
        if(elevator->nb_spots < 4)
            spot_elevator_prepare_spot_stacking(elevator);
        else
          if(_set_elevator_ax12(elevator, ELEVATOR_UP_MOVE_SPOT, ELEVATOR_FAST))
            elevator->sm_state = SESM_PICK_SPOT_POST_WAIT_ELEVATOR_UP;
        break;

      case SESM_PICK_SPOT_POST_WAIT_ELEVATOR_UP:
        _spipe_close(elevator);
        if(_is_elevator_ax12_in_position(elevator, ELEVATOR_UP_MOVE_SPOT)){
            elevator->sm_state = SESM_INACTIVE;
        }
        break;

      case SESM_PICK_BULB_INIT:
      case SESM_CLOSE_CLAW_FOR_BULB:
        if(_set_claw_ax12(elevator, CLAW_CLOSED_FOR_BULB))
          elevator->sm_state = SESM_WAIT_CLAW_CLOSED_FOR_BULB;
        break;

      case SESM_WAIT_CLAW_CLOSED_FOR_BULB:
        _spipe_open(elevator);
        if(_is_claw_ax12_in_position(elevator, CLAW_CLOSED_FOR_BULB))
          elevator->sm_state = SESM_LIFT_UP_ELEVATOR_FOR_BULB;
        break;

      case SESM_LIFT_UP_ELEVATOR_FOR_BULB:
        if(_set_elevator_ax12(elevator, ELEVATOR_UP, ELEVATOR_FAST))
          elevator->sm_state = SESM_WAIT_ELEVATOR_UP_FOR_BULB;
        break;

      case SESM_WAIT_ELEVATOR_UP_FOR_BULB:
        if(_is_elevator_ax12_in_position(elevator, ELEVATOR_UP)){
          elevator->sm_state = SESM_INACTIVE;
          _spipe_close(elevator);
        }
        break;

      case SESM_DISCHARGE_INIT:
      case SESM_DISCHARGE_ELEVATOR_DOWN:
        if(_set_elevator_ax12(elevator, ELEVATOR_DOWN_WAITING_SPOT, ELEVATOR_SLOW))
          elevator->sm_state = SESM_DISCHARGE_ELEVATOR_DOWN_WAIT;
        break;

      case SESM_DISCHARGE_ELEVATOR_DOWN_WAIT:
        if(_is_elevator_ax12_in_position(elevator, ELEVATOR_DOWN_WAITING_SPOT))
          elevator->sm_state = SESM_DISCHARGE_CLAW_OPEN;
        break;

      case SESM_DISCHARGE_CLAW_OPEN:
        elevator->nb_spots = 0;
        if(_set_claw_ax12(elevator, CLAW_OPENED))
          elevator->sm_state = SESM_DISCHARGE_CLAW_OPEN_WAIT;
        break;

      case SESM_DISCHARGE_CLAW_OPEN_WAIT:
        if ( _is_claw_ax12_in_position(elevator, CLAW_OPENED))
          elevator->sm_state = SESM_INACTIVE;
        break;

      case SESM_DISCHARGE_RELEASE_PILE:
      case SESM_DISCHARGE_ELEVATOR_UP:
        if(_set_elevator_ax12(elevator, ELEVATOR_UP, ELEVATOR_FAST))
          elevator->sm_state = SESM_DISCHARGE_ELEVATOR_UP_WAIT;
        break;

      case SESM_DISCHARGE_ELEVATOR_UP_WAIT:
        elevator->tm_state = SESM_TM_S(GROUND_CLEAR);
        _spipe_open(elevator);
        if(_is_elevator_ax12_in_position(elevator, ELEVATOR_UP))
          elevator->sm_state = SESM_INACTIVE;
        break;

      case SESM_PICK_CUP_CLAW_OPEN:
        _spipe_close(elevator);
        if(_set_claw_ax12(elevator, CLAW_OPENED))
          elevator->sm_state = SESM_PICK_CUP_CLAW_OPEN_WAIT;
        break;

      case SESM_PICK_CUP_CLAW_OPEN_WAIT:
        if(_is_claw_ax12_in_position(elevator, CLAW_OPENED))
          elevator->sm_state = SESM_PICK_CUP_ELEVATOR_DOWN;
        break;

      case SESM_PICK_CUP_ELEVATOR_DOWN:
        if(_set_elevator_ax12(elevator,ELEVATOR_DOWN_WAITING_SPOT,ELEVATOR_FAST))
          elevator->sm_state = SESM_PICK_CUP_ELEVATOR_DOWN_WAIT;
        break;

      case SESM_PICK_CUP_ELEVATOR_DOWN_WAIT:
        if(_is_elevator_ax12_in_position(elevator, ELEVATOR_DOWN_WAITING_SPOT)){
          elevator->sm_state = SESM_INACTIVE;
          _spipe_open(elevator);
        }
        break;

      case SESM_PICK_CUP_CHECK_CUP_PRESENCE:
        _spipe_open(elevator);
        if (elevator->is_spot_present == NULL){
          elevator->sm_state = SESM_INACTIVE;
          break;
        }
        
        if(elevator->is_spot_present())
          elevator->sm_state = SESM_PICK_CUP_CLAW_CLOSE;
        break;

      case SESM_PICK_CUP_CLAW_CLOSE:
        if(_set_claw_ax12(elevator,CLAW_CLOSED_FOR_SPOT))
          elevator->sm_state = SESM_PICK_CUP_CLAW_CLOSE_WAIT;
        break;

      case SESM_PICK_CUP_CLAW_CLOSE_WAIT:
        if(_is_claw_ax12_in_position(elevator, CLAW_CLOSED_FOR_SPOT))
          elevator->sm_state= SESM_PICK_CUP_ELEVATOR_UP;
        break;

     case SESM_PICK_CUP_ELEVATOR_UP:
        if(_set_elevator_ax12(elevator,ELEVATOR_UP_MOVE_SPOT,ELEVATOR_FAST))
          elevator->sm_state = SESM_PICK_CUP_ELEVATOR_UP_WAIT;
        break;

      case SESM_PICK_CUP_ELEVATOR_UP_WAIT:
        if(_is_elevator_ax12_in_position(elevator, ELEVATOR_UP_MOVE_SPOT)){
          elevator->sm_state = SESM_INACTIVE;
          _spipe_close(elevator);
        }
        break;
 
      case SESM_ENDOFMATCH_CLAW_OPEN:
        if(_set_claw_ax12(elevator, CLAW_OPENED))
          elevator->sm_state = SESM_ENDOFMATCH_CLAW_OPEN_WAIT;
        break;
 
      case SESM_ENDOFMATCH_CLAW_OPEN_WAIT:
        if(_is_claw_ax12_in_position(elevator, CLAW_OPENED))
          elevator->sm_state = SESM_ENDOFMATCH_SPIPE_OPEN;
        break;

      case SESM_ENDOFMATCH_SPIPE_OPEN:
        _spipe_open(elevator);
        elevator->sm_state = SESM_ENDOFMATCH;
        break;

      case SESM_ENDOFMATCH:
        elevator->tm_state = SESM_TM_S(READY);
        break;

      default : 
      // shall not arrive !!!
       elevator->sm_state = SESM_INACTIVE;
       break;
    }
  }
}


void spot_elevator_set_enable(spot_elevator_t *se, bool enable)
{
  se->tm_state = SESM_TM_S(BUSY);
  se->is_active = enable;
}

void spot_elevator_prepare_bulb(spot_elevator_t *se)
{
  se->tm_state = SESM_TM_S(BUSY);
  se->sm_state = SESM_PREPARE_CLAW_FOR_BULB_INIT;
}

void spot_elevator_pick_bulb(spot_elevator_t *se)
{
  se->tm_state = SESM_TM_S(BUSY);
  se->sm_state = SESM_PICK_BULB_INIT;
}

void spot_elevator_automatic_spot_stacking(spot_elevator_t *se)
{
  se->tm_state = SESM_TM_S(BUSY);
  se->sm_state = SESM_PICK_SPOT_INIT;
}

void spot_elevator_prepare_spot_stacking(spot_elevator_t *se)
{
  se->tm_state = SESM_TM_S(GROUND_CLEAR);
  se->sm_state = SESM_PREPARE_SPOT_INIT;
}

void spot_elevator_discharge_spot_stack(spot_elevator_t *se)
{
  se->tm_state = SESM_TM_S(BUSY);
  se->sm_state = SESM_DISCHARGE_INIT;
}

void spot_elevator_release_spot_stack(spot_elevator_t *se)
{
  se->tm_state = SESM_TM_S(BUSY);
  se->sm_state = SESM_DISCHARGE_RELEASE_PILE;
}

void spot_elevator_move_middle_arm(uint16_t position){
  int16_t ax12_consign = position;
  if (ax12_consign > SE_MIDDLE_ARM_MAX)
    ax12_consign = SE_MIDDLE_ARM_MAX;
  if (ax12_consign < SE_MIDDLE_ARM_MIN)
    ax12_consign = SE_MIDDLE_ARM_MIN;

  ax12_write_word(&ax12, SE_AX12_MIDDLE_ARM_ID, AX12_ADDR_MOVING_SPEED_L,  0x1FF);
  ax12_write_byte(&ax12, SE_AX12_MIDDLE_ARM_ID, AX12_ADDR_TORQUE_ENABLE,   0x01);
  ax12_write_word(&ax12, SE_AX12_MIDDLE_ARM_ID, AX12_ADDR_GOAL_POSITION_L, ax12_consign);
}

void spot_elevator_prepare_cup(spot_elevator_t *se){
  se->tm_state = SESM_TM_S(GROUND_CLEAR);
  se->sm_state = SESM_PICK_CUP_CLAW_OPEN;
}
void spot_elevator_unload_cup(spot_elevator_t *se){
  se->tm_state = SESM_TM_S(BUSY);
  se->sm_state = SESM_PICK_CUP_CLAW_OPEN;
}

void spot_elevator_pick_cup(spot_elevator_t *se){
  se->tm_state = SESM_TM_S(BUSY);
  se->sm_state = SESM_PICK_CUP_CLAW_CLOSE;
}

void spot_elevator_end_of_match(spot_elevator_t *se){
  if ((se->sm_state != SESM_ENDOFMATCH_CLAW_OPEN  )&&
      (se->sm_state != SESM_ENDOFMATCH_CLAW_OPEN_WAIT )&&
      (se->sm_state != SESM_ENDOFMATCH_SPIPE_OPEN )&&
      (se->sm_state != SESM_ENDOFMATCH            )  )
    se->sm_state = SESM_ENDOFMATCH_CLAW_OPEN;
}

void spot_elevator_reset(spot_elevator_t *se){
  se->tm_state = SESM_TM_S(BUSY);
  se->sm_state = SESM_INIT;
}
