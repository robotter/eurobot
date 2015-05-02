#include "spot_elevator.h"
#include <ax12/ax12.h>
#include <pwm/motor.h>
#include "color_defs.h"
#include <avarix.h>
#include <stdlib.h>
#include <rome/rome.h>

extern ax12_t ax12;

extern robot_color_t robot_color;

extern rome_intf_t rome_strat;

#define AX12_ELEVATOR_POS_ERROR 10

// set the claw ax12 to one of the _claw_ax12_positions_t enum positions
void _set_claw_ax12(spot_elevator_t *elevator, _claw_ax12_positions_t pos)
{
  ROME_LOGF(&rome_strat, DEBUG,"claw %d", pos);
  ax12_addr_t address = elevator->claw_ax12_addr;
  ax12_write_word(&ax12, address, AX12_ADDR_MOVING_SPEED_L, 0x1FF);
  ax12_write_byte(&ax12, address, AX12_ADDR_TORQUE_ENABLE, 0x01);
  ax12_write_word(&ax12, address, AX12_ADDR_TORQUE_LIMIT_L, 0x390);
  ax12_write_word(&ax12, address, AX12_ADDR_GOAL_POSITION_L,
    elevator->claw_ax12_positions[pos]);
}


void _set_elevator_ax12(spot_elevator_t *elevator, _elevator_ax12_positions_t pos, 
                        _elevator_ax12_positions_t speed)
{
  ax12_addr_t address = elevator->elevator_ax12_addr;
  ax12_write_word(&ax12, address, AX12_ADDR_MOVING_SPEED_L, speed);
  ax12_write_byte(&ax12, address, AX12_ADDR_TORQUE_ENABLE, 0x01);
  ax12_write_word(&ax12, address, AX12_ADDR_GOAL_POSITION_L,
    elevator->elevator_ax12_positions[pos]);
}

bool _is_claw_ax12_in_position(spot_elevator_t *elevator, _elevator_ax12_positions_t pos)
{
  uint16_t read_pos = 0;
  static uint8_t blocked_count = 0;
  uint16_t read_load = 0; 

  ax12_read_word(&ax12,
                  elevator->claw_ax12_addr,
                  AX12_ADDR_PRESENT_POSITION_L,
                  &read_pos);
  ax12_read_word(&ax12,
                  elevator->claw_ax12_addr,
                  AX12_ADDR_PRESENT_LOAD_L,
                  &read_load);
  int16_t diff = (int16_t)elevator->claw_ax12_positions[pos] - (int16_t) read_pos;


  if (read_load > 0x380)
    blocked_count++;
  else
    blocked_count = 0;
    
  ROME_LOGF(&rome_strat, DEBUG,"load %x", read_load);
  if ( ((diff >=0) && (diff < AX12_ELEVATOR_POS_ERROR)) ||  ((diff < 0) && (diff >- AX12_ELEVATOR_POS_ERROR)) )
  {
    return true;
  }
  else
  {
    return false;
  }

  if (blocked_count > 20)
    return true;
}

bool _is_elevator_ax12_in_position(spot_elevator_t *elevator, _elevator_ax12_positions_t pos)
{
  uint16_t read_pos = 0;

  ax12_read_word(&ax12,
                  elevator->elevator_ax12_addr,
                  AX12_ADDR_PRESENT_POSITION_L,
                  &read_pos);
  int16_t diff = (int16_t)elevator->elevator_ax12_positions[pos] - (int16_t) read_pos;

  if ( ((diff >=0) && (diff < AX12_ELEVATOR_POS_ERROR)) ||  ((diff < 0) && (diff >- AX12_ELEVATOR_POS_ERROR)) )
  {
    return true;
  }
  else
  {
    return false;
  }
}

// => state machine functions
_spot_elevator_state_t _sesm_init(spot_elevator_t *elevator)
{
  ax12_write_word(&ax12,
                  elevator->claw_ax12_addr,
                  AX12_ADDR_TORQUE_LIMIT_L,
                  0x0060);
 
  ax12_write_word(&ax12,
                  elevator->claw_ax12_addr,
                  AX12_ADDR_TORQUE_LIMIT_L,
                  0x0060);

  return SESM_INACTIVE; 
}

_spot_elevator_state_t _sesm_inactive(spot_elevator_t *elevator)
{
  return SESM_INACTIVE; 
}

_spot_elevator_state_t _sesm_prepare_claw_for_onboard_buld(spot_elevator_t *elevator)
{
  _set_claw_ax12(elevator, CLAW_CLOSED_FOR_BULB);
  _set_elevator_ax12(elevator, ELEVATOR_UP, ELEVATOR_FAST);

  return SESM_INACTIVE; 
}

_spot_elevator_state_t _sesm_open_claw_for_bulb(spot_elevator_t *elevator)
{
  _set_claw_ax12(elevator, CLAW_OPENED);
  _set_elevator_ax12(elevator, ELEVATOR_DOWN_WAITING_SPOT, ELEVATOR_FAST);
  return SESM_INACTIVE; 
}

_spot_elevator_state_t _sesm_check_spot_presence(spot_elevator_t *elevator)
{
  _spot_elevator_state_t next_state = SESM_INACTIVE;

  if (elevator->is_spot_present != NULL)
  {
    if (elevator->is_spot_present())
    {
     // spot detected => check color 
      next_state = SESM_CHECK_SPOT_COLOR;
    }
    else
    {
      // continue waiting for spot
      next_state = SESM_CHECK_SPOT_PRESENCE;
    }
  }
  return next_state;
}

_spot_elevator_state_t _sesm_check_spot_color(spot_elevator_t *elevator)
{
  _spot_elevator_state_t next_state = SESM_INACTIVE;
  if (elevator->get_spot_color != NULL)
  {
    if (robot_color == elevator->get_spot_color())
    {
      // good spot color => go to next state
      next_state = SESM_CHECK_CLAW_OPENED;
    }
    else
    {
      // wrong color => go to spot detection state
      next_state = SESM_CHECK_SPOT_PRESENCE;
    }
  }
  return next_state;
}

_spot_elevator_state_t _sesm_check_elevator_position(spot_elevator_t *elevator)
{
  _spot_elevator_state_t next_state = SESM_INACTIVE;
  // check if claw is up to allow
  if (_is_elevator_ax12_in_position(elevator, ELEVATOR_UP))
  {
    /// TODO
    (void)next_state;
  }
  
  return next_state; 
}

_spot_elevator_state_t _sesm_check_claw_opened(spot_elevator_t *elevator)
{
  _spot_elevator_state_t next_state = SESM_INACTIVE;
  
  if ( _is_claw_ax12_in_position(elevator, CLAW_OPENED))
  {
    // claw opened => go to elevator lift down
    next_state = SESM_LIFT_DOWN_ELEVATOR;
  }
  else
  {
    next_state = SESM_OPEN_CLAW;
  }
  return next_state; 
}

_spot_elevator_state_t _sesm_open_claw(spot_elevator_t *elevator)
{
  _set_claw_ax12(elevator, CLAW_OPENED);
  return SESM_WAIT_CLAW_OPENED; 
}

_spot_elevator_state_t _sesm_wait_claw_opened(spot_elevator_t *elevator)
{
  _spot_elevator_state_t next_state = SESM_INACTIVE;
  
  if ( _is_claw_ax12_in_position(elevator, CLAW_OPENED))
  {
    next_state = SESM_LIFT_DOWN_ELEVATOR;
  }
  else
  {
    next_state = SESM_OPEN_CLAW;
  }

  return next_state; 
}

_spot_elevator_state_t _sesm_lift_down_elevator(spot_elevator_t *elevator)
{
  _set_elevator_ax12(elevator, ELEVATOR_DOWN_WAITING_SPOT, ELEVATOR_FAST);
  return SESM_WAIT_ELEVATOR_DOWN;
}

_spot_elevator_state_t _sesm_wait_elevator_down(spot_elevator_t *elevator)
{
  _spot_elevator_state_t next_state = SESM_INACTIVE;

  if ( _is_elevator_ax12_in_position(elevator, ELEVATOR_DOWN_WAITING_SPOT))
  {
    // elevator down => go to next state
    next_state = SESM_CLOSE_CLAW;
  }
  else
  {
    next_state = SESM_LIFT_DOWN_ELEVATOR;
  }
  return next_state; 
}

_spot_elevator_state_t _sesm_close_claw(spot_elevator_t *elevator)
{
  _set_claw_ax12(elevator, CLAW_CLOSED_FOR_SPOT);
  return SESM_WAIT_CLAW_CLOSED;
}

_spot_elevator_state_t _sesm_wait_claw_closed(spot_elevator_t *elevator)
{
  /// TODO check if the correct way to check it is not simply torque
  _spot_elevator_state_t next_state = SESM_INACTIVE;

  if (_is_claw_ax12_in_position(elevator, CLAW_CLOSED_FOR_SPOT))
  {
    // claw closed => go to next state
    next_state = SESM_LIFT_UP_ELEVATOR;
  }
  else
  {
    // claw not closed yet => send order again to avoid 
    next_state = SESM_CLOSE_CLAW;
  }

  return next_state;
}

_spot_elevator_state_t _sesm_lift_up_elevator(spot_elevator_t *elevator)
{
  elevator->nb_spots ++;
 return SESM_WAIT_ELEVATOR_UP;
}

_spot_elevator_state_t _sesm_wait_elevator_up(spot_elevator_t *elevator)
{
  _spot_elevator_state_t next_state = SESM_INACTIVE;
 
  // send ax12 order
  if (elevator->nb_spots < 4) 
    _set_elevator_ax12(elevator, ELEVATOR_UP, ELEVATOR_FAST);
  else
    _set_elevator_ax12(elevator, ELEVATOR_UP_FIFTH_SPOT, ELEVATOR_FAST);

  
  if( _is_elevator_ax12_in_position(elevator, ELEVATOR_UP))
  {
    //next_state = SESM_CHECK_SPOT_PRESENCE;
    next_state = SESM_INACTIVE;
  }
  else
  {
    // continue in this step
    next_state = SESM_WAIT_ELEVATOR_UP;
  }


  return next_state;
}

_spot_elevator_state_t _sesm_close_claw_for_bulb(spot_elevator_t *elevator)
{
  _set_claw_ax12(elevator, CLAW_CLOSED_FOR_BULB);
  return SESM_WAIT_CLAW_CLOSED_FOR_BULB; 
}

_spot_elevator_state_t _sesm_wait_claw_closed_for_bulb(spot_elevator_t *elevator)
{
  _spot_elevator_state_t next_state = SESM_INACTIVE;
  
  if (_is_claw_ax12_in_position(elevator, CLAW_CLOSED_FOR_BULB))
  {
    next_state = SESM_LIFT_UP_ELEVATOR_FOR_BULB;
  }
  else
  {
    next_state = SESM_CLOSE_CLAW_FOR_BULB;
  }
  return next_state;
}

_spot_elevator_state_t _sesm_lift_up_elevator_for_bulb(spot_elevator_t *elevator)
{
  _set_elevator_ax12(elevator, ELEVATOR_UP, ELEVATOR_FAST);
  return SESM_WAIT_ELEVATOR_UP_FOR_BULB;
}

_spot_elevator_state_t _sesm_wait_elevator_up_for_bulb(spot_elevator_t *elevator)
{
  _spot_elevator_state_t next_state = SESM_INACTIVE;

  if ( _is_elevator_ax12_in_position(elevator, ELEVATOR_UP))
  {
    // elevator up => ready to check color
    next_state = SESM_INACTIVE;
  }
  else
  {
    // elevator not ready => stay in this state
    next_state = SESM_LIFT_UP_ELEVATOR_FOR_BULB;
  }
  return next_state;
}

_spot_elevator_state_t _sesm_discharge_elevator_down(spot_elevator_t *elevator)
{
  _set_elevator_ax12(elevator, ELEVATOR_DOWN_WAITING_SPOT, ELEVATOR_SLOW);
  return SESM_DISCHARGE_ELEVATOR_DOWN_WAIT;
}

_spot_elevator_state_t _sesm_discharge_elevator_down_wait(spot_elevator_t *elevator)
{
  if ( _is_elevator_ax12_in_position(elevator, ELEVATOR_DOWN_WAITING_SPOT))
    return SESM_DISCHARGE_CLAW_OPEN;
  else
    return SESM_DISCHARGE_ELEVATOR_DOWN;
}

_spot_elevator_state_t _sesm_discharge_claw_open(spot_elevator_t *elevator)
{
  elevator->nb_spots = 0;
  _set_claw_ax12(elevator, CLAW_OPENED);
  return SESM_DISCHARGE_CLAW_OPEN_WAIT;
}

_spot_elevator_state_t _sesm_discharge_claw_open_wait(spot_elevator_t *elevator)
{
  if ( _is_claw_ax12_in_position(elevator, CLAW_OPENED))
    return SESM_DISCHARGE_ELEVATOR_UP;
  else
    return SESM_DISCHARGE_CLAW_OPEN;
}

_spot_elevator_state_t _sesm_discharge_elevator_up(spot_elevator_t *elevator)
{
  _set_elevator_ax12(elevator, ELEVATOR_UP, ELEVATOR_FAST);
  return SESM_DISCHARGE_ELEVATOR_UP_WAIT;
}

_spot_elevator_state_t _sesm_discharge_elevator_up_wait(spot_elevator_t *elevator)
{
  if ( _is_elevator_ax12_in_position(elevator, ELEVATOR_UP))
    return SESM_INACTIVE;
  else
    return SESM_DISCHARGE_ELEVATOR_UP;
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
    elevator->tm_state = SESM_TM_S_READY;
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
    static int lstate = INT16_MAX;
    if(lstate != elevator->sm_state) {
      ROME_LOGF(&rome_strat, DEBUG,"state %u", elevator->sm_state);
    }
    lstate = elevator->sm_state;
    // each state uses a non blocking function that returns the next state of the state machine
    switch(elevator->sm_state)
    {
      case SESM_INIT:
        elevator->sm_state = _sesm_init(elevator);
        break;

      case SESM_INACTIVE:
        elevator->tm_state = SESM_TM_S_READY;
        elevator->sm_state = _sesm_inactive(elevator);
        break;

      case SESM_PREPARE_CLAW_FOR_ONBOARD_BULB:
        elevator->tm_state = SESM_TM_S_GROUND_CLEAR;
        elevator->sm_state = _sesm_prepare_claw_for_onboard_buld(elevator);
        break;

      case SESM_OPEN_CLAW_FOR_BULB:
        elevator->tm_state = SESM_TM_S_GROUND_CLEAR;
        elevator->sm_state = _sesm_open_claw_for_bulb(elevator);
        break;

      case SESM_CHECK_SPOT_PRESENCE:
        elevator->tm_state = SESM_TM_S_BUSY;
        elevator->sm_state = _sesm_check_spot_presence(elevator);
        break;

      case  SESM_CHECK_SPOT_COLOR:
        elevator->tm_state = SESM_TM_S_BUSY;
        elevator->sm_state = _sesm_check_spot_color(elevator);
        break;

      case SESM_CHECK_ELEVATOR_POSITION:
        elevator->tm_state = SESM_TM_S_BUSY;
        elevator->sm_state = _sesm_check_elevator_position(elevator);
        break;

      case  SESM_CHECK_CLAW_OPENED:
        elevator->tm_state = SESM_TM_S_BUSY;
        elevator->sm_state = _sesm_check_claw_opened(elevator);
        break;

      case SESM_OPEN_CLAW: 
        elevator->tm_state = SESM_TM_S_BUSY;
        elevator->sm_state = _sesm_open_claw(elevator);
        break;

      case SESM_WAIT_CLAW_OPENED:
        elevator->tm_state = SESM_TM_S_BUSY;
        elevator->sm_state = _sesm_wait_claw_opened(elevator);
        break;

      case SESM_LIFT_DOWN_ELEVATOR:
        elevator->tm_state = SESM_TM_S_BUSY;
        elevator->sm_state = _sesm_lift_down_elevator(elevator);
        break;

      case SESM_WAIT_ELEVATOR_DOWN:
        elevator->tm_state = SESM_TM_S_BUSY;
        elevator->sm_state = _sesm_wait_elevator_down(elevator);
        break;

      case SESM_CLOSE_CLAW:
        elevator->tm_state = SESM_TM_S_BUSY;
        elevator->sm_state = _sesm_close_claw(elevator);
        break;

      case SESM_WAIT_CLAW_CLOSED:
        elevator->tm_state = SESM_TM_S_BUSY;
        elevator->sm_state = _sesm_wait_claw_closed(elevator);
        break;

      case SESM_LIFT_UP_ELEVATOR:
        elevator->tm_state = SESM_TM_S_BUSY;
        elevator->sm_state = _sesm_lift_up_elevator(elevator);
        break;

      case SESM_WAIT_ELEVATOR_UP:
        elevator->tm_state = SESM_TM_S_BUSY;
        elevator->sm_state = _sesm_wait_elevator_up(elevator);
        break;

      case SESM_CLOSE_CLAW_FOR_BULB:
        elevator->tm_state = SESM_TM_S_BUSY;
        elevator->sm_state = _sesm_close_claw_for_bulb(elevator);
        break;

      case SESM_WAIT_CLAW_CLOSED_FOR_BULB:
        elevator->tm_state = SESM_TM_S_BUSY;
        elevator->sm_state = _sesm_wait_claw_closed_for_bulb(elevator);
        break;

      case SESM_LIFT_UP_ELEVATOR_FOR_BULB:
        elevator->tm_state = SESM_TM_S_BUSY;
        elevator->sm_state = _sesm_lift_up_elevator_for_bulb(elevator);
        break;

      case SESM_WAIT_ELEVATOR_UP_FOR_BULB:
        elevator->tm_state = SESM_TM_S_GROUND_CLEAR;
        elevator->sm_state = _sesm_wait_elevator_up_for_bulb(elevator);
        break;

      case SESM_DISCHARGE_ELEVATOR_DOWN:
        elevator->tm_state = SESM_TM_S_BUSY;
        elevator->sm_state = _sesm_discharge_elevator_down(elevator);
        // no break
      case SESM_DISCHARGE_ELEVATOR_DOWN_WAIT:
        elevator->tm_state = SESM_TM_S_BUSY;
        elevator->sm_state = _sesm_discharge_elevator_down_wait(elevator);
        break;

      case SESM_DISCHARGE_CLAW_OPEN:
        elevator->tm_state = SESM_TM_S_BUSY;
        elevator->sm_state = _sesm_discharge_claw_open(elevator);
        // no break
      case SESM_DISCHARGE_CLAW_OPEN_WAIT:
        elevator->tm_state = SESM_TM_S_BUSY;
        elevator->sm_state = _sesm_discharge_claw_open_wait(elevator);
        break;

      case SESM_DISCHARGE_ELEVATOR_UP:
        elevator->tm_state = SESM_TM_S_GROUND_CLEAR;
        elevator->sm_state = _sesm_discharge_elevator_up(elevator);
        // no break
      case SESM_DISCHARGE_ELEVATOR_UP_WAIT:
        elevator->tm_state = SESM_TM_S_GROUND_CLEAR;
        elevator->sm_state = _sesm_discharge_elevator_up_wait(elevator);
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
  se->tm_state = SESM_TM_S_BUSY;
  se->is_active = enable;
}

void spot_elevator_prepare_for_bulb_picking(spot_elevator_t *se)
{
  se->tm_state = SESM_TM_S_BUSY;
  se->sm_state = SESM_OPEN_CLAW_FOR_BULB;
}

void spot_elevator_prepare_for_onboard_bulb(spot_elevator_t *se)
{
  se->tm_state = SESM_TM_S_BUSY;
  se->sm_state = SESM_PREPARE_CLAW_FOR_ONBOARD_BULB;
}

void spot_elevator_pick_bulb(spot_elevator_t *se)
{
  se->tm_state = SESM_TM_S_BUSY;
  se->sm_state = SESM_CLOSE_CLAW_FOR_BULB;
}

void spot_elevator_automatic_spot_stacking(spot_elevator_t *se)
{
  se->tm_state = SESM_TM_S_BUSY;
  se->sm_state = SESM_CHECK_SPOT_PRESENCE;
}

void spot_elevator_discharge_spot_stack(spot_elevator_t *se)
{
  se->tm_state = SESM_TM_S_BUSY;
  se->sm_state = SESM_DISCHARGE_ELEVATOR_DOWN;
}

