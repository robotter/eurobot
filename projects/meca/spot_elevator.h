#ifndef SPOT_ELEVATOR_H
#define SPOT_ELEVATOR_H

#include <ax12/ax12.h>
#include <pwm/motor.h>
#include "color_defs.h"
/* 
 * In this module, spipe defines the "spot pipe" used to contain the spots
 */

typedef enum{
  CLAW_OPENED = 0,
  CLAW_CLOSED_FOR_SPOT,
  CLAW_CLOSED_FOR_BULB,

  // CLAW_AX12_POSITIONS_LENGTH must be the last element of this enum
  CLAW_AX12_POSITIONS_LENGTH
} _claw_ax12_positions_t;

typedef enum{
  ELEVATOR_UP = 0,
  ELEVATOR_DOWN_WAITING_SPOT,
  ELEVATOR_DOWN_WAITING_BULB,
  ELEVATOR_UP_FIFTH_SPOT,

  // ELEVATOR_AX12_POSITIONS_LENGTH must be the last element of this enum
  ELEVATOR_AX12_POSITIONS_LENGTH
} _elevator_ax12_positions_t;

typedef uint16_t _elevator_ax12_speeds_t;
static const uint16_t ELEVATOR_SLOW = 0x40;
static const uint16_t ELEVATOR_FAST = 0x1FF;

typedef enum{
  SPIPE_CLOSED = 0,
  SPIPE_OPENED,

  // ELEVATOR_AX12_POSITIONS_LENGTH must be the last element of this enum
  SPIPE_POSITIONS_LENGTH
} _spipe_positions_t;

// states of the state machine that handles the elevator and claw
typedef enum{
  SESM_INIT,
  SESM_INACTIVE,
  // prepare claw to retain bulb boarded during preparation of match
  SESM_PREPARE_CLAW_FOR_ONBOARD_BULB,
  // second init loop : bulb picking (tennis ball)
  SESM_OPEN_CLAW_FOR_BULB,
  
  // first main loop : spot handling
  SESM_CHECK_SPOT_PRESENCE,
  SESM_CHECK_SPOT_COLOR, // 5
  SESM_CHECK_ELEVATOR_POSITION,
  SESM_CHECK_CLAW_OPENED,
  SESM_OPEN_CLAW,
  SESM_WAIT_CLAW_OPENED,
  SESM_LIFT_DOWN_ELEVATOR, // 10
  SESM_WAIT_ELEVATOR_DOWN,
  SESM_CLOSE_CLAW,
  SESM_WAIT_CLAW_CLOSED, // 13
  SESM_LIFT_UP_ELEVATOR,
  SESM_WAIT_ELEVATOR_UP,
  // second main loop : prepare claw for bulb picking
  SESM_CLOSE_CLAW_FOR_BULB,
  SESM_WAIT_CLAW_CLOSED_FOR_BULB,
  SESM_LIFT_UP_ELEVATOR_FOR_BULB,
  SESM_WAIT_ELEVATOR_UP_FOR_BULB,
  // third main loop : discharge spots pile
  SESM_DISCHARGE_ELEVATOR_DOWN,
  SESM_DISCHARGE_ELEVATOR_DOWN_WAIT,
  SESM_DISCHARGE_CLAW_OPEN,
  SESM_DISCHARGE_CLAW_OPEN_WAIT,
  SESM_DISCHARGE_ELEVATOR_UP,
  SESM_DISCHARGE_ELEVATOR_UP_WAIT,

}_spot_elevator_state_t;

typedef enum{
  //elevator won't accept commands and robot shouldn't move
  SESM_TM_S_BUSY,
  //elevator won't accept commands but robot can move
  SESM_TM_S_GROUND_CLEAR,
  //elevator ready for new commands
  SESM_TM_S_READY,
}_spot_elevator_tm_state_t;

typedef struct{
  ax12_addr_t claw_ax12_addr;
  ax12_addr_t elevator_ax12_addr;
  // pwm used to control analog servo that controls the spot pipe opening.
  pwm_motor_t spipe_pwm; 
  
  // table of positions for ax12 used to control claw. Use _claw_ax12_positions_t enum to access to table.
  int16_t claw_ax12_positions[CLAW_AX12_POSITIONS_LENGTH];

  // table of positions for ax12 used to control elevator. Use _elevator_ax12_positions_t enum to access to table.
  int16_t elevator_ax12_positions[ELEVATOR_AX12_POSITIONS_LENGTH];
  
  // table of positions for analog servo that controls the spot pipe (spipe). Use _spipe_positions_t enum to access to table.
  int16_t spipe_pwm_positions[ELEVATOR_AX12_POSITIONS_LENGTH];

  // accessors that return spot presence and color
  bool (*is_spot_present)(void);
  robot_color_t (*get_spot_color)(void);
 
  // state of the state machine handles in spot_elevator_manage
  _spot_elevator_state_t sm_state;

  // indicates if the elevator is active or not
  bool is_active;

  _spot_elevator_tm_state_t tm_state;
  int8_t nb_spots;
}spot_elevator_t;


// initialize spot_elevator_t structure with default data value
void spot_elevator_init(spot_elevator_t *elevator);

// accessors to set addresses of ax12 used to control elevators.
void spot_elevator_set_claw_ax12_addr(spot_elevator_t *elevator, ax12_addr_t addr);
void spot_elevator_set_elevator_ax12_addr(spot_elevator_t *elevator, ax12_addr_t addr);

void spot_elevator_set_is_spot_present_fn(spot_elevator_t *elevator, bool (*is_spot_present)(void));
void spot_elevator_set_get_spot_color_fn(spot_elevator_t *elevator, robot_color_t(*get_spot_color)(void));

// function that must be called periodically to manage the spot elevator
void spot_elevator_manage(spot_elevator_t *elevator);

void spot_elevator_set_enable(spot_elevator_t *se, bool enable);

void spot_elevator_prepare_for_bulb_picking(spot_elevator_t *se);

void spot_elevator_prepare_for_onboard_bulb(spot_elevator_t *se);

void spot_elevator_pick_bulb(spot_elevator_t *se);

void spot_elevator_automatic_spot_stacking(spot_elevator_t *se);

void spot_elevator_discharge_spot_stack(spot_elevator_t *se);

#endif //SPOT_ELEVATOR_H
