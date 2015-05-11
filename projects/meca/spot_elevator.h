#ifndef SPOT_ELEVATOR_H
#define SPOT_ELEVATOR_H

#include <ax12/ax12.h>
#include <pwm/motor.h>
#include <rome/rome_msg.h>
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
  ELEVATOR_DOWN_PUT_BULB,
  ELEVATOR_UP_MOVE_SPOT,

  // ELEVATOR_AX12_POSITIONS_LENGTH must be the last element of this enum
  ELEVATOR_AX12_POSITIONS_LENGTH
} _elevator_ax12_positions_t;

typedef uint16_t _elevator_ax12_speeds_t;
static const uint16_t ELEVATOR_SLOW = 0x40;
static const uint16_t ELEVATOR_FAST = 0x3FF;

typedef enum{
  SPIPE_CLOSED = 0,
  SPIPE_OPENED,

  // ELEVATOR_AX12_POSITIONS_LENGTH must be the last element of this enum
  SPIPE_POSITIONS_LENGTH
} _spipe_positions_t;

// states of the state machine that handles the elevator and claw
typedef enum{
  SESM_INIT = 0,
  SESM_INACTIVE,
  // second init loop : bulb picking (tennis ball)
  SESM_OPEN_CLAW_FOR_BULB,
  
  // pick spot
  SESM_PREPARE_SPOT_INIT = 10,
  SESM_PREPARE_SPOT_LIFT_UP_ELEVATOR,
  SESM_PREPARE_SPOT_WAIT_ELEVATOR_UP,
  SESM_PICK_SPOT_INIT = 15,
  SESM_PICK_SPOT_PRE_LIFT_UP_ELEVATOR,
  SESM_PICK_SPOT_PRE_WAIT_ELEVATOR_UP,
  SESM_PICK_SPOT_CHECK_SPOT_PRESENCE,
  SESM_PICK_SPOT_CHECK_SPOT_COLOR, 
  SESM_PICK_SPOT_LIFT_DOWN_ELEVATOR_PUT_BULB, 
  SESM_PICK_SPOT_LIFT_DOWN_ELEVATOR_PUT_BULB_WAIT,
  SESM_PICK_SPOT_OPEN_CLAW,
  SESM_PICK_SPOT_WAIT_CLAW_OPENED, 
  SESM_PICK_SPOT_LIFT_DOWN_ELEVATOR, 
  SESM_PICK_SPOT_WAIT_ELEVATOR_DOWN,
  SESM_PICK_SPOT_CLOSE_CLAW,
  SESM_PICK_SPOT_WAIT_CLAW_CLOSED, 
  SESM_PICK_SPOT_POST_LIFT_UP_ELEVATOR,
  SESM_PICK_SPOT_POST_WAIT_ELEVATOR_UP,
  // discharge spot pile
  SESM_DISCHARGE_INIT = 30,
  SESM_DISCHARGE_ELEVATOR_DOWN,
  SESM_DISCHARGE_ELEVATOR_DOWN_WAIT,
  SESM_DISCHARGE_CLAW_OPEN,
  SESM_DISCHARGE_CLAW_OPEN_WAIT,
  SESM_DISCHARGE_RELEASE_PILE,
  SESM_DISCHARGE_ELEVATOR_UP,
  SESM_DISCHARGE_ELEVATOR_UP_WAIT,
  // bulbs
  SESM_PICK_BULB_INIT = 50,
  SESM_CLOSE_CLAW_FOR_BULB,
  SESM_WAIT_CLAW_CLOSED_FOR_BULB,
  SESM_LIFT_UP_ELEVATOR_FOR_BULB,
  SESM_WAIT_ELEVATOR_UP_FOR_BULB,
  SESM_PREPARE_CLAW_FOR_BULB_INIT,
  SESM_PREPARE_CLAW_FOR_BULB_CLAW,
  SESM_PREPARE_CLAW_FOR_BULB_CLAW_WAIT,
  SESM_PREPARE_CLAW_FOR_BULB_ELEV,
  SESM_PREPARE_CLAW_FOR_BULB_ELEV_WAIT,
  // cups
  SESM_PICK_CUP_CLAW_OPEN = 70,
  SESM_PICK_CUP_CLAW_OPEN_WAIT,
  SESM_PICK_CUP_ELEVATOR_DOWN,
  SESM_PICK_CUP_ELEVATOR_DOWN_WAIT,
  SESM_PICK_CUP_CHECK_CUP_PRESENCE,
  SESM_PICK_CUP_CLAW_CLOSE,
  SESM_PICK_CUP_CLAW_CLOSE_WAIT,
  SESM_PICK_CUP_ELEVATOR_UP,
  SESM_PICK_CUP_ELEVATOR_UP_WAIT, //35
  // end of match
  SESM_ENDOFMATCH_CLAW_OPEN = 250,
  SESM_ENDOFMATCH_CLAW_OPEN_WAIT,
  SESM_ENDOFMATCH_SPIPE_OPEN,
  SESM_ENDOFMATCH,

}_spot_elevator_state_t;

// elevator states
#define SESM_TM_S(s)  ROME_ENUM_MECA_ELEVATOR_STATE_ ## s

typedef enum{
  SESM_SPIPE_OPEN = 0,
  SESM_SPIPE_CLOSE,
} _spot_elevator_spipe_state_t;

typedef struct{
  ax12_addr_t claw_ax12_addr;
  ax12_addr_t elevator_ax12_addr;

  // pwm used to control analog servo that controls the spot pipe opening.
  pwm_motor_t spipe_servo;
  int16_t spipe_open, spipe_close;
  _spot_elevator_spipe_state_t spipe_state; 
  
  // table of positions for ax12 used to control claw. Use _claw_ax12_positions_t enum to access to table.
  int16_t claw_ax12_positions[CLAW_AX12_POSITIONS_LENGTH];

  // table of positions for ax12 used to control elevator. Use _elevator_ax12_positions_t enum to access to table.
  int16_t elevator_ax12_positions[ELEVATOR_AX12_POSITIONS_LENGTH];
  
  // accessors that return spot presence and color
  bool (*is_spot_present)(void);
  robot_color_t (*get_spot_color)(void);
 
  // state of the state machine handles in spot_elevator_manage
  _spot_elevator_state_t sm_state;

  // indicates if the elevator is active or not
  bool is_active;

  rome_enum_meca_elevator_state_t tm_state;
  int8_t nb_spots;
  
  uint8_t claw_blocked_cnt;
  uint8_t elev_blocked_cnt;

}spot_elevator_t;


// initialize spot_elevator_t structure with default data value
void spot_elevator_init(spot_elevator_t *elevator);

// accessors to set addresses of ax12 used to control elevators.
void spot_elevator_set_claw_ax12_addr(spot_elevator_t *elevator, ax12_addr_t addr);
void spot_elevator_set_elevator_ax12_addr(spot_elevator_t *elevator, ax12_addr_t addr);

void spot_elevator_init_spipe_servo(spot_elevator_t *elevator, char channel, int16_t open_pwm_us, int16_t close_pwm_us);

void spot_elevator_set_is_spot_present_fn(spot_elevator_t *elevator, bool (*is_spot_present)(void));
void spot_elevator_set_get_spot_color_fn(spot_elevator_t *elevator, robot_color_t(*get_spot_color)(void));

// function that must be called periodically to manage the spot elevator
void spot_elevator_manage(spot_elevator_t *elevator);

void spot_elevator_set_enable(spot_elevator_t *se, bool enable);

void spot_elevator_prepare_for_bulb_picking(spot_elevator_t *se);

void spot_elevator_prepare_for_bulb(spot_elevator_t *se);

void spot_elevator_pick_bulb(spot_elevator_t *se);

void spot_elevator_prepare_spot_stacking(spot_elevator_t *se);
void spot_elevator_automatic_spot_stacking(spot_elevator_t *se);

void spot_elevator_discharge_spot_stack(spot_elevator_t *se);
void spot_elevator_release_spot_stack(spot_elevator_t *se);

void spot_elevator_move_middle_arm(uint16_t position);

void spot_elevator_prepare_unload_cup(spot_elevator_t *se,bool block);
void spot_elevator_pick_cup(spot_elevator_t *se);

void spot_elevator_end_of_match(spot_elevator_t *se);
void spot_elevator_reset(spot_elevator_t *se);
#endif //SPOT_ELEVATOR_H
