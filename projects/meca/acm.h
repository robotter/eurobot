#ifndef ACM_H
#define ACM_H

#include <avarix.h>
#include <ax12/ax12.h>
#include <encoder/aeat/aeat.h>
#include "acm_defs.h"
#include "acm_config.h"

typedef enum{
  ACM_SM_INIT = 0,
  ACM_SM_INACTIVE,
  
  ACM_SM_MAX_OPEN_SECOND_LVL, //2
  ACM_SM_MAX_WAIT_SECOND_LVL_OPEN,
  ACM_SM_MAX_OPEN_FIRST_LVL_LEFT,
  ACM_SM_MAX_WAIT_FIRST_LVL_LEFT_OPEN,
  ACM_SM_MAX_OPEN_FIRST_LVL_RIGHT,
  ACM_SM_MAX_WAIT_FIRST_LVL_RIGHT_OPEN,
  
  ACM_SM_HOME_MOVE_FIRST_LVL_LEFT, //8
  ACM_SM_HOME_WAIT_FIRST_LVL_LEFT_HOMED,
  ACM_SM_HOME_MOVE_FIRST_LVL_RIGHT,
  ACM_SM_HOME_WAIT_FIRST_LVL_RIGHT_HOMED,
  ACM_SM_HOME_MOVE_SECOND_LVL,
  ACM_SM_HOME_WAIT_SECOND_LBL_HOMED,
  
  ACM_SM_CAKING_MOVE_SECOND_LVL, //14
  ACM_SM_CAKING_WAIT_SECOND_LVL_MOVED,
  ACM_SM_CAKING_MOVE_FIRST_LVL_LEFT,
  ACM_SM_CAKING_WAIT_FIRST_LVL_LEFT_MOVED,
  ACM_SM_CAKING_MOVE_FIRST_LVL_RIGHT,
  ACM_SM_CAKING_WAIT_FIRST_LVL_RIGHT_MOVED,
  ACM_SM_CAKING_RESET_ENCODER_VALUE,
  ACM_SM_CAKING_WAIT_CANDLES,
  ACM_SM_CAKING_GET_NEXT_CANDLE_COLOR,
  ACM_SM_CAKING_UPDATE_ARMS,
  ACM_SM_CAKING_UPDATE_CANDLE_COLOR,



} acm_sm_state_t;



typedef struct
{
  /// configuration of the robot
  acm_color_t robot_color;    // robot color
  acm_color_t cake_stall_side; // side of the cake where the robot stalled in order to roam the cake
  bool qualification_round; // boolean that indicate if robot is in qualification rounds (used to know if color needs to be taken into account in central fisrt level candles)

  uint8_t second_lvl_ax12_id;
  uint8_t first_lvl_left_ax12_id;
  uint8_t first_lvl_right_ax12_id;

  uint16_t second_lvl_home_pos;
  uint16_t second_lvl_blow_candle_pos;
  uint16_t second_lvl_max_open_pos_blue;
  uint16_t second_lvl_max_open_pos_red;

  uint16_t first_lvl_left_home_pos;
  uint16_t first_lvl_left_blow_candle_pos;
  uint16_t first_lvl_left_avoid_candle_pos;
 
  uint16_t first_lvl_right_home_pos;
  uint16_t first_lvl_right_blow_candle_pos;
  uint16_t first_lvl_right_avoid_candle_pos;

  double first_lvl_left_arm_enc_offset_mm; // distance from encoder wheel to left arm ax12
  double first_lvl_right_arm_enc_offset_mm; // distance from encoder wheel to right arm ax12
  double encoder_to_side_enc_offset_mm; // distance from encoder to side of the robot (it's centered so same distance to each side) 

  double candle_anticipation_offset_mm; // distance used to rise the arm before the next candle if next candle musn't be blown 

  int16_t motor_pwm_on_cake;
  uint16_t ax12_end_of_move_margin; // margin that will trigger if ax12 move is finished or not

  double cake_radius;
  double encoder_wheel_radius;

  /// ax12 structure used to communicate with AX12 that composes the arm
  ax12_t *ax12;
  
  aeat_t *encoder; // cake encoder used to know position on surface of cake

  void (*set_second_lvl_motor_pwm)(int16_t);
  void (*set_first_lvl_left_motor_pwm)(int16_t);
  void (*set_first_lvl_right_motor_pwm)(int16_t);

  /// acm internal variables
  acm_arm_config_t arm_config; // arm configuration 
  acm_sm_state_t sm_state; // state machine state 
  acm_candle_color_t candle_color[FIRST_LVL_CANDLE_NB];
  int32_t previous_enc_value; // encoder value at previous call

} acm_t;

void acm_init(acm_t *s);

// arm position accessor (must be called before stalling and at end of stalling and caking)
void acm_set_arm_config(acm_t *s, acm_arm_config_t config);
acm_arm_config_t acm_get_arm_config(acm_t *s);


// must be called as often as possible
void acm_update(acm_t *s);

#endif //ACM_H

