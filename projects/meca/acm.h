#ifndef ACM_H
#define ACM_H

#include <avarix.h>
#include <ax12/ax12.h>
#include <encoder/aeat/aeat.h>
#include "acm_defs.h"
#include "acm_config.h"
#include "candle_defs.h"
#include "ccom.h"


typedef enum {
  ACM_BLUE = 0,
  ACM_RED,
} acm_color_t;

typedef enum{
  ACM_MODE_HOME,          // arms retracted 
  ACM_MODE_STALLING,      // arms opened and ready to cake 
  ACM_MODE_CAKING,        // arms blowing candles automatically
} acm_arm_mode_t;

typedef struct {

  /// configuration of the robot
  acm_color_t robot_color;    // robot color
  acm_color_t cake_stall_side; // side of the cake where the robot stalled in order to roam the cake
  // If true, ignore color of candles in the center of the first level.
  // Used for qualification rounds.
  bool ignore_colors;

  uint8_t second_lvl_ax12_id;
  uint8_t first_lvl_left_ax12_id;
  uint8_t first_lvl_right_ax12_id;

  uint16_t second_lvl_home_pos;
  uint16_t second_lvl_blow_candle_pos;
  uint16_t second_lvl_blow_last_red_candle;
  uint16_t second_lvl_max_open_pos_blue;
  uint16_t second_lvl_max_open_pos_red;

  uint16_t first_lvl_left_home_pos;
  uint16_t first_lvl_left_blow_candle_pos;
  uint16_t first_lvl_left_avoid_candle_pos;

  uint16_t first_lvl_right_home_pos;
  uint16_t first_lvl_right_blow_candle_pos;
  uint16_t first_lvl_right_avoid_candle_pos;

  double second_lvl_arm_enc_offset_mm; // distance from encoder wheel to second lvl arm ax12
  double second_lvl_arm_red_side_avoid_red_candle_pos_offset_mm; // distance from encoder wheel to second lvl arm ax12
  double second_lvl_arm_blue_side_avoid_blue_candle_pos_offset_mm; // distance from encoder wheel to second lvl arm ax12
  double second_lvl_red_robot_stall_blue_side_blow_last_candle_offset_mm; // threshold to blow last red candle on second lvl cake when red robot is stalled in blue side
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

  /// camera communication
  ccom_t *camera;
  
  double cam_update_candle_window_mm;


  /// acm internal variables
  acm_arm_config_t arm_config; // arm configuration
  acm_sm_state_t sm_state; // state machine state
  acm_candle_color_t candle_color[FIRST_LVL_CANDLE_NB];
  int32_t previous_enc_value; // encoder value at previous call
  uint32_t last_ax12_order_timestamp;

  double cake_pos_mm; /// position on the cake in mm
} acm_t;

void acm_init(acm_t *s);

// arm position accessor (must be called before stalling and at end of stalling and caking)
void acm_set_arm_config(acm_t *s, acm_arm_config_t config);
acm_arm_config_t acm_get_arm_config(const acm_t *s);


// must be called as often as possible
void acm_update(acm_t *s);

void acm_set_stall_side(acm_t *s, acm_color_t color_side);

/// update global mode for arm
void acm_set_arm_mode(acm_t *s, acm_arm_mode_t mode);

#endif //ACM_H

