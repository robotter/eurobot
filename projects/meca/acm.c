#include "acm_config.h"
#include "acm.h"
#include <ax12/ax12.h>
#include <stdio.h>
#include <math.h>

static bool acm_arm_in_position(acm_t *s, acm_arm_t arm, acm_arm_config_t conf)
{
  uint16_t goal, pos = 0xffff;

  switch(arm)
  {
    case ACM_ARM_SECOND_LVL : 
      switch(conf)
      {
        case ACM_ARM_HOMED : 
          goal = s->second_lvl_home_pos;
          break;
        case ACM_ARM_STALLING:  // no break : same than ACM_ARM_ON_CAKE_AVOID_CANDLE
        case ACM_ARM_ON_CAKE_AVOID_CANDLE :
          if (s->cake_stall_side == ACM_BLUE)
          {
            goal = s->second_lvl_max_open_pos_blue;
          }
          else
          {
            goal = s->second_lvl_max_open_pos_red;
          }
          break;

        case ACM_ARM_ON_CAKE:
        case ACM_ARM_ON_CAKE_BLOW_CANDLE:
          goal = s->second_lvl_blow_candle_pos;
          break;
        default : return true;
      }
      ax12_read_word(s->ax12, s->second_lvl_ax12_id, AX12_ADDR_PRESENT_POSITION_L, &pos);
      break;

    case ACM_ARM_FIRST_LVL_LEFT : 
      switch(conf)
      {
        case ACM_ARM_HOMED : 
          goal = s->first_lvl_left_home_pos;
          break;
        case ACM_ARM_STALLING:  // no break : same than ACM_ARM_ON_CAKE_AVOID_CANDLE
        case ACM_ARM_ON_CAKE_AVOID_CANDLE :
          goal = s->first_lvl_left_avoid_candle_pos;
          break;

        case ACM_ARM_ON_CAKE:
          if (s->cake_stall_side == ACM_BLUE)
          {
            goal = s->first_lvl_left_blow_candle_pos;
          }
          else
          {
            goal = s->first_lvl_left_avoid_candle_pos;
          }
          break;

        case ACM_ARM_ON_CAKE_BLOW_CANDLE:
          goal = s->first_lvl_left_blow_candle_pos;
          break;
        default : return true;
      }
      ax12_read_word(s->ax12, s->first_lvl_left_ax12_id, AX12_ADDR_PRESENT_POSITION_L, &pos);
      break; 

    case ACM_ARM_FIRST_LVL_RIGHT : 
      switch(conf)
      {
        case ACM_ARM_HOMED : 
          goal = s->first_lvl_right_home_pos;
          break;
        case ACM_ARM_STALLING:  // no break : same than ACM_ARM_ON_CAKE_AVOID_CANDLE
        case ACM_ARM_ON_CAKE_AVOID_CANDLE :
          goal = s->first_lvl_right_avoid_candle_pos;
          break;

        case ACM_ARM_ON_CAKE:
          if (s->cake_stall_side == ACM_BLUE)
          {
            goal = s->first_lvl_right_avoid_candle_pos;
          }
          else
          {
            goal = s->first_lvl_right_blow_candle_pos;
          }
          break;


        case ACM_ARM_ON_CAKE_BLOW_CANDLE:
          goal = s->first_lvl_right_blow_candle_pos;
          break;
        default : return true;
      }
      ax12_read_word(s->ax12, s->first_lvl_right_ax12_id, AX12_ADDR_PRESENT_POSITION_L, &pos);
      break; 
    default : return true; 
  }


  if ( ( (goal >= pos) && ((goal - pos) <= s->ax12_end_of_move_margin ))
       ||
       ((goal < pos) && ((pos-goal) <= s->ax12_end_of_move_margin ) ))
  {
    return true;
  }
  else
  {
    return false;
  }
}

static void acm_update_motor(acm_t *s, acm_arm_t arm, acm_arm_config_t conf)
{
  switch(arm)
  {
    case ACM_ARM_SECOND_LVL:
      switch(conf)
      {
        case ACM_ARM_HOMED : 
        case ACM_ARM_STALLING:  
          (*s->set_second_lvl_motor_pwm)(0);
          break;

        case ACM_ARM_ON_CAKE_AVOID_CANDLE :
        case ACM_ARM_ON_CAKE:
        case ACM_ARM_ON_CAKE_BLOW_CANDLE : 
          if (s->cake_stall_side == ACM_BLUE)
          {
            (*s->set_second_lvl_motor_pwm)(-s->motor_pwm_on_cake);
          }
          else
          {
            (*s->set_second_lvl_motor_pwm)(s->motor_pwm_on_cake);
          }
          break;

        default: return;
      }
      break;

    case ACM_ARM_FIRST_LVL_LEFT:
      switch(conf)
      {
        case ACM_ARM_HOMED : 
        case ACM_ARM_STALLING:  
          (*s->set_first_lvl_left_motor_pwm)(0);
          break;

        case ACM_ARM_ON_CAKE_AVOID_CANDLE :
        case ACM_ARM_ON_CAKE:
        case ACM_ARM_ON_CAKE_BLOW_CANDLE : 
          if (s->cake_stall_side == ACM_BLUE)
          {
            (*s->set_first_lvl_left_motor_pwm)(-s->motor_pwm_on_cake);
          }
          else
          {
            (*s->set_first_lvl_left_motor_pwm)(s->motor_pwm_on_cake);
          }
          break;

        default: return;
      }
      break;

    case ACM_ARM_FIRST_LVL_RIGHT:
      switch(conf)
      {
        case ACM_ARM_HOMED : 
        case ACM_ARM_STALLING:  
          (*s->set_first_lvl_right_motor_pwm)(0);
          break;

        case ACM_ARM_ON_CAKE_AVOID_CANDLE :
        case ACM_ARM_ON_CAKE:
        case ACM_ARM_ON_CAKE_BLOW_CANDLE : 
          if (s->cake_stall_side == ACM_BLUE)
          {
            (*s->set_first_lvl_right_motor_pwm)(-s->motor_pwm_on_cake);
          }
          else
          {
            (*s->set_first_lvl_right_motor_pwm)(s->motor_pwm_on_cake);
          }
          break;

        default: return;
      }
      break;

    default : return;
  } 
}

static void acm_move_arm(acm_t *s, acm_arm_t arm, acm_arm_config_t conf)
{
  switch(arm)
  {
    case ACM_ARM_SECOND_LVL:
      switch(conf)
      {
        case ACM_ARM_HOMED : 
          ax12_write_word(s->ax12, s->second_lvl_ax12_id, AX12_ADDR_GOAL_POSITION_L, s->second_lvl_home_pos);
          break;

        case ACM_ARM_ON_CAKE:// no break : same than ACM_ARM_ON_CAKE_BLOW_CANDLE
        case ACM_ARM_STALLING:
        case ACM_ARM_ON_CAKE_AVOID_CANDLE :
          if (s->cake_stall_side == ACM_BLUE)
          {
            ax12_write_word(s->ax12, s->second_lvl_ax12_id, AX12_ADDR_GOAL_POSITION_L, s->second_lvl_max_open_pos_blue);
          }
          else
          {
            ax12_write_word(s->ax12, s->second_lvl_ax12_id, AX12_ADDR_GOAL_POSITION_L, s->second_lvl_max_open_pos_red);
          }
          break;
        
        case ACM_ARM_ON_CAKE_BLOW_CANDLE : 
          ax12_write_word(s->ax12, s->second_lvl_ax12_id, AX12_ADDR_GOAL_POSITION_L, s->second_lvl_blow_candle_pos);
          break;

        default: return;
      }
      break;

    case ACM_ARM_FIRST_LVL_LEFT:
      switch(conf)
      {
        case ACM_ARM_HOMED : 
          ax12_write_word(s->ax12, s->first_lvl_left_ax12_id, AX12_ADDR_GOAL_POSITION_L, s->first_lvl_left_home_pos);
          break;

        case ACM_ARM_ON_CAKE:
          if (s->cake_stall_side == ACM_BLUE)
          {
            ax12_write_word(s->ax12, s->first_lvl_left_ax12_id, AX12_ADDR_GOAL_POSITION_L, s->first_lvl_left_blow_candle_pos);
          }
          else
          {
            ax12_write_word(s->ax12, s->first_lvl_left_ax12_id, AX12_ADDR_GOAL_POSITION_L, s->first_lvl_left_avoid_candle_pos);
          }
          break;

        case ACM_ARM_STALLING:
        case ACM_ARM_ON_CAKE_AVOID_CANDLE :
          ax12_write_word(s->ax12, s->first_lvl_left_ax12_id, AX12_ADDR_GOAL_POSITION_L, s->first_lvl_left_avoid_candle_pos);
          break;

        case ACM_ARM_ON_CAKE_BLOW_CANDLE : 
          ax12_write_word(s->ax12, s->first_lvl_left_ax12_id, AX12_ADDR_GOAL_POSITION_L, s->first_lvl_left_blow_candle_pos);
          break;

        default: return;
      }
      break;

    case ACM_ARM_FIRST_LVL_RIGHT:
      switch(conf)
      {
        case ACM_ARM_HOMED : 
          ax12_write_word(s->ax12, s->first_lvl_right_ax12_id, AX12_ADDR_GOAL_POSITION_L, s->first_lvl_right_home_pos);
          break;

        case ACM_ARM_ON_CAKE:
          if (s->cake_stall_side == ACM_BLUE)
          {
            ax12_write_word(s->ax12, s->first_lvl_right_ax12_id, AX12_ADDR_GOAL_POSITION_L, s->first_lvl_right_avoid_candle_pos);
          }
          else
          {
            ax12_write_word(s->ax12, s->first_lvl_right_ax12_id, AX12_ADDR_GOAL_POSITION_L, s->first_lvl_right_blow_candle_pos);
          }
          break;


        case ACM_ARM_STALLING:  // no break : same than ACM_ARM_ON_CAKE_AVOID_CANDLE
     case ACM_ARM_ON_CAKE_AVOID_CANDLE :
          ax12_write_word(s->ax12, s->first_lvl_right_ax12_id, AX12_ADDR_GOAL_POSITION_L, s->first_lvl_right_avoid_candle_pos);
          break;

        case ACM_ARM_ON_CAKE_BLOW_CANDLE : 
          ax12_write_word(s->ax12, s->first_lvl_right_ax12_id, AX12_ADDR_GOAL_POSITION_L, s->first_lvl_right_blow_candle_pos);
          break;

        default: return;
      }
      break;

    default : return;
  }
  acm_update_motor(s, arm, conf);
}

void acm_update_first_lvl_arm_position(acm_t *s)
{
  int32_t enc_pos = aeat_get_value(s->encoder);

  double candle_anticipation_offset_mm;

if ( ( ((enc_pos - s->previous_enc_value) >0) && ((enc_pos - s->previous_enc_value) > ACM_ANTICIPATION_ACTIVATION_THRESHOLD)) 
  ||  ( ((enc_pos - s->previous_enc_value) <0) && ((enc_pos - s->previous_enc_value) < ACM_ANTICIPATION_ACTIVATION_THRESHOLD)) )
{
  if (enc_pos > s->previous_enc_value)
  {
    candle_anticipation_offset_mm = s->candle_anticipation_offset_mm;
  }
  else
  {
    candle_anticipation_offset_mm = -s->candle_anticipation_offset_mm;
  }
  s->previous_enc_value  =enc_pos;
}
else
{
  candle_anticipation_offset_mm = 0.0;
}
 // convert encoder to position in mm
  double cake_pos_mm = fabs(((double)enc_pos * 2 * M_PI * s->encoder_wheel_radius)/(double)(_BV(ACM_CAKE_ENCODER_RESOLUTION)-1)) + s->encoder_to_side_enc_offset_mm;

  double cake_perimeter_mm = s->cake_radius * M_PI;

  double first_lvl_left_arm_enc_offset_mm, first_lvl_right_arm_enc_offset_mm;
  if (s->cake_stall_side == ACM_BLUE)
  {
    first_lvl_left_arm_enc_offset_mm = s->first_lvl_left_arm_enc_offset_mm;
    first_lvl_right_arm_enc_offset_mm = s->first_lvl_left_arm_enc_offset_mm;
  }
  else
  {
    first_lvl_left_arm_enc_offset_mm = -s->first_lvl_left_arm_enc_offset_mm;
    first_lvl_right_arm_enc_offset_mm = -s->first_lvl_left_arm_enc_offset_mm;
  }


  // get ax12 candle position for each arm
  uint8_t left_arm_candle_id = (uint8_t) ((cake_pos_mm - first_lvl_left_arm_enc_offset_mm) / (cake_perimeter_mm / FIRST_LVL_CANDLE_NB));

  uint8_t right_arm_candle_id = (uint8_t) ((cake_pos_mm + first_lvl_right_arm_enc_offset_mm)/(cake_perimeter_mm/FIRST_LVL_CANDLE_NB));

  double tmp = cake_pos_mm;
 // printf("enc %li\tpos %.1f\tper %.1f\tlid %u\tflid %u\trid %u\t", enc_pos, cake_pos_mm, cake_perimeter_mm, left_arm_candle_id, left_arm_candle_id_anticipated, right_arm_candle_id);
  
  // add offset to check what next candle will be
  cake_pos_mm += candle_anticipation_offset_mm;

  uint8_t left_arm_candle_id_anticipated = (uint8_t) ((cake_pos_mm - first_lvl_left_arm_enc_offset_mm) / (cake_perimeter_mm / FIRST_LVL_CANDLE_NB));
  uint8_t right_arm_candle_id_anticipated = (uint8_t) ((cake_pos_mm + first_lvl_right_arm_enc_offset_mm)/(cake_perimeter_mm/FIRST_LVL_CANDLE_NB));

  printf("enc %li\tpos %.1f\tper %.1f\tlid %u\tflid %u\trid %u\t", enc_pos, tmp, cake_perimeter_mm, left_arm_candle_id, left_arm_candle_id_anticipated, right_arm_candle_id);
  
  // get candle color that we want to blow
  acm_candle_color_t candle_target_color;
  if (s->robot_color == ACM_BLUE)
  {
    candle_target_color = ACM_CANDLE_BLUE;
  }
  else
  {
    candle_target_color = ACM_CANDLE_RED;
  }
  

  // update arm position
  if(( (s->candle_color[left_arm_candle_id] == candle_target_color) ||( s->candle_color[left_arm_candle_id] == ACM_CANDLE_WHITE))&& 
 ((s->candle_color[left_arm_candle_id_anticipated] == candle_target_color) || ( s->candle_color[left_arm_candle_id_anticipated] == ACM_CANDLE_WHITE)) )
  {
    acm_move_arm(s, ACM_ARM_FIRST_LVL_LEFT, ACM_ARM_ON_CAKE_BLOW_CANDLE);
  }
  else
  {
    acm_move_arm(s, ACM_ARM_FIRST_LVL_LEFT, ACM_ARM_ON_CAKE_AVOID_CANDLE);
  }

  if ( ( (s->candle_color[right_arm_candle_id] == candle_target_color) ||( s->candle_color[right_arm_candle_id] == ACM_CANDLE_WHITE))&& 
 ((s->candle_color[right_arm_candle_id_anticipated] == candle_target_color) || ( s->candle_color[right_arm_candle_id_anticipated] == ACM_CANDLE_WHITE)) )
  
  {
    acm_move_arm(s, ACM_ARM_FIRST_LVL_RIGHT, ACM_ARM_ON_CAKE_BLOW_CANDLE);
  }
  else
  {
    acm_move_arm(s, ACM_ARM_FIRST_LVL_RIGHT, ACM_ARM_ON_CAKE_AVOID_CANDLE);
  }
}

void acm_init(acm_t *s)
{
  s->robot_color = DEFAULT_ACM_ROBOT_COLOR;
  s->cake_stall_side = DEFAULT_ACM_ROBOT_STALL_POSITION; // robot will search the most proximate stall position
  s->qualification_round = DEFAULT_ACM_QUALIFICATION_ROUNDS;

  s->second_lvl_ax12_id = SECOND_LVL_AX12_ID;
  s->first_lvl_left_ax12_id = FIRST_LVL_LEFT_AX12_ID;
  s-> first_lvl_right_ax12_id = FIRST_LVL_RIGHT_AX12_ID;

  s->second_lvl_home_pos = DEFAULT_ACM_SECOND_LVL_HOME_POS;
  s->second_lvl_blow_candle_pos = DEFAULT_ACM_SECOND_LVL_BLOW_CANDLE_POS;
  s->second_lvl_max_open_pos_blue = DEFAULT_ACM_SECOND_LVL_MAX_OPEN_POS_BLUE;
  s->second_lvl_max_open_pos_red = DEFAULT_ACM_SECOND_LVL_MAX_OPEN_POS_RED;

  s->first_lvl_left_home_pos = DEFAULT_ACM_FIRST_LVL_LEFT_HOME_POS;
  s->first_lvl_left_blow_candle_pos = DEFAULT_ACM_FIRST_LVL_LEFT_BLOW_CANDLE_POS;
  s->first_lvl_left_avoid_candle_pos = DEFAULT_ACM_FIRST_LVL_LEFT_AVOID_CANDLE_POS;

  s->first_lvl_right_home_pos = DEFAULT_ACM_FIRST_LVL_RIGHT_HOME_POS;
  s->first_lvl_right_blow_candle_pos = DEFAULT_ACM_FIRST_LVL_RIGHT_BLOW_CANDLE_POS;
  s->first_lvl_right_avoid_candle_pos = DEFAULT_ACM_FIRST_LVL_RIGHT_AVOID_CANDLE_POS;

  s->first_lvl_left_arm_enc_offset_mm = DEFAULT_ACM_FIRST_LVL_LEFT_ARM_ENC_OFFSET_MM;
  s->first_lvl_right_arm_enc_offset_mm = DEFAULT_ACM_FIRST_LVL_RIGHT_ARM_ENC_OFFSET_MM;
  s->encoder_to_side_enc_offset_mm = DEFAULT_ACM_ENCODER_TO_SIDE_ENC_OFFSET_MM;
  s->candle_anticipation_offset_mm = DEFAULT_ACM_CANDLE_ANTICIPATION_OFFSET_MM;
  
  s->ax12_end_of_move_margin = DEFAULT_ACM_AX12_END_OF_MOVE_MARGIN;
  s->motor_pwm_on_cake = DEFAULT_ACM_MOTOR_PWM_ON_CAKE;

  s->cake_radius = DEFAULT_ACM_CAKE_RADIUS_MM;
  s->encoder_wheel_radius = DEFAULT_ACM_CAKE_ENCODER_RADIUS_MM;

  s->sm_state = ACM_SM_INIT;
  s->arm_config = ACM_ARM_HOMED;

  /// TODO remove this debug
  for (uint8_t it= 0; it < FIRST_LVL_CANDLE_NB; it ++)
  {
    if (it >= (FIRST_LVL_CANDLE_NB/2 -2) && it <= (FIRST_LVL_CANDLE_NB/2 +2-1) && s->qualification_round==true)
    {
      s->candle_color[it] = ACM_CANDLE_WHITE; 
    }
    else
    {
      s->candle_color[it] = ACM_CANDLE_UNKNOWN; 
    }
  }
}



void acm_set_arm_config(acm_t *s, acm_arm_config_t config)
{
  s->arm_config = config;
}

acm_arm_config_t acm_get_arm_config(acm_t *s)
{
  return s->arm_config;
}

void acm_update(acm_t *s)
{

  switch(s->sm_state)
  {
    case ACM_SM_INIT:
      s->sm_state = ACM_SM_HOME_MOVE_FIRST_LVL_RIGHT;
      break;
    case ACM_SM_INACTIVE:
      break;

    case ACM_SM_MAX_OPEN_FIRST_LVL_LEFT:
      acm_move_arm(s, ACM_ARM_FIRST_LVL_LEFT, ACM_ARM_STALLING);
      s->sm_state = ACM_SM_MAX_WAIT_FIRST_LVL_LEFT_OPEN;
      break;
    case ACM_SM_MAX_WAIT_FIRST_LVL_LEFT_OPEN:
      if (acm_arm_in_position(s, ACM_ARM_FIRST_LVL_LEFT, ACM_ARM_STALLING))
      {
        s->sm_state = ACM_SM_MAX_OPEN_SECOND_LVL;
      }
      break;
    case ACM_SM_MAX_OPEN_SECOND_LVL:
      acm_move_arm(s, ACM_ARM_SECOND_LVL, ACM_ARM_STALLING);
      s->sm_state = ACM_SM_MAX_WAIT_SECOND_LVL_OPEN;
      break;
    case ACM_SM_MAX_WAIT_SECOND_LVL_OPEN:
      if (acm_arm_in_position(s, ACM_ARM_SECOND_LVL, ACM_ARM_STALLING))
      {
        s->sm_state = ACM_SM_MAX_OPEN_FIRST_LVL_RIGHT;
      }
      break;
    case ACM_SM_MAX_OPEN_FIRST_LVL_RIGHT:
      acm_move_arm(s, ACM_ARM_FIRST_LVL_RIGHT, ACM_ARM_STALLING);
      s->sm_state = ACM_SM_MAX_WAIT_FIRST_LVL_RIGHT_OPEN;
      break;
    case ACM_SM_MAX_WAIT_FIRST_LVL_RIGHT_OPEN:
      if (acm_arm_in_position(s, ACM_ARM_FIRST_LVL_RIGHT, ACM_ARM_STALLING))
      {
        s->sm_state = ACM_SM_INACTIVE;
      }
      break;

    case ACM_SM_HOME_MOVE_FIRST_LVL_RIGHT:
      acm_move_arm(s, ACM_ARM_FIRST_LVL_RIGHT, ACM_ARM_HOMED);
      s->sm_state = ACM_SM_HOME_WAIT_FIRST_LVL_RIGHT_HOMED;
      break;
    case ACM_SM_HOME_WAIT_FIRST_LVL_RIGHT_HOMED:
      if (acm_arm_in_position(s, ACM_ARM_FIRST_LVL_RIGHT, ACM_ARM_HOMED))
      {  
        s->sm_state = ACM_SM_HOME_MOVE_SECOND_LVL;
      }
      break;
    case ACM_SM_HOME_MOVE_SECOND_LVL:
      acm_move_arm(s, ACM_ARM_SECOND_LVL, ACM_ARM_HOMED);

      s->sm_state = ACM_SM_HOME_WAIT_SECOND_LBL_HOMED;
      break;
    case ACM_SM_HOME_WAIT_SECOND_LBL_HOMED:
      if (acm_arm_in_position(s, ACM_ARM_SECOND_LVL, ACM_ARM_HOMED))
      {
        s->sm_state = ACM_SM_HOME_MOVE_FIRST_LVL_LEFT;
      }
      break;
    case ACM_SM_HOME_MOVE_FIRST_LVL_LEFT:
      acm_move_arm(s, ACM_ARM_FIRST_LVL_LEFT, ACM_ARM_HOMED);
      s->sm_state = ACM_SM_HOME_WAIT_FIRST_LVL_LEFT_HOMED;
      break;
    case ACM_SM_HOME_WAIT_FIRST_LVL_LEFT_HOMED:
      if (acm_arm_in_position(s, ACM_ARM_FIRST_LVL_LEFT, ACM_ARM_HOMED))
      {  
        s->sm_state = ACM_SM_INACTIVE;
      }
      break;  

    case ACM_SM_CAKING_MOVE_SECOND_LVL:
      acm_move_arm(s, ACM_ARM_SECOND_LVL, ACM_ARM_ON_CAKE_BLOW_CANDLE);
      s->sm_state = ACM_SM_CAKING_WAIT_SECOND_LVL_MOVED;
      break;
    case ACM_SM_CAKING_WAIT_SECOND_LVL_MOVED:
      if (acm_arm_in_position(s, ACM_ARM_SECOND_LVL, ACM_ARM_ON_CAKE_BLOW_CANDLE))
      {  
        s->sm_state = ACM_SM_CAKING_MOVE_FIRST_LVL_LEFT;
      }
      break;
    case ACM_SM_CAKING_MOVE_FIRST_LVL_LEFT:
      acm_move_arm(s, ACM_ARM_FIRST_LVL_LEFT, ACM_ARM_ON_CAKE);
      s->sm_state = ACM_SM_CAKING_WAIT_FIRST_LVL_LEFT_MOVED;
      break;
    case  ACM_SM_CAKING_WAIT_FIRST_LVL_LEFT_MOVED:
      if (acm_arm_in_position(s, ACM_ARM_FIRST_LVL_LEFT, ACM_ARM_ON_CAKE))
      {  
        s->sm_state = ACM_SM_CAKING_MOVE_FIRST_LVL_RIGHT;
      }
      break;
   case ACM_SM_CAKING_MOVE_FIRST_LVL_RIGHT:
      acm_move_arm(s, ACM_ARM_FIRST_LVL_RIGHT, ACM_ARM_ON_CAKE);
      s->sm_state = ACM_SM_CAKING_WAIT_FIRST_LVL_RIGHT_MOVED;
      break;
    case  ACM_SM_CAKING_WAIT_FIRST_LVL_RIGHT_MOVED:
      if (acm_arm_in_position(s, ACM_ARM_FIRST_LVL_RIGHT, ACM_ARM_ON_CAKE))
      {  
        s->sm_state = ACM_SM_CAKING_RESET_ENCODER_VALUE;
      }
      break;
    case ACM_SM_CAKING_RESET_ENCODER_VALUE:
        aeat_set_value(s->encoder, 0);
        s->previous_enc_value = 0;
        s->sm_state = ACM_SM_CAKING_WAIT_CANDLES;
      break;
    case ACM_SM_CAKING_WAIT_CANDLES:
        s->sm_state = ACM_SM_CAKING_GET_NEXT_CANDLE_COLOR ;
      break;
    case ACM_SM_CAKING_GET_NEXT_CANDLE_COLOR:
      /// TODO add CMU CAM command to take picture and return color of next candle
        s->sm_state = ACM_SM_CAKING_UPDATE_ARMS;
      break;
    case ACM_SM_CAKING_UPDATE_ARMS:
      acm_update_first_lvl_arm_position(s);
        s->sm_state = ACM_SM_CAKING_WAIT_CANDLES;
      break;
    case ACM_SM_CAKING_UPDATE_CANDLE_COLOR:
      /// TODO add update list of candles
        s->sm_state = ACM_SM_CAKING_WAIT_CANDLES;
      break;

    default : s->sm_state = ACM_SM_INACTIVE;
              break;
  }


  if (s->sm_state != ACM_SM_INACTIVE)
    printf("state %u\n", s->sm_state);
}
