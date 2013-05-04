#ifndef ACM_CONFIG_H
#define ACM_CONFIG_H
/// ACM_config : Automatic Cake Management config file


#include "acm_defs.h"

#define SECOND_LVL_AX12_ID        2
#define FIRST_LVL_LEFT_AX12_ID    3
#define FIRST_LVL_RIGHT_AX12_ID   4



/// default positions of each AX12 at startup of board (in ax12 units)
#define DEFAULT_ACM_SECOND_LVL_HOME_POS          200 
#define DEFAULT_ACM_SECOND_LVL_BLOW_CANDLE_POS   490
#define DEFAULT_ACM_SECOND_LVL_MAX_OPEN_POS_BLUE 700
#define DEFAULT_ACM_SECOND_LVL_MAX_OPEN_POS_RED  320

#define DEFAULT_ACM_FIRST_LVL_LEFT_HOME_POS         540
#define DEFAULT_ACM_FIRST_LVL_LEFT_BLOW_CANDLE_POS  810
#define DEFAULT_ACM_FIRST_LVL_LEFT_AVOID_CANDLE_POS 710

#define DEFAULT_ACM_FIRST_LVL_RIGHT_HOME_POS         490
#define DEFAULT_ACM_FIRST_LVL_RIGHT_BLOW_CANDLE_POS  200
#define DEFAULT_ACM_FIRST_LVL_RIGHT_AVOID_CANDLE_POS 300


// distances of arm to encoder
#define DEFAULT_ACM_FIRST_LVL_LEFT_ARM_ENC_OFFSET_MM    100.0
#define DEFAULT_ACM_FIRST_LVL_RIGHT_ARM_ENC_OFFSET_MM   100.0
#define DEFAULT_ACM_ENCODER_TO_SIDE_ENC_OFFSET_MM       122.3

// distance used to rise the arm if the next candle doesn't need to be blown
#define DEFAULT_ACM_CANDLE_ANTICIPATION_OFFSET_MM    40.0
#define ACM_ANTICIPATION_ACTIVATION_THRESHOLD   20 // in encoder units


// environnement dimension
#define DEFAULT_ACM_CAKE_RADIUS_MM            500.0
#define DEFAULT_ACM_CAKE_ENCODER_RADIUS_MM    16.014

#define DEFAULT_ACM_AX12_END_OF_MOVE_MARGIN   20

// pwm value sent to the motor when "caking"
#define DEFAULT_ACM_MOTOR_PWM_ON_CAKE     32000


/// robot position and color must be define using color (XXX ) 
#define DEFAULT_ACM_ROBOT_COLOR  ACM_RED
#define DEFAULT_ACM_ROBOT_STALL_POSITION  ACM_RED



#define DEFAULT_ACM_QUALIFICATION_ROUNDS true

#endif //ACM_CONFIG_H
