#ifndef ACM_CONFIG_H
#define ACM_CONFIG_H
/// ACM_config : Automatic Cake Management config file


#include "acm_defs.h"

#define SECOND_LVL_AX12_ID        2
#define FIRST_LVL_LEFT_AX12_ID    3
#define FIRST_LVL_RIGHT_AX12_ID   4



/// default positions of each AX12 at startup of board (in ax12 units)
#define DEFAULT_SECOND_LVL_HOME_POS          200 
#define DEFAULT_SECOND_LVL_BLOW_CANDLE_POS   490
#define DEFAULT_SECOND_LVL_MAX_OPEN_POS      700


#define DEFAULT_FIRST_LVL_LEFT_HOME_POS         540
#define DEFAULT_FIRST_LVL_LEFT_BLOW_CANDLE_POS  810
#define DEFAULT_FIRST_LVL_LEFT_AVOID_CANDLE_POS 710

#define DEFAULT_FIRST_LVL_RIGHT_HOME_POS         490
#define DEFAULT_FIRST_LVL_RIGHT_BLOW_CANDLE_POS  200
#define DEFAULT_FIRST_LVL_RIGHT_AVOID_CANDLE_POS 300


// distances of arm to encoder
#define DEFAULT_FIRST_LVL_LEFT_ARM_ENC_OFFSET_MM    100.0
#define DEFAULT_FIRST_LVL_RIGHT_ARM_ENC_OFFSET_MM   100.0
#define DEFAULT_ENCODER_TO_SIDE_ENC_OFFSET_MM  122.3

#define DEFAULT_CAKE_RADIUS_MM            500.0
#define DEFAULT_CAKE_ENCODER_RADIUS_MM    16.014

#define DEFAULT_AX12_END_OF_MOVE_MARGIN   20

#define DEFAULT_MOTOR_PWM_ON_CAKE     32000


/// robot position and color must be define using color (XXX ) 
#define DEFAULT_ROBOT_COLOR           ACM_BLUE  
#define DEFAULT_ROBOT_STALL_POSITION  ACM_BLUE



#define DEFAULT_QUALIFICATION_ROUNDS

#endif //ACM_CONFIG_H
