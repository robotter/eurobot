#ifndef CONFIG_H
#define CONFIG_H

/*
 * TCD0: balloon servomotor
 * TCF0: scheduling (timer module)
 */


#define MATCH_DURATION_SECS (89)

/// Perlimpinpin node address
#define PPP_ADDR  0x12

#define UART_JEVOIS uartF1
#define UART_AX12  uartC1
#define UART_PPP  uartF0
#define UART_COLOR_SENSOR_LEFT   uartF1
#define UART_COLOR_SENSOR_RIGHT    uartD1

// Tick period of match timer, in microseconds
#define UPDATE_MATCH_TIMER_TICK_US  10000
// Interrupt level of tick interrupt
#define MATCH_TIMER_INTLVL  INTLVL_HI

// rome update in us
#define UPDATE_ROME_US 10000
#define UPDATE_MSG_US  100000

// AX-12 recv timeout, in microseconds
#define AX12_TIMEOUT_US  40000

// pinout

#define LED_AN_PP(n)  PORTPIN(A,n)
#define LED_RUN_PP  PORTPIN(K,0)
#define LED_ERROR_PP  PORTPIN(K,1)
#define LED_COM_PP  PORTPIN(K,2)

#define AX12_DIR_PP  PORTPIN(C,5)

#define CAKE_ENCODER_CS_PP  PORTPIN(E,4)

#define MOTOR0_TC  TCD1
#define MOTOR1_TC  TCD1
#define MOTOR2_TC  TCE0
#define MOTOR3_TC  TCE0
#define MOTOR0_TC_CH  'B'
#define MOTOR1_TC_CH  'D'
#define MOTOR2_TC_CH  'B'
#define MOTOR3_TC_CH  'D'
#define MOTOR0_SIGN_PP  PORTPIN(D,4)
#define MOTOR1_SIGN_PP  PORTPIN(D,6)
#define MOTOR2_SIGN_PP  PORTPIN(E,0)
#define MOTOR3_SIGN_PP  PORTPIN(E,2)

// analog servos
#define SERVO_ANA0_PP  PORTPIN(D,0)
#define SERVO_ANA1_PP  PORTPIN(C,4)
#define SERVO_ANA2_PP  PORTPIN(D,1)
#define SERVO_ANA3_PP  PORTPIN(D,2)
#define SERVO_ANA4_PP  PORTPIN(D,3)
// digital servos
#define SERVO_DIG0_PP  PORTPIN(B,0)
#define SERVO_DIG1_PP  PORTPIN(B,1)
#define SERVO_DIG2_PP  PORTPIN(B,2)
#define SERVO_DIG3_PP  PORTPIN(B,3)
#define SERVO_DIG4_PP  PORTPIN(C,0)
#define SERVO_DIG5_PP  PORTPIN(C,1)
#define SERVO_DIG6_PP  PORTPIN(C,2)
#define SERVO_DIG7_PP  PORTPIN(C,3)
// servo aliases, using board names
#define SERVO8_PP  SERVO_ANA0_PP
#define SERVO7_PP  SERVO_ANA1_PP
#define SERVO6_PP  SERVO_ANA2_PP
#define SERVO1_PP  SERVO_ANA3_PP
#define SERVO2_PP  SERVO_ANA4_PP
#define SERVO3_PP  SERVO_DIG0_PP
#define SERVO4_PP  SERVO_DIG1_PP
#define SERVO9_PP  SERVO_DIG2_PP
#define SERVO10_PP  SERVO_DIG3_PP
#define SERVO11_PP  SERVO_DIG4_PP
#define SERVO12_PP  SERVO_DIG5_PP
#define SERVO13_PP  SERVO_DIG6_PP
#define SERVO14_PP  SERVO_DIG7_PP

//Cylinder settings

//choose servo type, AX or MX
#define USE_AX12

//AX12 settings
#ifdef USE_AX12

#define CYLINDER_AX12_ID 5
#define CYLINDER_BALLEATER_POS { 6, 142, 288, 435, 578, 721, 863, 1002 }
#define CYLINDER_TURBINE_POS_OFFSET 830
#define CYLINDER_TURBINE_POS { 836, 972, 720, 22, 159, 311, 454, 596 }
#define CYLINDER_WHEELMODE_SPEED 400
#define CYLINDER_AX12_ARRIVED_WINDOW 6

#endif


//MX12 settings
#ifdef USE_MX12
#define CYLINDER_MX12_ID 1

#define CYLINDER_BALLEATER_POS_MIN 665
#define CYLINDER_BALLEATER_POS_MAX 3960
#define CYLINDER_TURBINE_POS_OFFSET 2746

#endif

#define CYLINDER_NB_POS 8

#define CYLINDER_MOVING_DELAY_US         300000
#define CYLINDER_BALL_DROP_DELAY_US      500000
#define CYLINDER_TURBINE_STOP_DELAY_US   700000
#define CYLINDER_BALL_FLYING_DELAY_US    1500000
#define CYLINDER_TURBINE_BOOT_DELAY_US   2000000

#define CYLINDER_EATING_TIMEOUT_US       1500000

#endif
