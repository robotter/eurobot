#ifndef CONFIG_H
#define CONFIG_H

/*
 * TCD0: balloon servomotor
 * TCF0: scheduling (timer module)
 */


#define MATCH_DURATION_SECS (89)

/// Perlimpinpin node address
#define PPP_ADDR  0x12

#define UART_AX12  uartC1
#define UART_PPP  uartF0
#define UART_COLOR_SENSOR_RIGHT   uartF1
#define UART_COLOR_SENSOR_LEFT    uartD1

// Tick period of match timer, in microseconds
#define UPDATE_MATCH_TIMER_TICK_US  10000
// Interrupt level of tick interrupt
#define MATCH_TIMER_INTLVL  INTLVL_HI

// arm update in us
#define UPDATE_ARM_US 100000

// AX-12 timeout for state switch, in microseconds
#define AX12_TIMEOUT_US  10000

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

#define SE_LEFT_AX12_CLAW_ID        2
#define SE_LEFT_AX12_ELEVATOR_ID    1

#define SE_RIGHT_AX12_CLAW_ID       3
#define SE_RIGHT_AX12_ELEVATOR_ID   4

#define SE_AX12_MIDDLE_ARM_ID 5

#define SE_LEFT_SPIPE_CLOSED            150
#define SE_LEFT_SPIPE_OPENED            150

#define SE_LEFT_CLAW_OPENED              320
#define SE_LEFT_CLAW_CLOSED_FOR_SPOT     170
#define SE_LEFT_CLAW_CLOSED_FOR_BULB     210

#define SE_LEFT_ELEVATOR_UP              200
#define SE_LEFT_ELEVATOR_DOWN_WAIT_SPOT  750
#define SE_LEFT_ELEVATOR_DOWN_WAIT_BULB  750
#define SE_LEFT_ELEVATOR_UP_FIFTH_SPOT   550

#define SE_RIGHT_SPIPE_CLOSED            150
#define SE_RIGHT_SPIPE_OPENED            150

#define SE_RIGHT_CLAW_OPENED              320
#define SE_RIGHT_CLAW_CLOSED_FOR_SPOT     190
#define SE_RIGHT_CLAW_CLOSED_FOR_BULB     210

#define SE_RIGHT_ELEVATOR_UP              200
#define SE_RIGHT_ELEVATOR_DOWN_WAIT_SPOT  750
#define SE_RIGHT_ELEVATOR_DOWN_WAIT_BULB  750
#define SE_RIGHT_ELEVATOR_UP_MOVE_SPOT   550

#define SE_MIDDLE_ARM_MAX 580
#define SE_MIDDLE_ARM_MIN 130

#define SE_SERVO_LEFT_TUBE    'A'
#define SE_LEFT_TUBE_OPEN_PWM_US    1300
#define SE_LEFT_TUBE_CLOSE_PWM_US   1700

#define SE_SERVO_RIGHT_TUBE   'B'
#define SE_RIGHT_TUBE_OPEN_PWM_US   1100
#define SE_RIGHT_TUBE_CLOSE_PWM_US  1500

#define SE_SERVO_LEFT_CARPET  'C'
#define SE_LEFT_CARPET_LOCK         1500
#define SE_LEFT_CARPET_UNLOCK       1500

#define SE_SERVO_RIGHT_CARPET 'D'
#define SE_RIGHT_CARPET_LOCK        1500
#define SE_RIGHT_CARPET_UNLOCK      1500

#endif
