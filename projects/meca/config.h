#ifndef CONFIG_H
#define CONFIG_H

/*
 * TCD0: balloon servomotor
 * TCF0: scheduling (timer module)
 */


/// Perlimpinpin node address
#define PPP_ADDR  0x12

#define UART_AX12  uartC1
#define UART_PPP  uartF0
#define UART_CAM  uartF1

// Tick period of uptime counter, in microseconds
#define UPTIME_TICK_US  1000
// Interrupt level of tick interrupt
#define UPTIME_INTLVL  INTLVL_HI

// Cake encoder update period, in microseconds
#define CAKE_ENCODER_UPDATE_PERIOD_US  20000
// Cake encoder update interrupt level
#define CAKE_ENCODER_INTLVL  INTLVL_MED

// AX-12 timeout for state switch, in microseconds
#define AX12_TIMEOUT_US  10000


// Funny action
#define SERVO_BALLOON_TC  TCD0  //TODO
#define SERVO_BALLOON_TC_CH  'A'  //TODO
#define SERVO_BALLOON_OPEN_POS  0  //TODO
#define SERVO_BALLOON_CLOSE_POS  2000


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


#endif
