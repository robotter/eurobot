#ifndef CONFIG_H
#define CONFIG_H

#define MATCH_DURATION_SECS (99)

#define UART_STRAT  uartF0
#define UART_JEVOIS uartF1
#define UART_AX12  uartC1

// Tick period of match timer, in microseconds
#define UPDATE_MATCH_TIMER_TICK_US  10000
// Interrupt level of tick interrupt
#define MATCH_TIMER_INTLVL  INTLVL_HI

// AX-12 recv timeout, in microseconds
#define AX12_TIMEOUT_US  40000

// meca defaut action timeout
#define MECA_TIMEOUT_US 10000000


// general pinout

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

//atom manipulator

//elevators stepper motor control pins
#define LEFT_MOTOR_STEP_PIN       PORTPIN(D,0)
#define LEFT_ARM_MOTOR_DIR_PIN    PORTPIN(D,1)
#define LEFT_ARM_MOTOR_EN_PIN     PORTPIN(D,3)

#define RIGHT_MOTOR_STEP_PIN      PORTPIN(D,2)
#define RIGHT_ARM_MOTOR_DIR_PIN   PORTPIN(B,2)
#define RIGHT_ARM_MOTOR_EN_PIN    PORTPIN(C,4)

//elevator upper side has limit switches
#define LEFT_ELEVATOR_STOP_PIN PORTPIN(B,0)

#define RIGHT_ELEVATOR_STOP_PIN PORTPIN(B,3)


//pumps and valves are controlled by the magichanism's mosboard
//interface is I2C E
#define ARM_I2C_ADDR 0x30
#define ARM_PUMP(x)          ((x) ? 0b00000001 : 0b00000010)
#define ARM_LEFT_VALVE(x)    ((x) ? 0b00000100 : 0b00100000)
#define ARM_CENTER_VALVE(x)  ((x) ? 0b00001000 : 0b01000000)
#define ARM_RIGHT_VALVE(x)   ((x) ? 0b00010000 : 0b10000000)

#define BARO_VOID_PRESSURE 250

#endif
