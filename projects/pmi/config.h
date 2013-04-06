#ifndef CONFIG_H
#define CONFIG_H

/*
 * TCC0 -- scheduler
 *  - channel A: update of motor encoders
 *
 * TCD0 -- motor PWMs
 *
 */

/// Perlimpinpin node address
#define PPP_ADDR  0x14

/// Motor frequency, in Hertz
#define MOTOR_FREQUENCY  10000

/// Control system update period, in microseconds
#define CONTROL_SYSTEM_PERIOD_US  10000
/// Interrupt level for control system update
#define CONTROL_SYSTEM_INTLVL  INTLVL_LO


/// Distance precision margin, in centimers
#define TRAJECTORY_MARGIN_DIST  2
/// Angle precision margin, in degrees
#define TRAJECTORY_MARGIN_ANGLE  5



// pinout

#define LED_GREEN_PP  PORTPIN(Q,1)
#define LED_RED_PP  PORTPIN(Q,2)
#define LED_BLUE_PP  PORTPIN(Q,3)

#define STARTING_CORD_PP  PORTPIN(K,6)
#define ROBOT_COLOR_PP  PORTPIN(K,7)

#define AX12_DIR_PP  PORTPIN(D,4)

#define MOTOR_LEFT_ENABLE_A_PP  PORTPIN(H,4)
#define MOTOR_LEFT_ENABLE_B_PP  PORTPIN(H,5)
#define MOTOR_LEFT_ROTATE_A_PP  PORTPIN(H,0)
#define MOTOR_LEFT_ROTATE_B_PP  PORTPIN(H,1)
#define MOTOR_LEFT_ENCODER_CS_PP  PORTPIN(E,1)
#define MOTOR_LEFT_PWM_PP  PORTPIN(E,2)
#define MOTOR_LEFT_PWM_TC_CH  'C'

#define MOTOR_RIGHT_ENABLE_A_PP  PORTPIN(H,7)
#define MOTOR_RIGHT_ENABLE_B_PP  PORTPIN(H,6)
#define MOTOR_RIGHT_ROTATE_A_PP  PORTPIN(H,4)
#define MOTOR_RIGHT_ROTATE_B_PP  PORTPIN(H,5)
#define MOTOR_RIGHT_ENCODER_CS_PP  PORTPIN(E,0)
#define MOTOR_RIGHT_PWM_PP  PORTPIN(E,3)
#define MOTOR_RIGHT_PWM_TC_CH  'D'

#define BATTERY_VOLTAGE_PP  PORTPIN(A,3)


#endif
