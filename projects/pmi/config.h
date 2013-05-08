#ifndef CONFIG_H
#define CONFIG_H

/*
 * TCC0 -- scheduler
 *  - channel A: update of motor encoders
 *  - channel B: battery monitoring
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

/// Battery monitoring period, in microseconds
#define BATTERY_MONITORING_PERIOD_US  100000
/// Interrupt level for battery monitoring
#define BATTERY_MONITORING_INTLVL  INTLVL_MED
/// Minimum voltage at boot
#define BATTERY_MONITORING_LOW_VOLTAGE_DECIVOLTS 105

/// Rangefinders monitoring period, in microseconds
#define RANGEFINDERS_MONITORING_PERIOD_US  100000
/// Interrupt level for rangefinders monitoring
#define RANGEFINDERS_MONITORING_INTLVL  INTLVL_MED

// main update period in us
#define MAIN_PERIOD_US 100000
// interrupt level for main update
#define MAIN_INTLVL INTLVL_HI

/// Distance precision margin, in mm
#define TRAJECTORY_MARGIN_DIST  10
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
#define MOTOR_LEFT_PWM_TC  TCE0
#define MOTOR_LEFT_PWM_CH  'C'

#define MOTOR_RIGHT_ENABLE_A_PP  PORTPIN(H,7)
#define MOTOR_RIGHT_ENABLE_B_PP  PORTPIN(H,6)
#define MOTOR_RIGHT_ROTATE_A_PP  PORTPIN(H,2)
#define MOTOR_RIGHT_ROTATE_B_PP  PORTPIN(H,3)
#define MOTOR_RIGHT_ENCODER_CS_PP  PORTPIN(E,0)
#define MOTOR_RIGHT_PWM_TC  TCE0
#define MOTOR_RIGHT_PWM_CH  'D'

#define BATTERY_VOLTAGE_PP  PORTPIN(A,3)

#define SERVO_ANA_0_TC  TCE1
#define SERVO_ANA_0_CH  'A'
#define SERVO_ANA_1_TC  TCE1
#define SERVO_ANA_1_CH  'B'
#define SERVO_ANA_2_TC  TCF0
#define SERVO_ANA_2_CH  'A'
#define SERVO_ANA_3_TC  TCF0
#define SERVO_ANA_3_CH  'B'
#define SERVO_ANA_4_TC  TCF0
#define SERVO_ANA_4_CH  'C'
#define SERVO_ANA_5_TC  TCF0
#define SERVO_ANA_5_CH  'D'
#define SERVO_ANA_6_TC  TCF1
#define SERVO_ANA_6_CH  'A'
#define SERVO_ANA_7_TC  TCF1
#define SERVO_ANA_7_CH  'B'


#endif
