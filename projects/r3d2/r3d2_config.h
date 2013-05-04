#ifndef R3D2_CONFIG_H__
#define R3D2_CONFIG_H__


/// Default motor speed (PWM value from 0 to 32167)
#define R3D2_MOTOR_SPEED_DEFAULT  13000
/// Starting motor speed, for burst (PWM value from 0 to 32167)
#define R3D2_MOTOR_SPEED_BURST  20000
/// Default motor stuck timeout (in capture counts)
#define R3D2_MOTOR_TIMEOUT_DEFAULT  50
/// Default capture angle offset, in radians
#define R3D2_ANGLE_OFFSET_DEFAULT  0.0
/// Default distance coefficient
#define R3D2_DIST_COEF_DEFAULT  1.0


/// Motor frequency, in hertz
#define R3D2_MOTOR_FREQ  10000

/// Timer/Counter used for motor PWM
#define R3D2_MOTOR_PWM_TC  TCD0
/// Timer/Counter channel used for motor PWM
#define R3D2_MOTOR_PWM_CH  A

/// Timer/Counter used for motor position
#define R3D2_MOTOR_POS_TC  TCC1
/// Prescaler ratio for Timer/Counter used motor position
#define R3D2_MOTOR_POS_PRESCALER  64

/// Port pin of interrupt triggered at motor revolution
#define R3D2_MOTOR_INT_PP  PORTPIN(C,3)
/// Port interrupt number of interrupt triggered at motor revolution
#define R3D2_MOTOR_INT_NUM  0
/// Interrupt vector triggered at motor revolution
#define R3D2_MOTOR_INT_VECT  PORTC_INT0_vect


/// Port pin of interrupt triggered by R3D2 sensor
#define R3D2_SENSOR_INT_PP  PORTPIN(C,2)
/// Port interrupt number of interrupt triggered by R3D2 sensor
#define R3D2_SENSOR_INT_NUM  1
/// interrupt vector triggered at motor by R3D2 sensor
#define R3D2_SENSOR_INT_VECT  PORTC_INT1_vect

/// Port pin of sensor control (on/off)
#define R3D2_SENSOR_CTRL_PP  PORTPIN(D,3)


/// Interrupt level used for R3D2 interrupts
#define R3D2_INTLVL  INTLVL_HI

/** @brief Number of detectable objects per motor turn
 *
 * Sensor detect objects as motor position ranges at which it detects
 * something. This value is the maximum number of ranges that can be stored for
 * each motor revolution.
 *
 * It should be set to the number of mirrors on the field.
 */
#define R3D2_OBJECTS_MAX  2


#endif
