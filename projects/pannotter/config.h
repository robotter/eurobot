#ifndef CONFIG_H
#define CONFIG_H

#define UART_XBEE  uartC1

#define MATCH_DURATION_SECS (99)
#define ROBOTS_ALIVE_TIMEOUT  (10)

#define BATTERY_ALERT_LIMIT  12000
#define BATTERY_STAND_THRESHOLD  18000

#define PIXEL_DIRECTION PIXEL_DIR_HORIZONTAL
//#define PIXEL_DIRECTION PIXEL_DIR_VERTICAL
//#define PIXEL_DIRECTION PIXEL_DIR_REVERSED

#define TYROLIENNE_MOTOR_STEP_PIN  PORTPIN(D, 0)
#define TYROLIENNE_MOTOR_DIR_PIN  PORTPIN(D, 2)
#define TYROLIENNE_MOTOR_EN_PIN  PORTPIN(D, 1)
#define TYROLIENNE_STOP_PIN  PORTPIN(D, 7)

#endif
