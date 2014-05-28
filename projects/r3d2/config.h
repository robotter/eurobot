#ifndef CONFIG_H
#define CONFIG_H

/// Perlimpinpin node address
#define UART_ROME  uartF1

// Tick period of uptime counter, in microseconds
#define UPDATE_TICK_US  10000
// Interrupt level of tick interrupt
#define UPTIME_INTLVL  INTLVL_HI

// Capture period, in microseconds
#define CAPTURE_PERIOD_US  20000
// Telemetry period, in microseconds
#define TELEMETRY_PERIOD_US  200000


// pinout

#define LED_RUN_PP  PORTPIN(K,0)
#define LED_ERROR_PP  PORTPIN(K,1)
#define LED_COM_PP  PORTPIN(K,2)

#define LED_NORTH_PP  PORTPIN(H,7)
#define LED_EAST_PP  PORTPIN(H,1)
#define LED_SOUTH_PP  PORTPIN(J,3)
#define LED_WEST_PP  PORTPIN(A,0)

#endif
