#ifndef CONFIG_H__
#define CONFIG_H__

// Mapping of UART to boards/interfaces
#define ROME_ASSERV_UART  uartE1
#define ROME_MECA_UART    uartD0
#define ROME_PADDOCK_UART uartF0

// Tick period of uptime counter, in microseconds
#define UPDATE_TICK_US  10000
// Interrupt level of tick interrupt
#define UPTIME_INTLVL  INTLVL_HI

// Timeout for ROME ACK, in microseconds
#define ROME_ACK_TIMEOUT_US  500000

// Timeout before repeating various strat orders
#define STRAT_TIMEOUT_US  1000000

// Margin for arm positions
#define ARM_UPPER_MARGIN  2000
#define ARM_LOWER_MARGIN  10

// Battery alert value
#define BATTERY_ALERT_LIMIT  13500

// R3D2 maximum objects to detect
#define R3D2_OBJECTS_MAX  2
// R3D2 distance under which object must be avoided
#define R3D2_AVOID_DISTANCE  500

// Starting cord: 0 is plugged, 1 in unplugged
#define STARTING_CORD_PP  PORTPIN(C,1)
// Color selector: 0 is yellow, 1 is red
#define COLOR_SELECTOR_PP  PORTPIN(C,0)


// Leds
#define LED_R_PP  PORTPIN(F,0)
#define LED_G_PP  PORTPIN(F,7)
#define LED_B_PP  PORTPIN(A,7)

#define LED0_PP  PORTPIN(A,1)
#define LED1_PP  PORTPIN(A,2)
#define LED2_PP  PORTPIN(A,3)
#define LED3_PP  PORTPIN(A,4)


#endif
