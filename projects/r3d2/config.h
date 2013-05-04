#ifndef CONFIG_H
#define CONFIG_H

/// Perlimpinpin node address
#define PPP_ADDR  0x13

#define UART_PPP  uartF1


// pinout

#define LED_RUN_PP  PORTPIN(K,0)
#define LED_ERROR_PP  PORTPIN(K,1)
#define LED_COM_PP  PORTPIN(K,2)

#define LED_NORTH_PP  PORTPIN(H,7)
#define LED_EAST_PP  PORTPIN(H,1)
#define LED_SOUTH_PP  PORTPIN(J,3)
#define LED_WEST_PP  PORTPIN(A,0)

#define LED_CARDINAL_PPP(n) \
    ((n) == 0 ? &LED_NORTH_PP : \
     (n) == 1 ? &LED_EAST_PP : \
     (n) == 2 ? &LED_SOUTH_PP : \
     (n) == 3 ? &LED_WEST_PP : \
     &PORTPIN_NONE)


#endif
