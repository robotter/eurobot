#ifndef CONFIG_H
#define CONFIG_H

#define UART_DFPLAYER  uartD0
#define UART_ROME  uartC0

#define DFPLAYER_BUSY_PP  PORTPIN(D,5)

#define AMPLI_GAIN_0_PP   PORTPIN(A,6)
#define AMPLI_GAIN_1_PP   PORTPIN(A,4)
#define AMPLI_MUTE_PP     PORTPIN(A,7)
#define AMPLI_SHUTDOWN_PP PORTPIN(B,0)

#endif