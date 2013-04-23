#include <avarix.h>
#include <clock/clock.h>
#include <uart/uart.h>
#include <pwm/motor.h>
#include <ax12/ax12.h>
#include <timer/timer.h>

#include <math.h>

#define AX12_DIR_PORT     PORTC
#define AX12_DIR_PIN_bp   5 

#define AX12_TX_PORT     PORTC
#define AX12_TX_PIN_bp   7 

#define AX12_RX_PORT     PORTC
#define AX12_RX_PIN_bp   6 

#define UART_AX12  uartC1


// Tick period of uptime counter, in microseconds
#define UPTIME_TICK_US  1000

// AX-12 timeout for state switch, in microseconds
#define AX12_TIMEOUT_US  10000



/// current time in microseconds
volatile uint32_t uptime;

/// Called on uptime timer tick
void uptime_update(void)
{
  uptime += UPTIME_TICK_US;
}

pwm_motor_t cc_motor[4];

static void cc_motor_0_pwm_set(int16_t val)
{
 if (val >= 0)
 {
    PORTD.OUTCLR = _BV(4);
    pwm_motor_set(cc_motor+0, val); 
 }
 else
 {
    PORTD.OUTSET = _BV(4);
    pwm_motor_set(cc_motor+0, PWM_MOTOR_MAX + val); 
  }
}

static void cc_motor_1_pwm_set(int16_t val)
{
 if (val >= 0)
 {
    PORTD.OUTCLR = _BV(6);
    pwm_motor_set(cc_motor+1, val); 
 }
 else
 {
    PORTD.OUTSET = _BV(6);
    pwm_motor_set(cc_motor+1, PWM_MOTOR_MAX + val); 
  }
}

static void cc_motor_2_pwm_set(int16_t val)
{
 if (val >= 0)
 {
    PORTE.OUTCLR = _BV(0);
    pwm_motor_set(cc_motor+2, val); 
 }
 else
 {
    PORTE.OUTSET = _BV(0);
    pwm_motor_set(cc_motor+2, PWM_MOTOR_MAX + val); 
  }
}

static void cc_motor_3_pwm_set(int16_t val)
{
 if (val >= 0)
 {
    PORTE.OUTCLR = _BV(2);
    pwm_motor_set(cc_motor+3, val); 
 }
 else
 {
    PORTE.OUTSET = _BV(2);
    pwm_motor_set(cc_motor+3, PWM_MOTOR_MAX + val); 
  }
}

static volatile uint8_t char_nsent;

void ax12_set_state(ax12_state_t state)
{
  switch(state)
  {
    case AX12_STATE_WRITE:
      //USARTC1.CTRLB |= USART_TXEN_bm; 
      AX12_DIR_PORT.OUTSET = _BV(AX12_DIR_PIN_bp);
      AX12_TX_PORT.DIRSET = _BV(AX12_TX_PIN_bp);
      char_nsent = 0;
     // USARTC1.CTRLB &= ~USART_RXEN_bm; 
      break;

    case AX12_STATE_READ:
      
      while(char_nsent>0) {
        while(uart_recv_nowait(uartC1) == -1);
        char_nsent--;
        }
     // USARTC1.CTRLB &= ~USART_TXEN_bm; 
      AX12_DIR_PORT.OUTCLR = _BV(AX12_DIR_PIN_bp);
      //AX12_TX_PORT.DIRCLR = _BV(AX12_TX_PIN_bp);
       USARTC1.CTRLB |= USART_RXEN_bm; 
      break;

  }

}
int8_t ax12_send_fn(uint8_t data)
{
  char_nsent ++;
  return uart_send(UART_AX12, data);
}

int ax12_recv_fn(void)
{
  uint32_t tend = uptime + AX12_TIMEOUT_US;
  for(;;) {
    int c = uart_recv_nowait(UART_AX12);
    if(c != -1) {
      return c;
    }
    if(tend <= uptime) {
      return -1; // timeout
    }
  }
//return uart_recv_nowait(UART_AX12);
}

int main(void) {

  clock_init();
  timer_init();

  // status led (com, error and run)
  PORTK.DIRSET = (1<<0)|(1<<1)|(1<<2);
  PORTK.OUTCLR = (1<<0)|(1<<1)|(1<<2);
  AX12_DIR_PORT.DIRSET = _BV(AX12_DIR_PIN_bp);
  AX12_DIR_PORT.OUTSET = _BV(AX12_DIR_PIN_bp);

  AX12_TX_PORT.DIRSET = _BV(AX12_TX_PIN_bp);
  
  uart_init();
  uart_fopen(uartF0); // use UARTF0 as stdin/stdout
  __asm__("sei");
  INTLVL_ENABLE_ALL();

  pwm_motor_t pwm_servos[5];
  pwm_servo_init( &pwm_servos[0], &TCD0, 'A');
  pwm_servo_init( &pwm_servos[1], &TCD0, 'B');
  pwm_servo_init( &pwm_servos[2], &TCD0, 'C'); //SV1 connector
  pwm_servo_init( &pwm_servos[3], &TCD0, 'D');
  
 PORTD.DIRSET = 0x0F; 

  printf("test meca\n");

// timer
  timer_set_callback(timerE0, 'A', TIMER_US_TO_TICKS(E0,UPTIME_TICK_US), INTLVL_LO, uptime_update);
  
  // ANALOG SERVOS
  uint8_t lvla = 50;
  uint8_t lvlb = 50;
  double t = 0.0;

  int pos = 2000;

  for (uint8_t it = 0; it < 4; it ++)
  {
   // pwm_motor_set_range(&pwm_servos[it], 1000, 2000);
    pwm_motor_set(&pwm_servos[it], pos);
  }

  // AX12
  ax12_t ax12 = 
  { 
    .send = ax12_send_fn,
    //.send = ax12_send_char,
    .recv = ax12_recv_fn,
    //.recv = ax12_recv_char,
    .set_state = ax12_set_state
  };

  for(uint8_t it; it < 100; it ++) 
  {
    printf("@%3u:%3u \t", it, ax12_ping(&ax12, it));
    if (it %5 == 0)
      printf("\n");
  }
  printf("\n");

  bool update_analog_servos = false;

  PORTD.DIRSET = _BV(4)|_BV(6);
  PORTE.DIRSET = _BV(0)|_BV(2);
  pwm_motor_init(cc_motor+0, (TC0_t*)&TCD1, 'B', 0);
  pwm_motor_init(cc_motor+1, (TC0_t*)&TCD1, 'D', 0);
  pwm_motor_init(cc_motor+2, &TCE0, 'B', 0);
  pwm_motor_init(cc_motor+3, &TCE0, 'D', 0);
  pwm_motor_set_frequency(cc_motor+0, 5000); 
  pwm_motor_set_frequency(cc_motor+1, 5000); 
  pwm_motor_set_frequency(cc_motor+2, 5000); 
  pwm_motor_set_frequency(cc_motor+3, 5000); 
  pwm_motor_set(cc_motor+0, 0);
  pwm_motor_set(cc_motor+1, 0);
  pwm_motor_set(cc_motor+2, 0);
  pwm_motor_set(cc_motor+3, 0);

  volatile uint32_t cpt = 0; 
  while(1) {

    cpt++;
    if (cpt == 100)
    {
      ax12_write_byte(&ax12, 2, 0x19, 0);
    }
    else if (cpt == 2000)
    {
      ax12_write_byte(&ax12, 2, 0x19, 1);
      cpt = 0;
    }
    int c = uart_recv_nowait(uartF0);
    if(c != -1) {
      update_analog_servos = false;
      switch(c)
      {
        case 'p' : pos += 100;
                   update_analog_servos = true;
                   break;
        case 'm' : pos -= 100;
                   update_analog_servos = true;
                   break;
     
      }


      /*for (uint8_t it = 0; it < 4; it ++)
      {
        pwm_motor_set(&pwm_servos[it], pos);
      }*/

      
      pwm_motor_set(&pwm_servos[2], pos);
      printf("%i\n", pos); 
    } 

    t+=0.01;
    lvla = 255*(0.5+0.5*cos(t));
    lvlb = 255*(0.5+0.5*sin(1.1*t));

    uint8_t x,y;
    x = _BV(0);
    y = _BV(1)|_BV(2);

    uint8_t i;
    for(i=0;i<255;i++) {
      PORTK.OUT = (i < lvla) ? x : 0;
      PORTK.OUT = (i < lvlb) ? y : 0;
    }

    (void)cc_motor_0_pwm_set;
    (void)cc_motor_1_pwm_set;
    (void)cc_motor_2_pwm_set;
    (void)cc_motor_3_pwm_set;
    //cc_motor_0_pwm_set(16000 * cos(t/10));
    //cc_motor_1_pwm_set(16000 * cos(t/10));
    //cc_motor_2_pwm_set(16000 * cos(t/10));
    //cc_motor_3_pwm_set(16000 * cos(t/10));
    cc_motor_1_pwm_set(16000);

    //pwm_motor_set(cc_motor+2, -16000); 

/*    pwm_motor_set(&cc_motor[2], 16000 * cos(t));
    pwm_motor_set(&cc_motor[3], 16000 * cos(t));
  
  
    pwm_motor_set(&cc_motor[2], 10 );
    pwm_motor_set(&cc_motor[3], 10 );
 */
 }

  return 0;
}
