#include <avarix.h>
#include <clock/clock.h>
#include <uart/uart.h>
#include <pwm/motor.h>
#include <ax12/ax12.h>
#include <timer/timer.h>
#include <encoder/aeat/aeat.h>
#include <math.h>
#include "../meca/acm.h"

/// use this define to have advanced 
#define USE_CAKE_ADVANCED_PROCESSING


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

#define MOTOR_INCREMENT  1000

#define AX12_INCREMENT 10

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

static volatile uint8_t ax12_nsent;

void ax12_set_state(ax12_state_t state)
{
  switch(state)
  {
    case AX12_STATE_WRITE:
      //USARTC1.CTRLB |= USART_TXEN_bm; 
      AX12_DIR_PORT.OUTSET = _BV(AX12_DIR_PIN_bp);
      AX12_TX_PORT.DIRSET = _BV(AX12_TX_PIN_bp);
      ax12_nsent = 0;
      // USARTC1.CTRLB &= ~USART_RXEN_bm; 
      break;

    case AX12_STATE_READ:

      while(ax12_nsent>0) {
        int c;
        while( (c = uart_recv_nowait(uartC1)) == -1);
        ax12_nsent--;
      }
      // USARTC1.CTRLB &= ~USART_TXEN_bm; 
      AX12_DIR_PORT.OUTCLR = _BV(AX12_DIR_PIN_bp);
      AX12_TX_PORT.DIRCLR = _BV(AX12_TX_PIN_bp);
      USARTC1.CTRLB |= USART_RXEN_bm; 
      break;

  }

}
int8_t ax12_send_char(uint8_t data)
{
  ax12_nsent ++;
  return uart_send(UART_AX12, data);
}

int ax12_recv_char(void)
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


uint8_t t_ax12_move(ax12_t *s, uint8_t id, uint16_t pos)
{
  return ax12_write_word(s, id, AX12_ADDR_GOAL_POSITION_L, pos);
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
    .send = ax12_send_char,
    .recv = ax12_recv_char,
    .set_state = ax12_set_state
  };

  for(uint8_t it; it < 5; it ++) 
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


  PORTE.DIRSET = _BV(4);
  PORTE.OUTSET = _BV(4);
  aeat_t cake_enc;
  aeat_spi_init();
  aeat_init(&cake_enc, PORTPIN( E, 4));

  volatile uint32_t cpt = 0; 

  int16_t motor_pwm[4]; 

  uint16_t ax12_pos[4]; 

  (void) cc_motor;
  (void) ax12_pos;
  (void) motor_pwm;
  (void)lvla;
  (void)lvlb;
  (void)cpt;
  (void)t;
  (void)cc_motor_0_pwm_set;
  (void)cc_motor_1_pwm_set;
  (void)cc_motor_2_pwm_set;
  (void)cc_motor_3_pwm_set;
  (void)update_analog_servos;

  acm_t acm;
  acm_init(&acm);
  acm.ax12 = &ax12;
  acm.encoder = &cake_enc;
  acm.set_second_lvl_motor_pwm = cc_motor_3_pwm_set;
  acm.set_first_lvl_left_motor_pwm = cc_motor_0_pwm_set;
  acm.set_first_lvl_right_motor_pwm = cc_motor_2_pwm_set;

 /// debug 
  for (uint8_t it= 0; it < FIRST_LVL_CANDLE_NB; it ++)
  {
    if (acm.candle_color[it] == ACM_CANDLE_UNKNOWN)
    {
      if (it%2 == 0)
      {
        acm.candle_color[it] = ACM_CANDLE_BLUE; 
   //     acm.candle_color[it] = ACM_CANDLE_RED; 
      }
      else
      {
        acm.candle_color[it] = ACM_CANDLE_RED; 
      }
    }
  }

  while(1) {
#ifdef USE_CAKE_ADVANCED_PROCESSING
    aeat_update(&cake_enc);
    acm_update(&acm);
    int c = uart_recv_nowait(uartF0);
    if(c != -1) 
    {
    printf("test");
      update_analog_servos = false;
      switch(c)
      {
        case 'p' : acm.sm_state = ACM_SM_MAX_OPEN_FIRST_LVL_LEFT;
                   break;
        case 'm' : acm.sm_state = ACM_SM_HOME_MOVE_FIRST_LVL_RIGHT;
                   break;
        case 'o' : acm.sm_state = ACM_SM_CAKING_MOVE_SECOND_LVL;
                   break;
        default : break;
      }
      PORTK.OUTTGL = _BV(2);
    }
   // for (volatile uint32_t it = 0; it < 100000; it ++) ;
    PORTK.OUTTGL = _BV(0);
#else
    aeat_update(&cake_enc);

    cpt++;
    if (cpt == 100)
    {
      ax12_write_byte(&ax12, 2, 0x19, 0);
    }
    /* else if (cpt == 200)
       {
       uint8_t data = 0xff;
       ax12_read_byte(&ax12, 2, 0x00, &data);
       printf( "%X\t", data);
       ax12_read_byte(&ax12, 2, 0x01, &data);
       printf( "%X\t",data);
       ax12_read_byte(&ax12, 2, 0x02, &data);
       printf( "%X\n",data);
       }*/
    else if (cpt == 2000)
    {
      ax12_write_byte(&ax12, 2, 0x19, 1);
      cpt = 0;
    }

    int c = uart_recv_nowait(uartF0);
    if(c != -1) 
    {
      update_analog_servos = false;
      switch(c)
      {
        case 'p' : pos += 100;
                   update_analog_servos = true;
                   break;
        case 'm' : pos -= 100;
                   update_analog_servos = true;
                   break;
        case 'y' : motor_pwm[0] +=  MOTOR_INCREMENT;
                   break;
        case 'h' : motor_pwm[0] -=  MOTOR_INCREMENT;
                   break;

        case 'u' : motor_pwm[1] +=  MOTOR_INCREMENT;
                   break;
        case 'j' : motor_pwm[1] -=  MOTOR_INCREMENT;
                   break;

        case 'i' : motor_pwm[2] +=  MOTOR_INCREMENT;
                   break;
        case 'k' : motor_pwm[2] -=  MOTOR_INCREMENT;
                   break;

        case 'o' : motor_pwm[3] +=  MOTOR_INCREMENT;
                   break;
        case 'l' : motor_pwm[3] -=  MOTOR_INCREMENT;
                   break;

        case 'q' : ax12_pos[0] +=  AX12_INCREMENT;
                   break;
        case 'w' : ax12_pos[0] -=  AX12_INCREMENT;
                   break;

        case 's' : ax12_pos[1] +=  AX12_INCREMENT;
                   break;
        case 'x' : ax12_pos[1] -=  AX12_INCREMENT;
                   break;

        case 'd' : ax12_pos[2] +=  AX12_INCREMENT;
                   break;
        case 'c' : ax12_pos[2] -=  AX12_INCREMENT;
                   break;

        case 'f' : ax12_pos[3] +=  AX12_INCREMENT;
                   break;
        case 'v' : ax12_pos[3] -=  AX12_INCREMENT;
                   break;

        case 'z' : pos = 2600;
                   motor_pwm[0] = 0;
                   motor_pwm[1] = 0;
                   motor_pwm[2] = 0;
                   motor_pwm[3] = 0;
                   ax12_pos[0] = 200;
                   ax12_pos[1] = 520; 
                   ax12_pos[2] = 500; 
                   break;

      }

      /*for (uint8_t it = 0; it < 4; it ++)
        {
        pwm_motor_set(&pwm_servos[it], pos);
        }*/

      for (int8_t it = 0; it < 4; it ++)
      {
        ax12_pos[it] &= 0x3ff;
      }
      t_ax12_move(&ax12, 2, ax12_pos[0]);
      t_ax12_move(&ax12, 3, ax12_pos[1]);
      t_ax12_move(&ax12, 4, ax12_pos[2]);
      pwm_motor_set(&pwm_servos[2], pos);
      printf("servos %4i\t mot0 %5i\t mot1 %5i\t mot2 %5i\t mot3 %5i\tax12_2 %u\tax12_3 %u\tax12_4 %u\n",pos, motor_pwm[0], motor_pwm[1], motor_pwm[2], motor_pwm[3], ax12_pos[0], ax12_pos[1], ax12_pos[2]); 
    } 

    if (cpt == 10)
    {
      printf("\tcake aeat %li\n", aeat_get_value(&cake_enc));
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
    cc_motor_0_pwm_set( motor_pwm[0]);
    cc_motor_1_pwm_set( motor_pwm[1]);
    cc_motor_2_pwm_set( motor_pwm[2]);
    cc_motor_3_pwm_set( motor_pwm[3]);
#     endif
  }
  return 0;
}
