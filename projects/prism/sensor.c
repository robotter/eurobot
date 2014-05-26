#include <clock/clock.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avarix/intlvl.h>
#include <string.h>
#include <pwm/motor.h>
#include <util/delay.h>
#include "beacom.h"
#include "sensor.h"
#include "sensor_config.h"

volatile uint16_t dma_top_data[2 * MAX_OBJECT];
volatile uint16_t top_data[2 * MAX_OBJECT];
volatile uint16_t top_period;
volatile uint8_t top_shift;
volatile uint16_t top_data_latched[2 * MAX_OBJECT];
volatile uint16_t top_period_latched;
volatile uint8_t new_data;

static void sensor_enable_top_dma(uint8_t enable) {
  if(enable) {
    DMA.CH0.TRFCNT = 2;
    DMA.CH0.REPCNT = MAX_OBJECT;
    uint16_t addr = (uint16_t)&dma_top_data[0]; 
    DMA.CH0.DESTADDR0 = (addr >>  0) & 0xff;
    DMA.CH0.DESTADDR1 = (addr >>  8) & 0xff;
    DMA.CH0.DESTADDR2 = 0;
    DMA.CH0.CTRLA = DMA_CH_ENABLE_bm | DMA_CH_BURSTLEN_2BYTE_gc | DMA_CH_SINGLE_bm | DMA_CH_REPEAT_bm;
  }
  else {
    DMA.CH0.CTRLA = 0;
  }
}

ISR(PORTD_INT0_vect) {
  sensor_enable_top_dma(0);

  //reset counter
  top_period = TCD1.CNT;
  TCD1.CNT = 0;

  if(PORTC.IN & (1 << 0)) {
    top_shift = 1;
  }
  else {
    top_shift = 0;
  }

  //save data
  memcpy((void *)top_data, (void *)dma_top_data, 2 * MAX_OBJECT * sizeof(uint16_t));
  memset((void *)dma_top_data, 0xffff, 2 * MAX_OBJECT * sizeof(uint16_t));

  sensor_enable_top_dma(1);
  new_data = 1;
}



void sensor_init(void) {

  /*
   * Start motor
   */
  SENSOR_TOP_MOTOR_ENABLE_PORT.DIRSET = (1 << SENSOR_TOP_MOTOR_ENABLE_PIN);
  SENSOR_TOP_MOTOR_ENABLE_PORT.OUTSET = (1 << SENSOR_TOP_MOTOR_ENABLE_PIN);

  /*
   * Start sensor
   */
  SENSOR_TOP_SENSOR_ENABLE_PORT.DIRSET = (1 << SENSOR_TOP_SENSOR_ENABLE_PIN);
  SENSOR_TOP_SENSOR_ENABLE_PORT.OUTSET = (1 << SENSOR_TOP_SENSOR_ENABLE_PIN);

  /*
   * init sensor TOP
   * -> PC0 -> Event0 -> DMA CH0 (trig)
   *    Timer D1      -> DMA CH0 (value)
   */

  //sensor pin
  PORTC.DIRCLR = (1 << 0);
  //  PORTC.PIN0CTRL = PORT_INVEN_bm;
  //idx pin
  PORTD.DIRCLR = (1 << 4);
  PORTD.PIN4CTRL = PORT_ISC_RISING_gc;
  PORTD.INTCTRL = PORT_INT0LVL_HI_gc;
  PORTD.INT0MASK = (1 << 4);
  //event
  EVSYS.CH0MUX = EVSYS_CHMUX_PORTC_PIN0_gc;
  //timer
  TCD1.CTRLA = TC_CLKSEL_DIV256_gc;
  //DMA
  DMA.CTRL = DMA_ENABLE_bm;
  DMA.CH0.TRIGSRC = DMA_CH_TRIGSRC_EVSYS_CH0_gc;
  DMA.CH0.ADDRCTRL = DMA_CH_SRCRELOAD_BURST_gc | DMA_CH_SRCDIR_INC_gc | DMA_CH_DESTRELOAD_TRANSACTION_gc | DMA_CH_DESTDIR_INC_gc;
  uint16_t addr;
  addr = (uint16_t)&TCD1.CNT; 
  DMA.CH0.SRCADDR0 = (addr >>  0) & 0xff;
  DMA.CH0.SRCADDR1 = (addr >>  8) & 0xff;
  DMA.CH0.SRCADDR2 = 0;
  sensor_enable_top_dma(1);

  pwm_motor_t fan_top;
  pwm_servo_init(&fan_top, &TCD0, 'A');
  pwm_motor_set_frequency(&fan_top, SENSOR_FAN_PWM_FREQUENCY);


  //fan speed burst
  switch(beacom_get_id()) {
    case 0:
      pwm_motor_set(&fan_top, SENSOR_B0_FAN_PWM_BURST);
      _delay_ms(SENSOR_B0_FAN_BURST_DELAY);
      pwm_motor_set(&fan_top, SENSOR_B0_FAN_PWM_RUN);
      break;

    case 1:
      pwm_motor_set(&fan_top, SENSOR_B1_FAN_PWM_BURST);
      _delay_ms(SENSOR_B1_FAN_BURST_DELAY);
      pwm_motor_set(&fan_top, SENSOR_B1_FAN_PWM_RUN);
      break;

    case 2:
      pwm_motor_set(&fan_top, SENSOR_B2_FAN_PWM_BURST);
      _delay_ms(SENSOR_B2_FAN_BURST_DELAY);
      pwm_motor_set(&fan_top, SENSOR_B2_FAN_PWM_RUN);
      break;
  }
}

void sensor_latch(void) {
  //INTLVL_DISABLE_ALL_BLOCK {
  memcpy((void *)top_data_latched, (void *)top_data, 2 * MAX_OBJECT * sizeof(uint16_t));
  top_period_latched = top_period;
  if(top_shift) {
    uint16_t dl0 = top_data_latched[0];
    for(int i = 0; i < 2 * MAX_OBJECT; i += 1) {
      top_data_latched[i] = top_data_latched[i + 1];
      if(top_data_latched[i] == 0xffff) {
        top_data_latched[i] = dl0;
        break;
      }
    }
  }
  //}
}

double sensor_get_period(sensor_position_t sid) {
  switch(sid) {
    case SENSOR_TOP:
      return top_period_latched * 256.0 / 16000.0;
  }
  return 0;
}

uint8_t sensor_get_object_number(sensor_position_t sid) {
  uint8_t count = 0;

  for(int i = 0; i < 2 * MAX_OBJECT; i += 2) {
    if(top_data_latched[i] != 0xffff) {
      count += 1;
    }
  }

  return count;
}

double sensor_get_object_angle(sensor_position_t sid, uint8_t id) {
  switch(sid) {
    case SENSOR_TOP:
      if(id < MAX_OBJECT) {
        double mid;
        if(top_data_latched[id] > top_data_latched[id + 1]) {
          mid = (top_data_latched[id] + top_data_latched[id + 1] - top_period_latched) / 2;
        }
        else {
          mid = (top_data_latched[id] + top_data_latched[id + 1]) / 2;
        }

        if(mid > top_period_latched) {
          mid -= top_period_latched;
        }
        return mid * 360.0 / ((double)top_period_latched);
      }
      break;
  }

  return -1;
}

double sensor_get_object_distance(sensor_position_t sid, uint8_t id) {
  switch(sid) {
    case SENSOR_TOP:
      if(id < MAX_OBJECT) {
        double d;
        if(top_data_latched[id] > top_data_latched[id + 1]) {
          d = top_data_latched[id + 1] + top_period_latched;
          d -= top_data_latched[id];
        }
        else {
          d = (top_data_latched[id + 1] - top_data_latched[id]);
        }

        return d * 360.0 / ((double)top_period_latched);
      }
      break;
  }

  return -1;
}

uint8_t sensor_new_data_available(void)
{
  //INTLVL_DISABLE_ALL_BLOCK {
    if(new_data) {
      new_data = 0;
      return 1;
    }
  //}

  return 0;
}
