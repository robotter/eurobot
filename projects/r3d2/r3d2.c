#include <stdbool.h>
#include <math.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/atomic.h>
#include <avarix/intlvl.h>
#include <pwm/motor.h>
#include <clock/defs.h>
#include <util/delay.h>
#include "r3d2.h"
#include "config.h"


#define TC_CLKSEL_DIVn_gc_(n)  TC_CLKSEL_DIV ## n ## _gc
#define TC_CLKSEL_DIVn_gc(n)  TC_CLKSEL_DIVn_gc_(n)
#define LETTER_TO_CHAR(x)  ((#x)[0])


/// Captured object positions
typedef struct {
  uint16_t start;  ///< capture start tick
  uint16_t len;  ///< capture duration in ticks
} r3d2_capture_t;


/// R3D2 state
typedef struct {

  pwm_motor_t pwm;  ///< PWM for R3D2 motor
  /// Captured object positions, buffered
  volatile r3d2_capture_t captures[R3D2_OBJECTS_MAX];
  /// Number of capture objects
  volatile uint8_t capture_count;
  /// Motor rotation period (in timer ticks)
  volatile uint16_t motor_period;
  /// Motor enabled
  bool motor_enabled;

  /** @brief Number of updates without motor rotation
   *
   * Use to detect when the motor is stucked.
   */
  volatile uint8_t motor_inactivity;

  /// R3D2 current configuration values
  r3d2_conf_t conf;

} r3d2_t;

/// R3D2 state singleton
static r3d2_t r3d2 = {
  .conf = {
    .motor_speed = R3D2_MOTOR_SPEED_DEFAULT,
    .motor_timeout = R3D2_MOTOR_TIMEOUT_DEFAULT,
    .angle_offset = R3D2_ANGLE_OFFSET_DEFAULT,
    .dist_coef = R3D2_DIST_COEF_DEFAULT,
  },
};

/// R3D2 EEPROM configuration values
EEMEM r3d2_conf_t eeprom_conf = {
  .motor_speed = R3D2_MOTOR_SPEED_DEFAULT,
  .motor_timeout = R3D2_MOTOR_TIMEOUT_DEFAULT,
  .angle_offset = R3D2_ANGLE_OFFSET_DEFAULT,
  .dist_coef = R3D2_DIST_COEF_DEFAULT,
};


void r3d2_init(void)
{
  // init motor
  pwm_motor_init(&r3d2.pwm, &(R3D2_MOTOR_PWM_TC), LETTER_TO_CHAR(R3D2_MOTOR_POS_CH), 0);
  R3D2_MOTOR_POS_TC.CNT = 0;
  R3D2_MOTOR_POS_TC.CTRLA = TC_CLKSEL_DIVn_gc(R3D2_MOTOR_POS_PRESCALER);
  pwm_motor_set_frequency(&r3d2.pwm, R3D2_MOTOR_FREQ);

  // init sensor
  portpin_dirset(&R3D2_SENSOR_CTRL_PP);

  // init motor interrupt (on falling edge)
  PORTPIN_CTRL(&R3D2_MOTOR_INT_PP) |= PORT_ISC_FALLING_gc;
  portpin_enable_int(&R3D2_MOTOR_INT_PP, R3D2_MOTOR_INT_NUM, R3D2_INTLVL);

  // init sensor interrupt (on both edges)
  PORTPIN_CTRL(&R3D2_SENSOR_INT_PP) |= PORT_ISC_BOTHEDGES_gc;
  portpin_enable_int(&R3D2_SENSOR_INT_PP, R3D2_SENSOR_INT_NUM, R3D2_INTLVL);

  r3d2_stop();
}


static void r3d2_start_motor(void)
{
  // motor need a short burst to start
  r3d2.motor_enabled = true;
  pwm_motor_set(&r3d2.pwm, R3D2_MOTOR_SPEED_BURST);
  _delay_ms(500);
  pwm_motor_set(&r3d2.pwm, r3d2.conf.motor_speed);
}

void r3d2_start(void)
{
  portpin_outset(&R3D2_SENSOR_CTRL_PP);
  r3d2_start_motor();
}


void r3d2_stop(void)
{
  r3d2.motor_enabled = false;
  pwm_motor_set(&r3d2.pwm, 0);
  portpin_outclr(&R3D2_SENSOR_CTRL_PP);
}


void r3d2_set_motor_speed(uint16_t speed)
{
  r3d2.conf.motor_speed = speed;
  pwm_motor_set(&r3d2.pwm, r3d2.conf.motor_speed);
  r3d2.motor_enabled = speed != 0;
}


/// Get R3D2 configuration
const r3d2_conf_t *r3d2_get_conf(void)
{
  return &r3d2.conf;
}

void r3d2_set_conf(const r3d2_conf_t *conf)
{
  if(conf) {
    r3d2.conf = *conf;
  }
  pwm_motor_set(&r3d2.pwm, r3d2.conf.motor_speed);
}


void r3d2_update(r3d2_data_t *data)
{
  // get a local copy of capture state
  uint8_t count;
  r3d2_capture_t captures[R3D2_OBJECTS_MAX];
  uint16_t motor_period;
  ATOMIC_BLOCK(ATOMIC_FORCEON) {
    count = r3d2.capture_count;
    for(uint8_t i=0; i<count; i++) {
      captures[i] = r3d2.captures[i];
    }
    motor_period = r3d2.motor_period;
  }

  for(uint8_t i=0; i<count; i++) {
    // compute object position
    // use opposite value because motor rotation direction is inverted
    double angle = -(double)(captures[i].start + captures[i].len/2)*2*M_PI/motor_period;
    angle += r3d2.conf.angle_offset;
    if(angle > 2*M_PI) {
      angle -= 2*M_PI;
    } else if(angle < 0) {
      angle += 2*M_PI;
    }

    double dist = r3d2.conf.dist_coef * captures[i].len;

    data->objects[i].angle = angle;
    data->objects[i].dist = dist;
  }
  // update count last, thus previous count may be used above
  data->count = count;

  // restart motor if stuck
  // note: motor_inactivity is a single byte, no need to lock
  if(r3d2.motor_enabled && r3d2.motor_inactivity++ >= R3D2_MOTOR_TIMEOUT_DEFAULT) {
    r3d2.motor_inactivity = 0;
    r3d2_start_motor();
  }
}


void r3d2_calibrate_angle(double angle)
{
  r3d2_data_t data;
  r3d2_update(&data);
  if(data.count == 0) {
    return;
  }

  double offset = (angle - data.objects[0].angle) + r3d2.conf.angle_offset;
  if(offset > 2*M_PI) {
    offset -= 2*M_PI;
  } else if(offset < 0) {
    offset += 2*M_PI;
  }
  r3d2.conf.angle_offset = offset;
}

void r3d2_calibrate_dist(double dist)
{
  r3d2_data_t data;
  r3d2_update(&data);
  if(data.count == 0) {
    return;
  }

	r3d2.conf.dist_coef *= dist / data.objects[0].dist;
}


void r3d2_conf_load(void)
{
  eeprom_read_block(&r3d2.conf, &eeprom_conf, sizeof(eeprom_conf));
  r3d2_set_conf(0); // reapply the configuration
}

void r3d2_conf_save(void)
{
  eeprom_update_block(&r3d2.conf, &eeprom_conf, sizeof(eeprom_conf));
}



/// Internal current capture state, used only by interrupt routines
static struct {
  /// Captured objects for current motor revolution
  r3d2_capture_t captures[R3D2_OBJECTS_MAX];
  /// Current position in \e capture
  uint8_t index;
  /// Motor rotation period (in timer ticks)
  volatile uint16_t motor_period;
  /// 1 if an object is being captured
  uint8_t capturing:1;
  /// Flag to end capture on detection end
  uint8_t capture_end:1;

} capture_state;


/** @brief End current capture, swap the result, start a new capture
 * @warning This method must only be used by interrupt routines.
 */
static void r3d2_capture_swap(void)
{
  ATOMIC_BLOCK(ATOMIC_FORCEON) {
    // copy capture to r3d2
    r3d2.capture_count = capture_state.index;
    for(uint8_t i=0; i<capture_state.index; i++) {
      r3d2.captures[i] = capture_state.captures[i];
    }
    r3d2.motor_period = capture_state.motor_period;
  }

  // start a new capture
  capture_state.index = 0;
}


/// Motor revolution interrupt routine
ISR(R3D2_MOTOR_INT_VECT)
{
  capture_state.motor_period = R3D2_MOTOR_POS_TC.CNT;
  R3D2_MOTOR_POS_TC.CNT = 0;

  r3d2.motor_inactivity = 0;
  if(capture_state.capturing) {
    // wait for the end of the current detection
    capture_state.capture_end = 1;
  } else {
    // end capture now
    r3d2_capture_swap();
  }
}


/// Sensor interrupt routine
ISR(R3D2_SENSOR_INT_VECT)
{
  portpin_outtgl(&LED_RUN_PP);
  r3d2_capture_t *capture = capture_state.captures + capture_state.index;
  uint16_t motor_pos = R3D2_MOTOR_POS_TC.CNT;
  if(!capture_state.capturing) {
    // rising edge, detection starts
    capture->start = motor_pos;
    capture_state.capturing = 1;
  } else {
    // falling edge, detection ends
    if(capture_state.capture_end) {
      // note: ok on overflow
      capture->len = motor_pos+capture_state.motor_period - capture->start;
      capture_state.capture_end = 0;
      r3d2_capture_swap();
    } else {
      capture->len = motor_pos - capture->start;
      capture_state.index++;
    }
    capture_state.capturing = 0;
  }
}


