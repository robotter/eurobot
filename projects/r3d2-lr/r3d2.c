#include <avarix.h>
#include <clock/defs.h>
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <timer/timer.h>
#include <pwm/motor.h>
#include <rome/rome.h>
#include "r3d2.h"
#include "leds.h"
#include "pid.h"

#include <timer/uptime.h>

static uint32_t uptime_ms(void) {
  return uptime_us()/1000;
}

#define LENGTH_OF(array) (sizeof(array)/sizeof(array[0]))

extern rome_intf_t rome_intf;

#define R3D2_MOTOR_PWM_TC TCE0
#define R3D2_MOTOR_POS_CH 'A'
#define R3D2_MOTOR_FREQUENCY 10000

#define R3D2_MOTOR_POS_TC  TCC1
#define R3D2_MOTOR_POS_PRESCALER  TIMER_PRESCALER_DIV
/// Port pin of interrupt triggered at motor revolution
#define R3D2_MOTOR_INT_PP  PORTPIN(C,3)
/// Channel used to capture motor period
#define R3D2_MOTOR_CAPT_CH 'A'
/// Event used to capture motor period
#define R3D2_MOTOR_CAPT_EVENT  0
/** @brief Interrupt vector for motor capture
 *
 * Value must match R3D2_MOTOR_POS_TC and R3D2_MOTOR_CAPT_CH.
 */
#define R3D2_MOTOR_CAPT_VECT  TCC1_CCA_vect

/// Port pin of interrupt triggered by R3D2 short range sensor
#define R3D2_SR_SENSOR_INT_PP  PORTPIN(C,1)
/// Port interrupt number of interrupt triggered by R3D2 short range sensor
#define R3D2_SR_SENSOR_INT_NUM  0
/// interrupt vector triggered at motor by R3D2 short range sensor
#define R3D2_SR_SENSOR_INT_VECT  PORTC_INT0_vect

/// Port pin of interrupt triggered by R3D2 long range sensor
#define R3D2_LR_SENSOR_INT_PP  PORTPIN(C,2)
/// Port interrupt number of interrupt triggered by R3D2 long range sensor
#define R3D2_LR_SENSOR_INT_NUM  1
/// interrupt vector triggered at motor by R3D2 long range sensor
#define R3D2_LR_SENSOR_INT_VECT  PORTC_INT1_vect

/// Port pin of sensor control (on/off)
#define R3D2_SENSOR_CTRL_PP  PORTPIN(E,1)

#define R3D2_OBJECT_WIDTH_CM 8.0
#define R3D2_MAX_OBJECTS  4

#define TC_CLKSEL_DIVn_gc_(n)  TC_CLKSEL_DIV ## n ## _gc
#define TC_CLKSEL_DIVn_gc(n)  TC_CLKSEL_DIVn_gc_(n)


typedef struct {
  int32_t start_tick_us;
  int32_t end_tick_us;
}measure_t;

typedef struct {
  double distance;
  double angle;
}distance_angle_t;

typedef struct {
  distance_angle_t da;
  bool is_valid;
  uint32_t last_seen_ms;
}object_t;

typedef struct {
  // mirror motor pwm
  pwm_motor_t pwm;
  // mirror motor pid
  struct pid_filter pid;
  // motor rotation speed (in turns per minute)
  int32_t motor_rpm;
  // motor rotation period (in us)
  int32_t motor_period_us;
  // motor rotation speed consign (in turns per minute)
  int32_t motor_consign_rpm;
  // motor rotation speed validity threshold (in percents of consign)
  int32_t motor_consign_threshold_pc;
  // if true next motor tick will be invalidated
  bool invalidate_next_tick;
  // true if motor rpm is stable
  bool motor_rpm_stable;
  // blind spot
  struct {
    double begin;
    double end;
  }blind_spot_arc;

  measure_t measure_lr;
  measure_t measure_sr;

  object_t objects[5];

} r3d2_t;
/// r3d2 singleton
r3d2_t r3d2 = {
  .motor_consign_rpm = 2*60,
  .motor_consign_threshold_pc = 5,

  .blind_spot_arc = {.begin = 0, .end = 0},

  .invalidate_next_tick = false,
  .motor_rpm_stable = false,
};

static inline distance_angle_t _compute_distance_angle(measure_t *m, double offset_rad) {
  const double w = R3D2_OBJECT_WIDTH_CM;

  int32_t delta_us = m->end_tick_us - m->start_tick_us;
  int32_t angle_us = (m->end_tick_us + m->start_tick_us)/2;

  // compute view angle
  double delta = (2*M_PI*delta_us)/r3d2.motor_period_us;
  double angle = -(2*M_PI*angle_us)/r3d2.motor_period_us;
  double distance = w/(2*tan(.5*delta));

  angle += offset_rad;

  return (distance_angle_t){.angle = angle, .distance=distance};
}

/// return true if x is within range [a;b]
static inline bool _in_range(double x, double a, double b) {
  return (a <= x) && (x <= b);
}

/// check if angle x is within arc [a;b]
static inline bool _in_arc(double x, double a, double b) {
  return _in_range(x,a,b) || _in_range(x+2*M_PI,a,b);
}

/// check if angle x is within blind spot
static inline bool _in_blind_spot(double x) {
  return _in_arc(x, r3d2.blind_spot_arc.begin, r3d2.blind_spot_arc.end);
}

///
static bool _close_to(object_t *object, distance_angle_t da) {
  double d1 = object->da.distance;
  double a1 = object->da.angle;
  double d2 = da.distance;
  double a2 = da.angle;

  double x1 = d1*cos(a1);
  double y1 = d1*sin(a1);
  double x2 = d2*cos(a2);
  double y2 = d2*sin(a2);

  double dx = (x2-x1);
  double dy = (y2-y1);

  const double limit_cm = 30.0;

  return (dx*dx + dy*dy) < limit_cm*limit_cm;
}

/// invalidate old detected objects
static void _invalidate_old_objects(void) {

  int32_t timeout_us = 3*r3d2.motor_period_us;
  int32_t timeout_ms = timeout_us/1000;

  for(uint8_t i=0; i<LENGTH_OF(r3d2.objects); i++) {
    object_t* object = r3d2.objects + i;

    if(object->is_valid) {
      int32_t delta_ms = uptime_ms() - object->last_seen_ms;
      if(delta_ms > timeout_ms) {
        object->is_valid = false;
      }
    }
  }
}

/// register a new detection
static void _register_detection(distance_angle_t da) {
  
  for(uint8_t i=0; i<LENGTH_OF(r3d2.objects); i++) {
    object_t* object = r3d2.objects + i;
    // check if valid and near an existing object
    if(object->is_valid && _close_to(object, da) ) {
      object->da = da;
      object->last_seen_ms = uptime_ms();
      return;
    }
  }

  // find first invalid object and set it w/ current position
  for(uint8_t i=0; i<LENGTH_OF(r3d2.objects); i++) {
    object_t* object = r3d2.objects + i;

    if(object->is_valid == false) {
      object->da = da;
      object->is_valid = true;
      object->last_seen_ms = uptime_ms();
      ROME_LOGF(&rome_intf, INFO, "new object");
      return;
    }
  }

  ROME_LOGF(&rome_intf, INFO, "no room for him");
}

/// register a new measure
static void _new_measure(measure_t *m, double offset_rad) {
  // if motor speed is not stable, reject
  if(!r3d2.motor_rpm_stable)
    return;

  int32_t delta = m->end_tick_us-m->start_tick_us;
  if(delta < 0) {
    m->end_tick_us += r3d2.motor_period_us;
  }
  // compute distance angle
  distance_angle_t da = _compute_distance_angle(m, offset_rad);
  // check if angle is contained within blind spot
  double start_tick_angle = -2*M_PI*m->start_tick_us/r3d2.motor_period_us;
  double end_tick_angle = -2*M_PI*m->end_tick_us/r3d2.motor_period_us;

  int32_t distance;
  if(_in_blind_spot(start_tick_angle)||_in_blind_spot(end_tick_angle)) {
    ROME_LOGF(&rome_intf, INFO, "In blind spot !");
    // when object is within blind spot detected angle
    // will be roughtly okay but distance totaly wrong
    // detection will be sent but with distance set to -1
    distance = -1;
  }
  else {
    distance = 10*da.distance;
  }

  (void)distance;
  _register_detection(da);

}

/// Short range sensor interrupt routine
ISR(R3D2_SR_SENSOR_INT_VECT) {
  uint32_t cnt = TIMER_TICKS_TO_US(E0, R3D2_MOTOR_POS_TC.CNT);
  bool pin = portpin_in(&R3D2_SR_SENSOR_INT_PP);

  if(pin) {
    portpin_outset(&LED_SR_PP);
    r3d2.measure_sr.start_tick_us = cnt;
  }
  else {
    portpin_outclr(&LED_SR_PP);
    r3d2.measure_sr.end_tick_us = cnt;
    _new_measure(&r3d2.measure_sr, 0);
  }

}

/// Long range sensor interrupt routine
ISR(R3D2_LR_SENSOR_INT_VECT) {
  uint32_t cnt = TIMER_TICKS_TO_US(E0, R3D2_MOTOR_POS_TC.CNT);
  bool pin = portpin_in(&R3D2_LR_SENSOR_INT_PP);

  if(pin) {
    portpin_outset(&LED_LR_PP);
    r3d2.measure_lr.start_tick_us = cnt;
  }
  else {
    portpin_outclr(&LED_LR_PP);
    r3d2.measure_lr.end_tick_us = cnt;
    _new_measure(&r3d2.measure_lr, M_PI);
  }
}

/// @return error in percent of value against consign
static inline int32_t _error_pc(int32_t value, int32_t consign) {
  return (100*(int32_t)abs(consign-value))/consign;
}

/// Motor revolution interrupt routine
ISR(R3D2_MOTOR_CAPT_VECT) {
  portpin_outtgl(&LED_RUN_PP);

  uint32_t counter_us =
    TIMER_TICKS_TO_US(E0, R3D2_MOTOR_POS_TC.CCA + (R3D2_MOTOR_CAPT_CH - 'A'));
  R3D2_MOTOR_POS_TC.CNT = 0;
  if(!r3d2.invalidate_next_tick) {
    r3d2.motor_period_us = counter_us;
    r3d2.motor_rpm = 60e6/counter_us;
  }

  static bool pstable = false;
  int32_t error = _error_pc(r3d2.motor_rpm, r3d2.motor_consign_rpm);
  r3d2.motor_rpm_stable = error < r3d2.motor_consign_threshold_pc;
  if(pstable != r3d2.motor_rpm_stable) {
    if(r3d2.motor_rpm_stable) {
      ROME_LOGF(&rome_intf, INFO, "Motor speed OK\n");
      portpin_outset(&LED_COM_PP);
    }
    else {
      ROME_LOGF(&rome_intf, INFO, "Motor speed KO\n");
      portpin_outclr(&LED_COM_PP);
    }
  }
  pstable = r3d2.motor_rpm_stable;

  r3d2.invalidate_next_tick = false;
}

/// Motor revolution counter overflow
ISR(TCC1_OVF_vect) {
  r3d2.motor_rpm = 0;
  r3d2.invalidate_next_tick = true;
  ROME_LOGF(&rome_intf, INFO, "OVF");
}

static void _update_motor(void) {
  int32_t error = r3d2.motor_consign_rpm - r3d2.motor_rpm;
  int32_t output = pid_do_filter(&r3d2.pid, error);
  output = MAX(output, 0);
  pwm_motor_set(&r3d2.pwm, output);

}

void r3d2_init() {

  // initialize motor
  pwm_motor_init(&r3d2.pwm, &(R3D2_MOTOR_PWM_TC), R3D2_MOTOR_POS_CH, 0);
  pwm_motor_set_frequency(&r3d2.pwm, R3D2_MOTOR_FREQUENCY);
  // initialize timer counter used for rotor position, with capture
  R3D2_MOTOR_POS_TC.CNT = 0;
  R3D2_MOTOR_POS_TC.CTRLA = TC_CLKSEL_DIVn_gc(R3D2_MOTOR_POS_PRESCALER);
  R3D2_MOTOR_POS_TC.CTRLB = (1 << (TC0_CCAEN_bp + (R3D2_MOTOR_CAPT_CH - 'A')));
  R3D2_MOTOR_POS_TC.CTRLD = TC_EVACT_FRQ_gc | (TC_EVSEL_CH0_gc + R3D2_MOTOR_CAPT_EVENT);
  R3D2_MOTOR_POS_TC.INTCTRLA = TC_OVFINTLVL_LO_gc;
  R3D2_MOTOR_POS_TC.INTCTRLB = TC_CCAINTLVL_LO_gc + (R3D2_MOTOR_CAPT_CH - 'A') * (TC_CCBINTLVL_LO_gc - TC_CCAINTLVL_LO_gc);
  (&EVSYS.CH0MUX)[R3D2_MOTOR_CAPT_EVENT] = PORTPIN_EVSYS_CHMUX(&R3D2_MOTOR_INT_PP);

  // intialize pid filter
  pid_init(&r3d2.pid);
  pid_set_gains(&r3d2.pid, 300, 70, 0);
  pid_set_maximums(&r3d2.pid, 1000, 10000, 0x7fff);
  pid_set_out_shift(&r3d2.pid, 5);

  // init sensor
  portpin_dirset(&R3D2_SENSOR_CTRL_PP);
  portpin_outset(&R3D2_SENSOR_CTRL_PP);
  // init sensor interrupts (on both edges)
  PORTPIN_CTRL(&R3D2_SR_SENSOR_INT_PP) |= PORT_ISC_BOTHEDGES_gc;
  portpin_enable_int(&R3D2_SR_SENSOR_INT_PP, R3D2_SR_SENSOR_INT_NUM, INTLVL_LO);
  PORTPIN_CTRL(&R3D2_LR_SENSOR_INT_PP) |= PORT_ISC_BOTHEDGES_gc;
  portpin_enable_int(&R3D2_LR_SENSOR_INT_PP, R3D2_LR_SENSOR_INT_NUM, INTLVL_LO);
}

void r3d2_update() {
  _update_motor();

  _invalidate_old_objects();

  for(uint8_t i=0; i<LENGTH_OF(r3d2.objects); i++) {
    object_t* object = r3d2.objects + i;
    // check if valid and near an existing objects
    if(object->is_valid) {
      distance_angle_t da = object->da;
      ROME_SEND_R3D2_TM_DETECTION(&rome_intf, i, true, 1000*da.angle,10.0*da.distance);
    }
    else {
      ROME_SEND_R3D2_TM_DETECTION(&rome_intf, i, false, 0.0, 0.0);
    }
  }

  _delay_ms(200);

}

// Set rotation parameters
void r3d2_set_rotation(uint16_t speed, uint8_t threshold) {
  ROME_LOGF(&rome_intf, INFO,
    "Motor rotation parameters set to %d rpm / %d%%",
    speed, threshold);
  r3d2.motor_consign_rpm = speed;
  r3d2.motor_consign_threshold_pc = threshold;
}

/// Set R3D2 blind spot, angle is defined positively from begin to end
void r3d2_set_blind_spot(float begin, float end) {
  int _begin = 180*begin/M_PI;
  int _end = 180*end/M_PI;
  ROME_LOGF(&rome_intf, INFO,
    "R3D2 blind spot set from %d to %d",
    _begin, _end);
  r3d2.blind_spot_arc.begin = begin;
  r3d2.blind_spot_arc.end = end;
}
