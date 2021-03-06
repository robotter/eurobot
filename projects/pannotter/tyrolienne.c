#include <avarix.h>
#include <avarix/portpin.h>
#include <timer/timer.h>
#include "tyrolienne.h"
#include "config.h"

static bool tyrolienne_is_arrived;
static bool tyrolienne_is_started;

void tyrolienne_timer_callback(void) {
  if (portpin_in(&TYROLIENNE_STOP_PIN)) {
    tyrolienne_is_arrived = true;
    tyrolienne_stop();
  }
  portpin_outtgl(&TYROLIENNE_MOTOR_STEP_PIN);
}

void tyrolienne_init(void) {
  portpin_dirset(&TYROLIENNE_MOTOR_STEP_PIN);
  portpin_outclr(&TYROLIENNE_MOTOR_STEP_PIN);

  portpin_dirset(&TYROLIENNE_MOTOR_DIR_PIN);
  portpin_outclr(&TYROLIENNE_MOTOR_DIR_PIN);

  portpin_dirset(&TYROLIENNE_MOTOR_EN_PIN);
  portpin_outclr(&TYROLIENNE_MOTOR_EN_PIN);

  portpin_dirclr(&TYROLIENNE_STOP_PIN);
  PORTPIN_CTRL(&TYROLIENNE_STOP_PIN) = PORT_OPC_PULLUP_gc;
}

void tyrolienne_start(void) {
  if(tyrolienne_is_started) {
    return;
  }
  tyrolienne_is_arrived = false;

  portpin_outclr(&TYROLIENNE_MOTOR_DIR_PIN);

  portpin_outset(&TYROLIENNE_MOTOR_EN_PIN);
  TIMER_SET_CALLBACK_US(D0, 'A', 2000, INTLVL_HI, tyrolienne_timer_callback);

  tyrolienne_is_started = true;
}

bool tyrolienne_started(void) {
  return tyrolienne_is_started;
}

bool tyrolienne_arrived(void) {
  return tyrolienne_is_arrived;
}

void tyrolienne_stop(void) {
  TIMER_CLEAR_CALLBACK(D0, 'A');
}

void tyrolienne_shutdown(void) {
  portpin_outclr(&TYROLIENNE_MOTOR_EN_PIN);
  TIMER_CLEAR_CALLBACK(D0, 'A');
}

