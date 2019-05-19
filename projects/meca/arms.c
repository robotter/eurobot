#include <avr/io.h>
#include <avarix.h>
#include <avarix/portpin.h>
#include <ax12/ax12.h>
#include "config.h"
#include "servos.h"
#include <clock/clock.h>
#include <util/delay.h>
#include "arms.h"
#include <timer/uptime.h>

extern ax12_t ax12;

extern rome_intf_t rome_strat;

#define RETRY_AND_UPDATE(fcall) while(!(fcall)) { \
  portpin_outtgl(&LED_AN_PP(0)); \
  _delay_ms(10); \
  }

arm_t arm_l;
arm_t arm_r;

void arms_init(void){
  arm_l.tm_state = ROME_ENUM_MECA_STATE_BUSY;
  arm_r.tm_state = ROME_ENUM_MECA_STATE_BUSY;
  // left arm, barometer ADCA4
  barometer_init(&arm_l.baro, &ADCA, ADC_CH_MUXPOS_PIN4_gc);
  // right arm, barometer ADCA5
  barometer_init(&arm_r.baro, &ADCA, ADC_CH_MUXPOS_PIN5_gc);
}

uint8_t arms_get_tm_state(void){
  if (arm_l.tm_state <= arm_r.tm_state)
    return arm_l.tm_state;
  else
    return arm_r.tm_state;
}

void arm_take_atoms(arm_t arm){
}

void arm_release_atoms(arm_t arm){
}

void arm_elevator_up(arm_t arm){
}

void arm_elevator_down(arm_t arm){
}

void arm_update(arm_t arm){

}

void arms_shutdown(void){
  arm_release_atoms(arm_l);
  arm_release_atoms(arm_r);
}
