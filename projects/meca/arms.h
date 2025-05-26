/*
 *  Copyright RobOtter (2016)
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#ifndef ARMS_H
#define ARMS_H

#include <rome/rome.h>
#include "barometer.h"

typedef enum {
  ARM_INIT = 0,
  ARM_IDLE,

  ARM_ELEVATOR_RESET = 10,
  ARM_ELEVATOR_RESET_DEBOUNCE,
  ARM_ELEVATOR_MOVE,
  ARM_ELEVATOR_MOVE_WAIT,

  ARM_TAKE_CANS = 20,

  ARM_RELEASE_CANS = 30,

  ARM_CAN_GRABBER_WAIT,

} arm_state_t;

typedef struct {
  bool side;
  rome_enum_meca_state_t tm_state;
  arm_state_t state;
  struct {
    // note: pos and target must be castable to int16_t
    uint16_t pos;  // Current position in steps, unrelevant if pos_known is true
    uint16_t target;  // Target position in steps
    bool pos_known;  // True if elevator has been reset
  } elevator;

  bool atoms[3];
  barometer_t baro;

  /// When measuring a value, time at which end the measure
  uint32_t measure_end;

  uint16_t pressure;
  uint32_t baro_time;
} arm_t;

extern arm_t arm_l;
extern arm_t arm_r;

void arm_deploy_wings(arm_t *arm);
void arm_fold_wings(arm_t *arm);
void arm_take_cans(arm_t *arm);
void arm_release_cans(arm_t *arm);
void arm_elevator_move(arm_t *arm, uint16_t pos);
void arm_elevator_up(arm_t *arm);
void arm_elevator_shutdown(arm_t *arm);
void arm_elevator_accelerator(arm_t *arm);
uint8_t arms_get_tm_state(void);
void arms_init(void);
void arm_update(arm_t *arm);
void arms_shutdown(void);
void arms_deploy_arm(arm_t *arm);
void arms_close_arm(arm_t *arm);

/// Convert an arm position from millimeters to steps
#define ARM_MM_TO_STEPS(side, mm) \
    (((mm) * (uint32_t)((side) ? LEFT_ARM_HEIGHT_STEPS : RIGHT_ARM_HEIGHT_STEPS)) / ARM_HEIGHT_MM)

/// Convert an arm position from steps to millimeters
#define ARM_STEPS_TO_MM(side, steps) \
    (((steps) * (uint32_t)ARM_HEIGHT_MM) / ((side) ? LEFT_ARM_HEIGHT_STEPS : RIGHT_ARM_HEIGHT_STEPS))

#define SIDE_NAME(side) ((side) ? "left" : "right")
#define SIDE_ARM(side)  ((side) ? &arm_l : &arm_r )

#endif//ARMS_H
