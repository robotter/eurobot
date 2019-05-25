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

typedef enum{
  ARM_INIT = 0,
  ARM_IDLE,

  ARM_RESET_ELEVATOR_START = 10,
  ARM_RESET_ELEVATOR_WAIT,

  ARM_ELEVATOR_GO_UP,
  ARM_ELEVATOR_WAIT_UP,

  ARM_ELEVATOR_GO_DOWN,
  ARM_ELEVATOR_WAIT_DOWN,

  ARM_TAKE_ATOMS = 20,

  ARM_RELEASE_ATOMS = 30,

  ARM_CHECK_SUCKERS = 40,
  ARM_CHECK_LEFT_SUCKER,
  ARM_CHECK_CENTER_SUCKER,
  ARM_CHECK_RIGHT_SUCKER,
}arm_state_t;

typedef struct {
  uint8_t tm_state;
  arm_state_t state;
  bool up;
  bool atoms[3];
  bool side;
  uint16_t pressure;
  barometer_t baro;
}arm_t;

extern arm_t arm_l;
extern arm_t arm_r;

void arm_take_atoms(arm_t* arm);
void arm_release_atoms(arm_t* arm);
void arm_elevator_up(arm_t* arm);
void arm_elevator_down(arm_t* arm);
uint8_t arms_get_tm_state(void);
void arms_init(void);
void arm_update(arm_t* arm);
void arms_shutdown(void);

#endif//ARMS_H
