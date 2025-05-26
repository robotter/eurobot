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
#ifndef SERVO_HAT_H
#define SERVO_HAT_H

#include <rome/rome.h>


#include <avr/io.h>
#include <clock/clock.h>
#include <util/delay.h>
#include <avarix.h>
#include <avarix/portpin.h>
#include <i2c/i2c.h>
#include <timer/uptime.h>
#include "config.h"
#include "stepper_motor.h"

void servo_hat_init(void);

// set pwm limits for one servo channel
void servo_hat_configure_channel(uint8_t channel, uint16_t min_pwm, uint16_t max_pwm);
void servo_hat_set_pwm(uint8_t channel, uint16_t position_percent);

#endif //SERVO_HAT_H
