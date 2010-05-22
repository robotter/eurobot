/*  
 *  Copyright RobOtter (2010) 
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

/** \file scanner.c
  * \author JD
  */

#include <aversive.h>
#include <aversive/error.h>
#include <adc.h>

#include "scanner.h"

void scanner_init(scanner_t* s)
{
  s->scanner_maxPosition = 50;
  s->scanner_minPosition = -50;
  s->current_ax12_position = s->scanner_minPosition;
  s->scanner_direction = 0;
  adc_init();
  return;
}

void scanner_setActuator(scanner_t* s, actuators_t *t)
{
  s->actuator = t;
}

void scanner_update(scanner_t* s)
{
  
  int16_t future_position;
  NOTICE(0, "GP2 val: %u", adc_get_value(ADC_NO_CONFIG));
  
  // calculate future position
  if(s->scanner_direction == 1)
    future_position = s->current_ax12_position + s->scanner_speed;
  else
    future_position = s-> current_ax12_position - s->scanner_speed;

  // change direction and filter position in case of out of range
  if(future_position >= s->scanner_maxPosition)
  {
    future_position = s->scanner_maxPosition;
    s->scanner_direction = 0;
  }

  if(future_position<= s->scanner_minPosition)
  {
    future_position = s->scanner_minPosition;
    s->scanner_direction = 1;
  }
  
  // update ax12_position
  actuators_scanner_setAngle(s->actuator, future_position); 
 
  s->current_ax12_position = future_position;

  // lauch future analog conversion
  adc_launch(ADC_REF_AVCC|MUX_ADC0);
  return;
}
