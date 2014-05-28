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

/** \file avoidance.c
  * \author JD
  */

#include "avoidance.h"
#include "adc.h"
#include "settings.h"
#include "telemetry.h"

/** @brief Initialize avoidance systems */
void avoidance_init(avoidance_t* av)
{
  adc_init(&SETTING_AVOIDANCE_GP2ARRAY_ADC);

  av->gp2_it = 0;

  av->gp2_muxs[0] = SETTING_AVOIDANCE_GP2ARRAY_30;
  av->gp2_muxs[1] = SETTING_AVOIDANCE_GP2ARRAY_150;
  av->gp2_muxs[2] = SETTING_AVOIDANCE_GP2ARRAY_270;

  av->forced_blocked = 0;
}

/** @brief Update avoidance system */
void avoidance_update(avoidance_t* av)
{
  uint16_t adcval;
  uint8_t *pv;

  // get new ADC measure (from a previously launched one)
  adcval = adc_get_value(&SETTING_AVOIDANCE_GP2ARRAY_ADC,
                          av->gp2_muxs[av->gp2_it]);
  // store raw value
  av->gp2_raws[av->gp2_it] = adcval;
  // update dectection vector
  pv = av->gp2_detections + av->gp2_it;
  // if ADC over threshold (something detected)
  if( adcval > SETTING_AVOIDANCE_GP2ARRAY_THRESHOLD )
  {
    if(*pv < 255)
      (*pv)++;
  }
  else
    *pv = 0;

  // increment gp2 sensor muxer
  av->gp2_it++;
  
  if(av->gp2_it >= SETTING_AVOIDANCE_GP2ARRAY_SIZE)
    av->gp2_it = 0;

  // update telemetries
  uint16_t *raws = av->gp2_raws;
  TM_DL_GP2_RAWS(raws[0], raws[1], raws[2]);
  uint8_t *det = av->gp2_detections;
  TM_DL_GP2_DET(det[0], det[1], det[2]);
}

/** @brief Force avoidance to blocked 
 * @param state 0 - normal 1 - force to blocked status */
void avoidance_force_blocked(avoidance_t* av, uint8_t state)
{
  av->forced_blocked = state;
}

/** @brief Check avoidance sensors */
direction_t avoidance_check(avoidance_t* av)
{
  // if avoidance is blocked set all direction
  if( av->forced_blocked )
    return DIR_30|DIR_150|DIR_270;

#ifdef SETTING_AVOIDANCE_ENABLED
  uint16_t rv = DIR_NONE;
  if( av->gp2_detections[0] > SETTING_AVOIDANCE_GP2ARRAY_COUNT )
    rv |= DIR_30;
  if( av->gp2_detections[1] > SETTING_AVOIDANCE_GP2ARRAY_COUNT )
    rv |= DIR_150;
  if( av->gp2_detections[2] > SETTING_AVOIDANCE_GP2ARRAY_COUNT )
    rv |= DIR_270;
  return rv;
#else
  return DIR_NONE;
#endif
}
