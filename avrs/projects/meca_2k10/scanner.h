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

/** \file scanner.h
  * \author JD
  */

#ifndef SCANNER_H
#define SCANNER_H

#include <aversive.h>
#include <aversive/error.h>
#include <actuators.h>

typedef struct
{
  // structure used to communicate with the scanner
  actuators_t *actuator;

  // current ax12 position
  int16_t current_ax12_position;

  // ax12 increment between 2 call of scanner_updatette
  uint16_t scanner_speed;

  // rotation direction
  uint8_t scanner_direction;

  // scanner scanner minimum position
  int16_t scanner_minPosition;
  
  // scanner scanner maximum position
  int16_t scanner_maxPosition;
}scanner_t;

/** */
void scanner_init(scanner_t*);

/** */
void scanner_update(scanner_t*);

#endif/*SCANNER_H*/
