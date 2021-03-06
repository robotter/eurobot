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

/** \file motor_encoders.h
  * \author JD
  */

#ifndef MOTOR_ENCODERS_H
#define MOTOR_ENCODERS_H

#include <avarix.h>
#include <avarix/portpin.h>
#include <encoder/quadra/quadra.h>

typedef struct {
  // motors quadrature decoders
  quadra_t quadras[3];
  // decoders 0degs (A) signals
  portpin_t ppAs[3];
  // decoders 90degs (B) signals
  portpin_t ppBs[3];
  // output vectors
  int16_t vectors[6];
}motor_encoders_t;

void motor_encoders_init(void);

int16_t *motor_encoders_get_value(void);

void motor_encoders_update(void);

#endif/*MOTOR_ENCODERS_H*/
