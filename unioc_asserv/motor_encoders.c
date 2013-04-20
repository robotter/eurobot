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

/** \file motor_encoders.c
  * \author JD
  */

#include <avarix.h>
#include <string.h>
#include <encoder/quadra/quadra.h>
#include "motor_encoders.h"

// encoders singleton
motor_encoders_t encoders;

void motor_encoders_init() {
  
  // setup port pins for channels A & B for each encoder
  encoders.ppBs[0] = PORTPIN(C,0);
  encoders.ppAs[0] = PORTPIN(C,1);
  encoders.ppBs[1] = PORTPIN(C,2);
  encoders.ppAs[1] = PORTPIN(C,3);
  encoders.ppBs[2] = PORTPIN(C,4);
  encoders.ppAs[2] = PORTPIN(C,5);
  // setup quadrature decoders 
  quadra_init(encoders.quadras+0, &TCC1, 0, encoders.ppAs[0], encoders.ppBs[0], 8);
  quadra_init(encoders.quadras+1, &TCD1, 2, encoders.ppAs[1], encoders.ppBs[1], 8);
  quadra_init(encoders.quadras+2, &TCE1, 4, encoders.ppAs[2], encoders.ppBs[2], 8);
  // reset vectors
  memset(encoders.vectors, 0, sizeof(encoders.vectors));
}

int16_t* motor_encoders_get_value()
{
  uint8_t it;
  for(it=0;it<3;it++)
    encoders.vectors[it] = (int16_t)quadra_get_value(encoders.quadras+it);
  for(it=3;it<6;it++)
    encoders.vectors[it] = 0;
  return encoders.vectors;
}
