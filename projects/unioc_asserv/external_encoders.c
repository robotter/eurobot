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

#include <avarix.h>
#include <string.h>
#include "external_encoders.h"
#include "scales.h"

// encoders singleton
external_encoders_t encoders;

void external_encoders_init() {

  aeat_spi_init();
  aeat_init(encoders.aeats+0, PORTPIN(E,4));
  aeat_init(encoders.aeats+1, PORTPIN(C,7));
  aeat_init(encoders.aeats+2, PORTPIN(C,6));

  // reset vectors
  memset(encoders.vectors, 0, sizeof(encoders.vectors));
}

int32_t* external_encoders_get_value() {

  uint8_t it;
  for(it=0;it<3;it++) {
    encoders.vectors[it] = aeat_get_value(encoders.aeats+it);
  }
  return encoders.vectors;
}

void external_encoders_update()
{
  uint8_t it;
  for(it=0;it<3;it++) {
    aeat_update(encoders.aeats+it);
  }
}
