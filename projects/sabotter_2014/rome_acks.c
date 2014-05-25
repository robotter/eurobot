/*  
 *  Copyright RobOtter (2013)
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
/** @brief Extend ROME protocol with ack'ed message management */

#include "rome_acks.h"
#include "rome_config.h"

#include <clock/clock.h>
#include <util/delay.h>

static volatile bool rome_acks_vector[ROME_FRAME_ID_MAX+1] = {false};

uint8_t rome_acks_new_frame_id() {
  static uint8_t fid = 0; 
  // wrap the wall loop with disable block
  // an available fid should be found quickly
  INTLVL_DISABLE_BLOCK(ROME_SEND_INTLVL) {
    // limit search in case all frame IDs are assigned
    for(int wdog=0; wdog<=ROME_FRAME_ID_MAX; wdog++) {
      // generate a new frame ID
      fid = fid == ROME_FRAME_ID_MAX ? 0 : fid+1;
      // check frame ID avaibility, use it if available
      if(!rome_acks_vector[fid]) {
        break;
      }
    }
    // new frame ID is marked as "unavaible"/"waiting for an answer"
    rome_acks_vector[fid] = true;
  }
  return fid;
}

void rome_acks_free_frame_id(uint8_t fid) {
  rome_acks_vector[fid] = false;
}

bool rome_acks_wait(uint8_t fid) {
  //XXX don't use hardcoded wdog limit and delay
  for(uint16_t wdog=0; wdog<500; wdog++) {
    INTLVL_DISABLE_BLOCK(ROME_SEND_INTLVL) {
      if(rome_acks_vector[fid]) {
        return true;
      }
    }
    _delay_us(1000);
  }
  return false;
}

