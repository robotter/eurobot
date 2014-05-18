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

volatile bool rome_acks_vector[256] = {false};

uint8_t rome_acks_new_frame_id() {
  static uint8_t fid = 0; 
  bool b;
  // limit search in case all frame IDs are assigned
  int wdog;
  for(wdog=0; wdog<256; wdog++) {
    // generate a new frame ID
    fid++;
    // check frame ID avaibility
    INTLVL_DISABLE_BLOCK(ROME_SEND_INTLVL) {
      b = rome_acks_vector[fid];
    }
    if(!b) break;
  }
  // new frame ID is marked as "unavaible"/"waiting for an answer"
  INTLVL_DISABLE_BLOCK(ROME_SEND_INTLVL) {
    rome_acks_vector[fid] = true;
  }
  return fid;
}

bool rome_acks_handle(rome_intf_t *intf, const rome_frame_t *frame) {
  if(frame->mid != ROME_MID_ACK) {
    return false;
  }

  // frame ID associated with ACK
  uint8_t fid = frame->ack.fid;
  // mark FID as "avaible"/"received"
  rome_acks_vector[fid] = false;

  return true;
}

bool rome_acks_wait(uint8_t fid) {
  bool b;
  int32_t wdog = 0;
  do {
    INTLVL_DISABLE_BLOCK(ROME_SEND_INTLVL) {
      b = rome_acks_vector[fid];
    }
    // XXX ugly, ugly, ugly code ! 
    // replace me with some timers !
    _delay_us(1000);
    wdog++;
    if(wdog > 500) 
      return false;
    // XXX
  } while(b);

  return true;
}
