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

#ifndef ROME_ACKS_H
#define ROME_ACKS_H

#include <avarix.h>
#include <stdint.h>
#include <stdbool.h>

#include <rome/rome.h>

#define ROME_SEND_AND_WAIT(msg, intf, ...) do {\
  uint8_t fid;\
  do {\
    fid = rome_acks_new_frame_id();\
    ROME_SEND_##msg(intf, fid, ##__VA_ARGS__);\
  } while(rome_acks_wait(fid)); \
} while(0);

/** @brief Generate a new frame ID and mark it as unavaible */
uint8_t rome_acks_new_frame_id(void);

/** @brief Handle ROME_ACK messages, return true if an ACK message was handled */
bool rome_acks_handle(rome_intf_t *intf, const rome_frame_t *frame);

/** @brief Wait some time for frame ID ACK, 
 * @return true if ACK was received while waiting, false otherwise */
bool rome_acks_wait(uint8_t fid);

#endif//ROME_ACKS_H
