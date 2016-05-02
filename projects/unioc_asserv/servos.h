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

#ifndef SERVOS_H
#define SERVOS_H

void servos_init (void);

// servo 0 connected to connector PF3
// servo 1 connected to connector PF6
// servo 2 connected to connector PF7
// servo 3 connected to connector PF4
void servo_set(uint8_t id, uint16_t value);
#endif
