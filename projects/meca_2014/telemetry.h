/*  
 *  Copyright RobOtter (2014)
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

#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <rome/rome.h>
#include "config.h"

extern rome_intf_t rome;

#define TM_FREQUENCY (3)
#define TM_PRESCALER ((int32_t)1e6/((int32_t)TM_FREQUENCY*UPDATE_ARM_US))

#define _DWZ(f) do{f}while(0)

#define _PRESCALE(N,f) _DWZ(\
static int32_t _prescaler = 0;\
if(_prescaler++>(N)) {\
  _prescaler = 0;\
  f;\
})

#define TM_PERIODIC(f) _PRESCALE(TM_PRESCALER, f;)
#define TM_DL_ARM(a,b,c) _PRESCALE(TM_PRESCALER, ROME_SEND_MECA_TM_ARM(&rome,a,b,c))
#define TM_DL_SUCKERS(a,b) _PRESCALE(TM_PRESCALER, ROME_SEND_MECA_TM_SUCKERS(&rome,a,b))

#define TM_DL_MATCH_TIMER(a) _PRESCALE(TM_PRESCALER, ROME_SEND_MECA_TM_MATCH_TIMER(&rome,a))

#endif//TELEMETRY_H
