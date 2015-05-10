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
#include "settings.h"

extern rome_intf_t rome;

#define TM_FREQUENCY (3)
#define TM_PRESCALER ((int32_t)1e6/((int32_t)TM_FREQUENCY*CONTROL_SYSTEM_PERIOD_US))

#define _DWZ(f) do{f}while(0)

#define _PRESCALE(N,f) _DWZ(\
static int _prescaler = 0;\
if(_prescaler++>(N)) {\
  _prescaler = 0;\
  f;\
})

#define TM_DL_XYA(x,y,a) _PRESCALE(TM_PRESCALER, ROME_SEND_ASSERV_TM_XYA(&rome,x,y,a))
#define TM_DL_ENCODER_RAW(enc0,enc1,enc2) _PRESCALE(TM_PRESCALER, ROME_SEND_ASSERV_TM_ENCODER_RAW(&rome,enc0,enc1,enc2))

#define TM_DL_X_PID(i,e,o) _PRESCALE(TM_PRESCALER, ROME_SEND_ASSERV_TM_X_PID(&rome,i,e,o))
#define TM_DL_Y_PID(i,e,o) _PRESCALE(TM_PRESCALER, ROME_SEND_ASSERV_TM_Y_PID(&rome,i,e,o))
#define TM_DL_A_PID(i,e,o) _PRESCALE(TM_PRESCALER, ROME_SEND_ASSERV_TM_A_PID(&rome,i,e,o))

#define TM_DL_HTRAJ_DONE(xy,a) _PRESCALE(TM_PRESCALER, ROME_SEND_ASSERV_TM_HTRAJ_DONE(&rome,xy,a))
#define TM_DL_HTRAJ_AUTOSET_DONE(b) _PRESCALE(TM_PRESCALER, ROME_SEND_ASSERV_TM_HTRAJ_AUTOSET_DONE(&rome,b))
#define TM_DL_HTRAJ_STATE(s) _PRESCALE(TM_PRESCALER, ROME_SEND_ASSERV_TM_HTRAJ_STATE(&rome,s))
#define TM_DL_HTRAJ_CARROT_XY(x,y) _PRESCALE(TM_PRESCALER, ROME_SEND_ASSERV_TM_HTRAJ_CARROT_XY(&rome,x,y))
#define TM_DL_HTRAJ_SPEED(v) _PRESCALE(TM_PRESCALER, ROME_SEND_ASSERV_TM_HTRAJ_SPEED(&rome,v))
#define TM_DL_HTRAJ_PATH_INDEX(i,n) _PRESCALE(TM_PRESCALER, ROME_SEND_ASSERV_TM_HTRAJ_PATH_INDEX(&rome,i,n))

#define TM_DL_MOTORS(a,b,c) _PRESCALE(TM_PRESCALER, ROME_SEND_ASSERV_TM_MOTORS(&rome,a,b,c))

#define TM_DL_GP2_RAWS(a,b,c) _PRESCALE(TM_PRESCALER, ROME_SEND_ASSERV_TM_GP2_RAWS(&rome,a,b,c))
#define TM_DL_GP2_DET(a,b,c) _PRESCALE(TM_PRESCALER, ROME_SEND_ASSERV_TM_GP2_DET(&rome,a,b,c))

#define TM_DL_MATCH_TIMER(a) _PRESCALE(TM_PRESCALER, ROME_SEND_ASSERV_TM_MATCH_TIMER(&rome,a))

#define TM_DL_GYRO_CALIBRATION(a) _PRESCALE(TM_PRESCALER, ROME_SEND_ASSERV_TM_GYRO_CALIBRATION(&rome, a))

#endif//TELEMETRY_H
