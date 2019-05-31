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

#if defined(GALIPEUR)
#define ROME_DEVICE ROME_ENUM_DEVICE_GALIPEUR_ASSERV
#elif defined(GALIPETTE)
#define ROME_DEVICE ROME_ENUM_DEVICE_GALIPETTE_ASSERV
#endif

#define TM_FREQUENCY (3)
#define TM_PRESCALER ((int32_t)1e6/((int32_t)TM_FREQUENCY*CONTROL_SYSTEM_PERIOD_US))

#define _DWZ(f) do{f}while(0)

#define _PRESCALE(N,f) _DWZ(\
static int _prescaler = 0;\
if(_prescaler++>(N)) {\
  _prescaler = 0;\
  f;\
})

#define TM_DL_XYA(x,y,a) _PRESCALE(TM_PRESCALER, ROME_SEND_ASSERV_TM_XYA(UART_STRAT,ROME_DEVICE,x,y,a))
#define TM_DL_ENCODER_RAW(enc0,enc1,enc2) _PRESCALE(TM_PRESCALER, ROME_SEND_ASSERV_TM_ENCODER_RAW(UART_STRAT,enc0,enc1,enc2))

#if 0
#define TM_DL_X_PID(i,e,o) _PRESCALE(TM_PRESCALER, ROME_SEND_ASSERV_TM_X_PID(UART_STRAT,i,e,o))
#define TM_DL_Y_PID(i,e,o) _PRESCALE(TM_PRESCALER, ROME_SEND_ASSERV_TM_Y_PID(UART_STRAT,i,e,o))
#define TM_DL_A_PID(i,e,o) _PRESCALE(TM_PRESCALER, ROME_SEND_ASSERV_TM_A_PID(UART_STRAT,i,e,o))
#else
#define TM_DL_X_PID(i,e,o)
#define TM_DL_Y_PID(i,e,o)
#define TM_DL_A_PID(i,e,o)
#endif

#define TM_DL_HTRAJ_DONE(xy,a) _PRESCALE(TM_PRESCALER, ROME_SEND_ASSERV_TM_HTRAJ_DONE(UART_STRAT,xy,a))
#define TM_DL_HTRAJ_AUTOSET_DONE(b) _PRESCALE(TM_PRESCALER, ROME_SEND_ASSERV_TM_HTRAJ_AUTOSET_DONE(UART_STRAT,b))
#define TM_DL_HTRAJ_STATE(s) _PRESCALE(TM_PRESCALER, ROME_SEND_ASSERV_TM_HTRAJ_STATE(UART_STRAT,s))
#define TM_DL_HTRAJ_CARROT_XY(x,y) _PRESCALE(TM_PRESCALER, ROME_SEND_ASSERV_TM_HTRAJ_CARROT_XY(UART_STRAT,x,y))
#define TM_DL_HTRAJ_SPEED(v) _PRESCALE(TM_PRESCALER, ROME_SEND_ASSERV_TM_HTRAJ_SPEED(UART_STRAT,v))
#define TM_DL_HTRAJ_PATH_INDEX(i,n) _PRESCALE(TM_PRESCALER, ROME_SEND_ASSERV_TM_HTRAJ_PATH_INDEX(UART_STRAT,i,n))

#if 0
#define TM_DL_MOTORS(a,b,c) _PRESCALE(TM_PRESCALER, ROME_SEND_ASSERV_TM_MOTORS(UART_STRAT,a,b,c))

#define TM_DL_GP2_RAWS(a,b,c) _PRESCALE(TM_PRESCALER, ROME_SEND_ASSERV_TM_GP2_RAWS(UART_STRAT,a,b,c))
#define TM_DL_GP2_DET(a,b,c) _PRESCALE(TM_PRESCALER, ROME_SEND_ASSERV_TM_GP2_DET(UART_STRAT,a,b,c))
#else
#define TM_DL_MOTORS(a,b,c)

#define TM_DL_GP2_RAWS(a,b,c)
#define TM_DL_GP2_DET(a,b,c)
#endif

#define TM_DL_BUMPERS(a,b) _PRESCALE(TM_PRESCALER, ROME_SEND_ASSERV_TM_BUMPERS_STATE(UART_STRAT,a,b))

#define TM_DL_MATCH_TIMER(a) _PRESCALE(TM_PRESCALER, ROME_SEND_TM_MATCH_TIMER(UART_STRAT,ROME_DEVICE,a))

#define TM_DL_GYRO_CALIBRATION(a) _PRESCALE(TM_PRESCALER, ROME_SEND_ASSERV_TM_GYRO_CALIBRATION(UART_STRAT, a))

#endif//TELEMETRY_H
