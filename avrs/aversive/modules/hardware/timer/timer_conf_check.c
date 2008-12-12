/*  
 *  Copyright Droids Corporation, Microb Technology, Eirbot (2006)
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
 *
 *  Revision : $Id: timer_conf_check.c,v 1.3 2007-05-24 13:08:48 zer0 Exp $
 *
 */

#include <aversive/timers.h>

#include <timer.h>
#include <timer_definitions.h>
#include <timer_prescaler.h>

#include <timer_config.h>


#if defined TIMER0_ENABLED && ! defined TIMER0_AVAILABLE
#error This arch has no TIMER0
#endif

#if defined TIMER1_ENABLED && ! defined TIMER1_AVAILABLE
#error This arch has no TIMER1
#endif

#if defined TIMER2_ENABLED && ! defined TIMER2_AVAILABLE
#error This arch has no TIMER2
#endif

#if defined TIMER3_ENABLED && ! defined TIMER3_AVAILABLE
#error This arch has no TIMER3
#endif


#if defined TIMER0_ENABLED

#if defined TIMER0_PRESCALER_REG_0 && TIMER0_PRESCALER_REG_0 == TIMER0_PRESCALER_DIV
#define TIMER0_CONF_OK
#endif

#if defined TIMER0_PRESCALER_REG_1 && TIMER0_PRESCALER_REG_1 == TIMER0_PRESCALER_DIV
#define TIMER0_CONF_OK
#endif

#if defined TIMER0_PRESCALER_REG_2 && TIMER0_PRESCALER_REG_2 == TIMER0_PRESCALER_DIV
#define TIMER0_CONF_OK
#endif

#if defined TIMER0_PRESCALER_REG_3 && TIMER0_PRESCALER_REG_3 == TIMER0_PRESCALER_DIV
#define TIMER0_CONF_OK
#endif

#if defined TIMER0_PRESCALER_REG_4 && TIMER0_PRESCALER_REG_4 == TIMER0_PRESCALER_DIV
#define TIMER0_CONF_OK
#endif

#if defined TIMER0_PRESCALER_REG_5 && TIMER0_PRESCALER_REG_5 == TIMER0_PRESCALER_DIV
#define TIMER0_CONF_OK
#endif

#if defined TIMER0_PRESCALER_REG_6 && TIMER0_PRESCALER_REG_6 == TIMER0_PRESCALER_DIV
#define TIMER0_CONF_OK
#endif

#if defined TIMER0_PRESCALER_REG_7 && TIMER0_PRESCALER_REG_7 == TIMER0_PRESCALER_DIV
#define TIMER0_CONF_OK
#endif

#if defined TIMER0_PRESCALER_REG_8 && TIMER0_PRESCALER_REG_8 == TIMER0_PRESCALER_DIV
#define TIMER0_CONF_OK
#endif

#if defined TIMER0_PRESCALER_REG_9 && TIMER0_PRESCALER_REG_9 == TIMER0_PRESCALER_DIV
#define TIMER0_CONF_OK
#endif

#if defined TIMER0_PRESCALER_REG_10 && TIMER0_PRESCALER_REG_10 == TIMER0_PRESCALER_DIV
#define TIMER0_CONF_OK
#endif

#if defined TIMER0_PRESCALER_REG_11 && TIMER0_PRESCALER_REG_11 == TIMER0_PRESCALER_DIV
#define TIMER0_CONF_OK
#endif

#if defined TIMER0_PRESCALER_REG_12 && TIMER0_PRESCALER_REG_12 == TIMER0_PRESCALER_DIV
#define TIMER0_CONF_OK
#endif

#if defined TIMER0_PRESCALER_REG_13 && TIMER0_PRESCALER_REG_13 == TIMER0_PRESCALER_DIV
#define TIMER0_CONF_OK
#endif

#if defined TIMER0_PRESCALER_REG_14 && TIMER0_PRESCALER_REG_14 == TIMER0_PRESCALER_DIV
#define TIMER0_CONF_OK
#endif

#if defined TIMER0_PRESCALER_REG_15 && TIMER0_PRESCALER_REG_15 == TIMER0_PRESCALER_DIV
#define TIMER0_CONF_OK
#endif

#ifndef TIMER0_CONF_OK
#error TIMER0 has a bad prescaler value
#endif

#endif



#if defined TIMER1_ENABLED

#if defined TIMER1_PRESCALER_REG_0 && TIMER1_PRESCALER_REG_0 == TIMER1_PRESCALER_DIV
#define TIMER1_CONF_OK
#endif

#if defined TIMER1_PRESCALER_REG_1 && TIMER1_PRESCALER_REG_1 == TIMER1_PRESCALER_DIV
#define TIMER1_CONF_OK
#endif

#if defined TIMER1_PRESCALER_REG_2 && TIMER1_PRESCALER_REG_2 == TIMER1_PRESCALER_DIV
#define TIMER1_CONF_OK
#endif

#if defined TIMER1_PRESCALER_REG_3 && TIMER1_PRESCALER_REG_3 == TIMER1_PRESCALER_DIV
#define TIMER1_CONF_OK
#endif

#if defined TIMER1_PRESCALER_REG_4 && TIMER1_PRESCALER_REG_4 == TIMER1_PRESCALER_DIV
#define TIMER1_CONF_OK
#endif

#if defined TIMER1_PRESCALER_REG_5 && TIMER1_PRESCALER_REG_5 == TIMER1_PRESCALER_DIV
#define TIMER1_CONF_OK
#endif

#if defined TIMER1_PRESCALER_REG_6 && TIMER1_PRESCALER_REG_6 == TIMER1_PRESCALER_DIV
#define TIMER1_CONF_OK
#endif

#if defined TIMER1_PRESCALER_REG_7 && TIMER1_PRESCALER_REG_7 == TIMER1_PRESCALER_DIV
#define TIMER1_CONF_OK
#endif

#if defined TIMER1_PRESCALER_REG_8 && TIMER1_PRESCALER_REG_8 == TIMER1_PRESCALER_DIV
#define TIMER1_CONF_OK
#endif

#if defined TIMER1_PRESCALER_REG_9 && TIMER1_PRESCALER_REG_9 == TIMER1_PRESCALER_DIV
#define TIMER1_CONF_OK
#endif

#if defined TIMER1_PRESCALER_REG_10 && TIMER1_PRESCALER_REG_10 == TIMER1_PRESCALER_DIV
#define TIMER1_CONF_OK
#endif

#if defined TIMER1_PRESCALER_REG_11 && TIMER1_PRESCALER_REG_11 == TIMER1_PRESCALER_DIV
#define TIMER1_CONF_OK
#endif

#if defined TIMER1_PRESCALER_REG_12 && TIMER1_PRESCALER_REG_12 == TIMER1_PRESCALER_DIV
#define TIMER1_CONF_OK
#endif

#if defined TIMER1_PRESCALER_REG_13 && TIMER1_PRESCALER_REG_13 == TIMER1_PRESCALER_DIV
#define TIMER1_CONF_OK
#endif

#if defined TIMER1_PRESCALER_REG_14 && TIMER1_PRESCALER_REG_14 == TIMER1_PRESCALER_DIV
#define TIMER1_CONF_OK
#endif

#if defined TIMER1_PRESCALER_REG_15 && TIMER1_PRESCALER_REG_15 == TIMER1_PRESCALER_DIV
#define TIMER1_CONF_OK
#endif

#ifndef TIMER1_CONF_OK
#error TIMER1 has a bad prescaler value
#endif

#endif


#if defined TIMER2_ENABLED

#if defined TIMER2_PRESCALER_REG_0 && TIMER2_PRESCALER_REG_0 == TIMER2_PRESCALER_DIV
#define TIMER2_CONF_OK
#endif

#if defined TIMER2_PRESCALER_REG_1 && TIMER2_PRESCALER_REG_1 == TIMER2_PRESCALER_DIV
#define TIMER2_CONF_OK
#endif

#if defined TIMER2_PRESCALER_REG_2 && TIMER2_PRESCALER_REG_2 == TIMER2_PRESCALER_DIV
#define TIMER2_CONF_OK
#endif

#if defined TIMER2_PRESCALER_REG_3 && TIMER2_PRESCALER_REG_3 == TIMER2_PRESCALER_DIV
#define TIMER2_CONF_OK
#endif

#if defined TIMER2_PRESCALER_REG_4 && TIMER2_PRESCALER_REG_4 == TIMER2_PRESCALER_DIV
#define TIMER2_CONF_OK
#endif

#if defined TIMER2_PRESCALER_REG_5 && TIMER2_PRESCALER_REG_5 == TIMER2_PRESCALER_DIV
#define TIMER2_CONF_OK
#endif

#if defined TIMER2_PRESCALER_REG_6 && TIMER2_PRESCALER_REG_6 == TIMER2_PRESCALER_DIV
#define TIMER2_CONF_OK
#endif

#if defined TIMER2_PRESCALER_REG_7 && TIMER2_PRESCALER_REG_7 == TIMER2_PRESCALER_DIV
#define TIMER2_CONF_OK
#endif

#if defined TIMER2_PRESCALER_REG_8 && TIMER2_PRESCALER_REG_8 == TIMER2_PRESCALER_DIV
#define TIMER2_CONF_OK
#endif

#if defined TIMER2_PRESCALER_REG_9 && TIMER2_PRESCALER_REG_9 == TIMER2_PRESCALER_DIV
#define TIMER2_CONF_OK
#endif

#if defined TIMER2_PRESCALER_REG_10 && TIMER2_PRESCALER_REG_10 == TIMER2_PRESCALER_DIV
#define TIMER2_CONF_OK
#endif

#if defined TIMER2_PRESCALER_REG_11 && TIMER2_PRESCALER_REG_11 == TIMER2_PRESCALER_DIV
#define TIMER2_CONF_OK
#endif

#if defined TIMER2_PRESCALER_REG_12 && TIMER2_PRESCALER_REG_12 == TIMER2_PRESCALER_DIV
#define TIMER2_CONF_OK
#endif

#if defined TIMER2_PRESCALER_REG_13 && TIMER2_PRESCALER_REG_13 == TIMER2_PRESCALER_DIV
#define TIMER2_CONF_OK
#endif

#if defined TIMER2_PRESCALER_REG_14 && TIMER2_PRESCALER_REG_14 == TIMER2_PRESCALER_DIV
#define TIMER2_CONF_OK
#endif

#if defined TIMER2_PRESCALER_REG_15 && TIMER2_PRESCALER_REG_15 == TIMER2_PRESCALER_DIV
#define TIMER2_CONF_OK
#endif

#ifndef TIMER2_CONF_OK
#error TIMER2 has a bad prescaler value
#endif

#endif


#if defined TIMER3_ENABLED

#if defined TIMER3_PRESCALER_REG_0 && TIMER3_PRESCALER_REG_0 == TIMER3_PRESCALER_DIV
#define TIMER3_CONF_OK
#endif

#if defined TIMER3_PRESCALER_REG_1 && TIMER3_PRESCALER_REG_1 == TIMER3_PRESCALER_DIV
#define TIMER3_CONF_OK
#endif

#if defined TIMER3_PRESCALER_REG_2 && TIMER3_PRESCALER_REG_2 == TIMER3_PRESCALER_DIV
#define TIMER3_CONF_OK
#endif

#if defined TIMER3_PRESCALER_REG_3 && TIMER3_PRESCALER_REG_3 == TIMER3_PRESCALER_DIV
#define TIMER3_CONF_OK
#endif

#if defined TIMER3_PRESCALER_REG_4 && TIMER3_PRESCALER_REG_4 == TIMER3_PRESCALER_DIV
#define TIMER3_CONF_OK
#endif

#if defined TIMER3_PRESCALER_REG_5 && TIMER3_PRESCALER_REG_5 == TIMER3_PRESCALER_DIV
#define TIMER3_CONF_OK
#endif

#if defined TIMER3_PRESCALER_REG_6 && TIMER3_PRESCALER_REG_6 == TIMER3_PRESCALER_DIV
#define TIMER3_CONF_OK
#endif

#if defined TIMER3_PRESCALER_REG_7 && TIMER3_PRESCALER_REG_7 == TIMER3_PRESCALER_DIV
#define TIMER3_CONF_OK
#endif

#if defined TIMER3_PRESCALER_REG_8 && TIMER3_PRESCALER_REG_8 == TIMER3_PRESCALER_DIV
#define TIMER3_CONF_OK
#endif

#if defined TIMER3_PRESCALER_REG_9 && TIMER3_PRESCALER_REG_9 == TIMER3_PRESCALER_DIV
#define TIMER3_CONF_OK
#endif

#if defined TIMER3_PRESCALER_REG_10 && TIMER3_PRESCALER_REG_10 == TIMER3_PRESCALER_DIV
#define TIMER3_CONF_OK
#endif

#if defined TIMER3_PRESCALER_REG_11 && TIMER3_PRESCALER_REG_11 == TIMER3_PRESCALER_DIV
#define TIMER3_CONF_OK
#endif

#if defined TIMER3_PRESCALER_REG_12 && TIMER3_PRESCALER_REG_12 == TIMER3_PRESCALER_DIV
#define TIMER3_CONF_OK
#endif

#if defined TIMER3_PRESCALER_REG_13 && TIMER3_PRESCALER_REG_13 == TIMER3_PRESCALER_DIV
#define TIMER3_CONF_OK
#endif

#if defined TIMER3_PRESCALER_REG_14 && TIMER3_PRESCALER_REG_14 == TIMER3_PRESCALER_DIV
#define TIMER3_CONF_OK
#endif

#if defined TIMER3_PRESCALER_REG_15 && TIMER3_PRESCALER_REG_15 == TIMER3_PRESCALER_DIV
#define TIMER3_CONF_OK
#endif

#ifndef TIMER3_CONF_OK
#error TIMER3 has a bad prescaler value
#endif

#endif

