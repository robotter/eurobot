/*  
 *  Copyright Droids Corporation, Microb Technology, Eirbot (2005)
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
 *  Revision : $Id: control_system_manager.c,v 1.8 2008-01-08 20:05:03 zer0 Exp $
 *
 */
#include "control_system_manager.h"

#include <stdio.h>
#include <avarix/intlvl.h>

/** Call a filter() pointer : 
 * - lock the interrupts
 * - read the pointer to the filter function
 * - unlock the interrupts
 * - if pointer is null, return the IN value
 * - else apply filter
 */
static inline uint32_t
safe_filter(int32_t (*f)(void *, int32_t), void * param, int32_t value)
{
	int32_t (*f_tmp)(void *, int32_t);
	void * param_tmp;
  INTLVL_DISABLE_BLOCK(INTLVL_LO) {
  	f_tmp = f;
	  param_tmp = param;
  }
	if (f_tmp) {
		return f_tmp(param_tmp, value);
	}
	return value;
}

/** Call a processout() pointer : 
 * - lock the interrupts
 * - read the pointer to the processout function
 * - unlock the interrupts
 * - if pointer is null, return 0
 * - else return the value processed by the function
 */
static inline uint32_t
safe_getprocessout(int32_t (*f)(void *), void * param)
{
	int32_t (*f_tmp)(void *);
	void * param_tmp;
  INTLVL_DISABLE_BLOCK(INTLVL_LO) {
	  f_tmp = f;
  	param_tmp = param;
  }
	if (f_tmp) {
		return f_tmp(param_tmp);
	}
	return 0;
}

/** Call a processin() pointer : 
 * - lock the interrupts
 * - read the pointer to the processin function
 * - unlock the interrupts
 * - if pointer is null, don't do anything
 * - else call the processin with the parameters
 */
static inline void
safe_setprocessin(void (*f)(void *, int32_t), void * param, int32_t value)
{
	void (*f_tmp)(void *, int32_t);
	void * param_tmp;
  INTLVL_DISABLE_BLOCK(INTLVL_LO) {
	  f_tmp = f;
	  param_tmp = param;
  }
	if (f_tmp) {
		f_tmp(param_tmp, value);
	}
}

/**********************************************/

void cs_init(struct cs* cs)
{
  INTLVL_DISABLE_BLOCK(INTLVL_LO) {
    cs->consign_filter = NULL;
    cs->consign_filter_params = NULL;

    cs->correct_filter = NULL;
    cs->correct_filter_params = NULL;

    cs->feedback_filter = NULL;
    cs->feedback_filter_params = NULL;

    cs->process_out = NULL;
    cs->process_out_params = NULL;

    cs->process_in = NULL;
    cs->process_in_params = NULL;

    cs->consign_value = 0;
    cs->error_value = 0;
    cs->out_value = 0;
  }
  return;
}


void cs_set_consign_filter(struct cs* cs, int32_t (*consign_filter)(void*, int32_t), void* consign_filter_params)
{
  INTLVL_DISABLE_BLOCK(INTLVL_LO) {
    cs->consign_filter = consign_filter;
    cs->consign_filter_params = consign_filter_params;
  }
}



void cs_set_correct_filter(struct cs* cs, int32_t (*correct_filter)(void*, int32_t), void* correct_filter_params)
{
  INTLVL_DISABLE_BLOCK(INTLVL_LO) {
    cs->correct_filter = correct_filter;
    cs->correct_filter_params = correct_filter_params;
  }
}


void cs_set_feedback_filter(struct cs* cs, int32_t (*feedback_filter)(void*, int32_t), void* feedback_filter_params)
{
  INTLVL_DISABLE_BLOCK(INTLVL_LO) {
    cs->feedback_filter = feedback_filter;
    cs->feedback_filter_params = feedback_filter_params;
  }
}


void cs_set_process_in(struct cs* cs, void (*process_in)(void*, int32_t), void* process_in_params)
{  
  INTLVL_DISABLE_BLOCK(INTLVL_LO) {
    cs->process_in = process_in;
    cs->process_in_params = process_in_params;
  }
}



void cs_set_process_out(struct cs* cs, int32_t (*process_out)(void*), void* process_out_params)
{
  INTLVL_DISABLE_BLOCK(INTLVL_LO) {
    cs->process_out = process_out;
    cs->process_out_params = process_out_params;
  }
}



int32_t cs_do_process(struct cs* cs, int32_t consign)
{
  int32_t process_out_value = 0;
  /* save the consign value into the structure */
  cs->consign_value = consign;
  /* if the consign filter exist */
  cs->filtered_consign_value = consign = safe_filter(cs->consign_filter, cs->consign_filter_params, consign);
  /* read the process out if defined */
  process_out_value = safe_getprocessout(cs->process_out, cs->process_out_params);
  /* apply the feedback filter if defined */
  process_out_value = safe_filter(cs->feedback_filter, cs->feedback_filter_params, process_out_value);
  cs->filtered_feedback_value = process_out_value;
  /* substract consign and process out and put it into error */
  cs->error_value = cs->filtered_consign_value - process_out_value ;
  /* apply the correct filter to error_value and put it into out_value */
  cs->out_value = safe_filter(cs->correct_filter, cs->correct_filter_params, cs->error_value);
  /* send out_value to process in*/
  safe_setprocessin (cs->process_in, cs->process_in_params, cs->out_value);
  /* return the out value */
  return (cs->out_value);
}



void cs_manage(void * data)
{
  struct cs* cs = data;
  cs_do_process(cs, cs->consign_value);
}



int32_t cs_get_out(struct cs* cs)
{
  int32_t tmp;
  INTLVL_DISABLE_BLOCK(INTLVL_LO) {
    tmp = cs->out_value;
  }
  return tmp;
}



int32_t cs_get_error(struct cs* cs)
{
  int32_t tmp;
  INTLVL_DISABLE_BLOCK(INTLVL_LO) {
    tmp = cs->error_value;
  }
  return tmp;
}



int32_t cs_get_consign(struct cs* cs)
{
  int32_t tmp;
  INTLVL_DISABLE_BLOCK(INTLVL_LO) {
    tmp = cs->consign_value;
  }
  return tmp;
}

int32_t cs_get_filtered_consign(struct cs* cs)
{
  int32_t tmp;
  INTLVL_DISABLE_BLOCK(INTLVL_LO) {
    tmp = cs->filtered_consign_value;
  }
  return tmp;
}

int32_t cs_get_filtered_feedback(struct cs* cs)
{
  int32_t tmp;
  INTLVL_DISABLE_BLOCK(INTLVL_LO) {
    tmp = cs->filtered_feedback_value;
  }
  return tmp;
}



void cs_set_consign(struct cs* cs, int32_t v)
{
  INTLVL_DISABLE_BLOCK(INTLVL_LO) {
    /* set the consign */
    cs->consign_value = v;
  }
}        
