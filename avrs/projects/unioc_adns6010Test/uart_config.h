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
 *  Revision : $Id: uart_config.h,v 1.3 2005-11-07 23:23:34 zer0 Exp $
 *
 */

/* Droids-corp 2004 - Zer0
 * config for uart module
 */

#ifndef UART_CONFIG_H
#define UART_CONFIG_H

/*
 * UART1 definitions 
 */

/* compile UART1 fonctions, undefine it to pass compilation */
#define UART1_COMPILE  

/* enable UART1 if == 1, disable if == 0 */
#define UART1_ENABLED  1

/* enable UART1 interrupts if == 1, disable if == 0 */
#define UART1_INTERRUPT_ENABLED  1

#define UART1_BAUDRATE 38400

/* 
 * if you enable this, the maximum baudrate you can reach is 
 * higher, but the precision is lower. 
 */
#define UART1_USE_DOUBLE_SPEED 0
//#define UART1_USE_DOUBLE_SPEED 1

#define UART1_RX_FIFO_SIZE 4
#define UART1_TX_FIFO_SIZE 4
//#define UART1_NBITS 5
//#define UART1_NBITS 6
//#define UART1_NBITS 7
#define UART1_NBITS 8
//#define UART1_NBITS 9

#define UART1_PARITY UART_PARTITY_NONE
//#define UART1_PARITY UART_PARTITY_ODD
//#define UART1_PARITY UART_PARTITY_EVEN

#define UART1_STOP_BIT UART_STOP_BITS_1
//#define UART1_STOP_BIT UART_STOP_BITS_2




/* .... same for uart 1, 2, 3 ... */

#endif

