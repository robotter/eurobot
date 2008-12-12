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
 *  Revision : $Id $
 *
 */

/* WARNING : this file is automatically generated by scripts.
 * You should not edit it. If you find something wrong in it,
 * write to zer0@droids-corp.org */


/* prescalers timer 0 */
#define TIMER0_PRESCALER_DIV_0     0
#define TIMER0_PRESCALER_DIV_1     1
#define TIMER0_PRESCALER_DIV_8     2
#define TIMER0_PRESCALER_DIV_64    3
#define TIMER0_PRESCALER_DIV_256   4
#define TIMER0_PRESCALER_DIV_1024  5
#define TIMER0_PRESCALER_EXT_FALL  6
#define TIMER0_PRESCALER_EXT_RISE  7

#define TIMER0_PRESCALER_REG_0     0
#define TIMER0_PRESCALER_REG_1     1
#define TIMER0_PRESCALER_REG_2     8
#define TIMER0_PRESCALER_REG_3     64
#define TIMER0_PRESCALER_REG_4     256
#define TIMER0_PRESCALER_REG_5     1024
#define TIMER0_PRESCALER_REG_6     -1
#define TIMER0_PRESCALER_REG_7     -2


/* prescalers timer 1 */
#define TIMER1_PRESCALER_DIV_0     0
#define TIMER1_PRESCALER_DIV_1     1
#define TIMER1_PRESCALER_DIV_2     2
#define TIMER1_PRESCALER_DIV_4     3
#define TIMER1_PRESCALER_DIV_8     4
#define TIMER1_PRESCALER_DIV_16    5
#define TIMER1_PRESCALER_DIV_32    6
#define TIMER1_PRESCALER_DIV_64    7
#define TIMER1_PRESCALER_DIV_128   8
#define TIMER1_PRESCALER_DIV_256   9
#define TIMER1_PRESCALER_DIV_512   10
#define TIMER1_PRESCALER_DIV_1024  11

#define TIMER1_PRESCALER_REG_0     0
#define TIMER1_PRESCALER_REG_1     1
#define TIMER1_PRESCALER_REG_2     2
#define TIMER1_PRESCALER_REG_3     4
#define TIMER1_PRESCALER_REG_4     8
#define TIMER1_PRESCALER_REG_5     16
#define TIMER1_PRESCALER_REG_6     32
#define TIMER1_PRESCALER_REG_7     64
#define TIMER1_PRESCALER_REG_8     128
#define TIMER1_PRESCALER_REG_9     256
#define TIMER1_PRESCALER_REG_10     512
#define TIMER1_PRESCALER_REG_11     1024




/* available timers */
#define TIMER0_AVAILABLE
#define TIMER1_AVAILABLE

/* overflow interrupt number */
#define SIG_OVERFLOW0_NUM 0
#define SIG_OVERFLOW1_NUM 1
#define SIG_OVERFLOW_TOTAL_NUM 2

/* output compare interrupt number */
#define SIG_OUTPUT_COMPARE1_NUM 0
#define SIG_OUTPUT_COMPARE_TOTAL_NUM 1

/* Pwm nums */
#define PWM1_NUM 0
#define PWM_TOTAL_NUM 1

/* input capture interrupt number */
#define SIG_INPUT_CAPTURE_TOTAL_NUM 0


/* WDTCR */
#define WDP0_REG             WDTCR
#define WDP1_REG             WDTCR
#define WDP2_REG             WDTCR
#define WDE_REG              WDTCR
#define WDTOE_REG            WDTCR

/* GIMSK */
#define PCIE_REG             GIMSK
#define INT0_REG             GIMSK

/* ADMUX */
#define MUX0_REG             ADMUX
#define MUX1_REG             ADMUX
#define MUX2_REG             ADMUX
#define ADLAR_REG            ADMUX
#define REFS0_REG            ADMUX
#define REFS1_REG            ADMUX

/* TCCR1 */
#define CS10_REG             TCCR1
#define CS11_REG             TCCR1
#define CS12_REG             TCCR1
#define CS13_REG             TCCR1
#define COM1A0_REG           TCCR1
#define COM1A1_REG           TCCR1
#define PWM1_REG             TCCR1
#define CTC1_REG             TCCR1

/* TCCR0 */
#define CS00_REG             TCCR0
#define CS01_REG             TCCR0
#define CS02_REG             TCCR0

/* SREG */
#define C_REG                SREG
#define Z_REG                SREG
#define N_REG                SREG
#define V_REG                SREG
#define S_REG                SREG
#define H_REG                SREG
#define T_REG                SREG
#define I_REG                SREG

/* DDRB */
#define DDB0_REG             DDRB
#define DDB1_REG             DDRB
#define DDB2_REG             DDRB
#define DDB3_REG             DDRB
#define DDB4_REG             DDRB
#define DDB5_REG             DDRB

/* EEDR */
#define EEDR0_REG            EEDR
#define EEDR1_REG            EEDR
#define EEDR2_REG            EEDR
#define EEDR3_REG            EEDR
#define EEDR4_REG            EEDR
#define EEDR5_REG            EEDR
#define EEDR6_REG            EEDR
#define EEDR7_REG            EEDR

/* OCR1A */
#define OCR1A0_REG           OCR1A
#define OCR1A1_REG           OCR1A
#define OCR1A2_REG           OCR1A
#define OCR1A3_REG           OCR1A
#define OCR1A4_REG           OCR1A
#define OCR1A5_REG           OCR1A
#define OCR1A6_REG           OCR1A
#define OCR1A7_REG           OCR1A

/* GIFR */
#define PCIF_REG             GIFR
#define INTF0_REG            GIFR

/* TIMSK */
#define TOIE0_REG            TIMSK
#define TOIE1_REG            TIMSK
#define OCIE1A_REG           TIMSK

/* SFIOR */
#define PSR0_REG             SFIOR
#define PSR1_REG             SFIOR
#define FOC1A_REG            SFIOR

/* ACSR */
#define ACIS0_REG            ACSR
#define ACIS1_REG            ACSR
#define ACIE_REG             ACSR
#define ACI_REG              ACSR
#define ACO_REG              ACSR
#define ACBG_REG             ACSR
#define ACD_REG              ACSR

/* MCUSR */
#define PORF_REG             MCUSR
#define EXTRF_REG            MCUSR
#define BORF_REG             MCUSR
#define WDRF_REG             MCUSR

/* EECR */
#define EERE_REG             EECR
#define EEWE_REG             EECR
#define EEMWE_REG            EECR
#define EERIE_REG            EECR

/* OSCCAL */
#define CAL0_REG             OSCCAL
#define CAL1_REG             OSCCAL
#define CAL2_REG             OSCCAL
#define CAL3_REG             OSCCAL
#define CAL4_REG             OSCCAL
#define CAL5_REG             OSCCAL
#define CAL6_REG             OSCCAL
#define CAL7_REG             OSCCAL

/* ADCL */
#define ADCL0_REG            ADCL
#define ADCL1_REG            ADCL
#define ADCL2_REG            ADCL
#define ADCL3_REG            ADCL
#define ADCL4_REG            ADCL
#define ADCL5_REG            ADCL
#define ADCL6_REG            ADCL
#define ADCL7_REG            ADCL

/* EEAR */
#define EEAR0_REG            EEAR
#define EEAR1_REG            EEAR
#define EEAR2_REG            EEAR
#define EEAR3_REG            EEAR
#define EEAR4_REG            EEAR
#define EEAR5_REG            EEAR

/* PORTB */
#define PORTB0_REG           PORTB
#define PORTB1_REG           PORTB
#define PORTB2_REG           PORTB
#define PORTB3_REG           PORTB
#define PORTB4_REG           PORTB

/* ADCH */
#define ADCH0_REG            ADCH
#define ADCH1_REG            ADCH
#define ADCH2_REG            ADCH
#define ADCH3_REG            ADCH
#define ADCH4_REG            ADCH
#define ADCH5_REG            ADCH
#define ADCH6_REG            ADCH
#define ADCH7_REG            ADCH

/* TCNT0 */
#define TCNT00_REG           TCNT0
#define TCNT01_REG           TCNT0
#define TCNT02_REG           TCNT0
#define TCNT03_REG           TCNT0
#define TCNT04_REG           TCNT0
#define TCNT05_REG           TCNT0
#define TCNT06_REG           TCNT0
#define TCNT07_REG           TCNT0

/* TCNT1 */
#define TCNT1_0_REG          TCNT1
#define TCNT1_1_REG          TCNT1
#define TCNT1_2_REG          TCNT1
#define TCNT1_3_REG          TCNT1
#define TCNT1_4_REG          TCNT1
#define TCNT1_5_REG          TCNT1
#define TCNT1_6_REG          TCNT1
#define TCNT1_7_REG          TCNT1

/* TIFR */
#define TOV0_REG             TIFR
#define TOV1_REG             TIFR
#define OCF1A_REG            TIFR

/* ADCSR */
#define ADPS0_REG            ADCSR
#define ADPS1_REG            ADCSR
#define ADPS2_REG            ADCSR
#define ADIE_REG             ADCSR
#define ADIF_REG             ADCSR
#define ADFR_REG             ADCSR
#define ADSC_REG             ADCSR
#define ADEN_REG             ADCSR

/* PINB */
#define PINB0_REG            PINB
#define PINB1_REG            PINB
#define PINB2_REG            PINB
#define PINB3_REG            PINB
#define PINB4_REG            PINB
#define PINB5_REG            PINB

/* OCR1B */
#define OCR1B0_REG           OCR1B
#define OCR1B1_REG           OCR1B
#define OCR1B2_REG           OCR1B
#define OCR1B3_REG           OCR1B
#define OCR1B4_REG           OCR1B
#define OCR1B5_REG           OCR1B
#define OCR1B6_REG           OCR1B
#define OCR1B7_REG           OCR1B

/* MCUCR */
#define ISC00_REG            MCUCR
#define ISC01_REG            MCUCR
#define SM0_REG              MCUCR
#define SM1_REG              MCUCR
#define SE_REG               MCUCR
#define PUD_REG              MCUCR
