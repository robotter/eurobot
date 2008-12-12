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
#define TIMER0_PRESCALER_DIV_32    3
#define TIMER0_PRESCALER_DIV_64    4
#define TIMER0_PRESCALER_DIV_128   5
#define TIMER0_PRESCALER_DIV_256   6
#define TIMER0_PRESCALER_DIV_1024  7

#define TIMER0_PRESCALER_REG_0     0
#define TIMER0_PRESCALER_REG_1     1
#define TIMER0_PRESCALER_REG_2     8
#define TIMER0_PRESCALER_REG_3     32
#define TIMER0_PRESCALER_REG_4     64
#define TIMER0_PRESCALER_REG_5     128
#define TIMER0_PRESCALER_REG_6     256
#define TIMER0_PRESCALER_REG_7     1024


/* prescalers timer 1 */
#define TIMER1_PRESCALER_DIV_0     0
#define TIMER1_PRESCALER_DIV_1     1
#define TIMER1_PRESCALER_DIV_8     2
#define TIMER1_PRESCALER_DIV_64    3
#define TIMER1_PRESCALER_DIV_256   4
#define TIMER1_PRESCALER_DIV_1024  5
#define TIMER1_PRESCALER_EXT_FALL  6
#define TIMER1_PRESCALER_EXT_RISE  7

#define TIMER1_PRESCALER_REG_0     0
#define TIMER1_PRESCALER_REG_1     1
#define TIMER1_PRESCALER_REG_2     8
#define TIMER1_PRESCALER_REG_3     64
#define TIMER1_PRESCALER_REG_4     256
#define TIMER1_PRESCALER_REG_5     1024
#define TIMER1_PRESCALER_REG_6     -1
#define TIMER1_PRESCALER_REG_7     -2


/* prescalers timer 2 */
#define TIMER2_PRESCALER_DIV_0     0
#define TIMER2_PRESCALER_DIV_1     1
#define TIMER2_PRESCALER_DIV_8     2
#define TIMER2_PRESCALER_DIV_64    3
#define TIMER2_PRESCALER_DIV_256   4
#define TIMER2_PRESCALER_DIV_1024  5

#define TIMER2_PRESCALER_REG_0     0
#define TIMER2_PRESCALER_REG_1     1
#define TIMER2_PRESCALER_REG_2     8
#define TIMER2_PRESCALER_REG_3     64
#define TIMER2_PRESCALER_REG_4     256
#define TIMER2_PRESCALER_REG_5     1024


/* prescalers timer 3 */
#define TIMER3_PRESCALER_DIV_0     0
#define TIMER3_PRESCALER_DIV_1     1
#define TIMER3_PRESCALER_DIV_8     2
#define TIMER3_PRESCALER_DIV_64    3
#define TIMER3_PRESCALER_DIV_256   4
#define TIMER3_PRESCALER_DIV_1024  5
#define TIMER3_PRESCALER_EXT_FALL  6
#define TIMER3_PRESCALER_EXT_RISE  7

#define TIMER3_PRESCALER_REG_0     0
#define TIMER3_PRESCALER_REG_1     1
#define TIMER3_PRESCALER_REG_2     8
#define TIMER3_PRESCALER_REG_3     64
#define TIMER3_PRESCALER_REG_4     256
#define TIMER3_PRESCALER_REG_5     1024
#define TIMER3_PRESCALER_REG_6     -1
#define TIMER3_PRESCALER_REG_7     -2




/* available timers */
#define TIMER0_AVAILABLE
#define TIMER1_AVAILABLE
#define TIMER1A_AVAILABLE
#define TIMER1B_AVAILABLE
#define TIMER1C_AVAILABLE
#define TIMER2_AVAILABLE
#define TIMER3_AVAILABLE
#define TIMER3A_AVAILABLE
#define TIMER3B_AVAILABLE
#define TIMER3C_AVAILABLE

/* overflow interrupt number */
#define SIG_OVERFLOW0_NUM 0
#define SIG_OVERFLOW1_NUM 1
#define SIG_OVERFLOW2_NUM 2
#define SIG_OVERFLOW3_NUM 3
#define SIG_OVERFLOW_TOTAL_NUM 4

/* output compare interrupt number */
#define SIG_OUTPUT_COMPARE0_NUM 0
#define SIG_OUTPUT_COMPARE1A_NUM 1
#define SIG_OUTPUT_COMPARE1B_NUM 2
#define SIG_OUTPUT_COMPARE1C_NUM 3
#define SIG_OUTPUT_COMPARE2_NUM 4
#define SIG_OUTPUT_COMPARE3A_NUM 5
#define SIG_OUTPUT_COMPARE3B_NUM 6
#define SIG_OUTPUT_COMPARE3C_NUM 7
#define SIG_OUTPUT_COMPARE_TOTAL_NUM 8

/* Pwm nums */
#define PWM0_NUM 0
#define PWM1A_NUM 1
#define PWM1B_NUM 2
#define PWM1C_NUM 3
#define PWM2_NUM 4
#define PWM3A_NUM 5
#define PWM3B_NUM 6
#define PWM3C_NUM 7
#define PWM_TOTAL_NUM 8

/* input capture interrupt number */
#define SIG_INPUT_CAPTURE1_NUM 0
#define SIG_INPUT_CAPTURE3_NUM 1
#define SIG_INPUT_CAPTURE_TOTAL_NUM 2


/* WDTCR */
#define WDP0_REG             WDTCR
#define WDP1_REG             WDTCR
#define WDP2_REG             WDTCR
#define WDE_REG              WDTCR
#define WDCE_REG             WDTCR

/* ADMUX */
#define MUX0_REG             ADMUX
#define MUX1_REG             ADMUX
#define MUX2_REG             ADMUX
#define MUX3_REG             ADMUX
#define MUX4_REG             ADMUX
#define ADLAR_REG            ADMUX
#define REFS0_REG            ADMUX
#define REFS1_REG            ADMUX

/* EEDR */
#define EEDR0_REG            EEDR
#define EEDR1_REG            EEDR
#define EEDR2_REG            EEDR
#define EEDR3_REG            EEDR
#define EEDR4_REG            EEDR
#define EEDR5_REG            EEDR
#define EEDR6_REG            EEDR
#define EEDR7_REG            EEDR

/* RAMPZ */
#define RAMPZ0_REG           RAMPZ

/* SPDR */
#define SPDR0_REG            SPDR
#define SPDR1_REG            SPDR
#define SPDR2_REG            SPDR
#define SPDR3_REG            SPDR
#define SPDR4_REG            SPDR
#define SPDR5_REG            SPDR
#define SPDR6_REG            SPDR
#define SPDR7_REG            SPDR

/* SPSR */
#define SPI2X_REG            SPSR
#define WCOL_REG             SPSR
#define SPIF_REG             SPSR

/* ICR1H */
#define ICR1H0_REG           ICR1H
#define ICR1H1_REG           ICR1H
#define ICR1H2_REG           ICR1H
#define ICR1H3_REG           ICR1H
#define ICR1H4_REG           ICR1H
#define ICR1H5_REG           ICR1H
#define ICR1H6_REG           ICR1H
#define ICR1H7_REG           ICR1H

/* SPL */
#define SP0_REG              SPL
#define SP1_REG              SPL
#define SP2_REG              SPL
#define SP3_REG              SPL
#define SP4_REG              SPL
#define SP5_REG              SPL
#define SP6_REG              SPL
#define SP7_REG              SPL

/* DDRD */
#define DDD0_REG             DDRD
#define DDD1_REG             DDRD
#define DDD2_REG             DDRD
#define DDD3_REG             DDRD
#define DDD4_REG             DDRD
#define DDD5_REG             DDRD
#define DDD6_REG             DDRD
#define DDD7_REG             DDRD

/* TWSR */
#define TWPS0_REG            TWSR
#define TWPS1_REG            TWSR
#define TWS3_REG             TWSR
#define TWS4_REG             TWSR
#define TWS5_REG             TWSR
#define TWS6_REG             TWSR
#define TWS7_REG             TWSR

/* TCNT1L */
#define TCNT1L0_REG          TCNT1L
#define TCNT1L1_REG          TCNT1L
#define TCNT1L2_REG          TCNT1L
#define TCNT1L3_REG          TCNT1L
#define TCNT1L4_REG          TCNT1L
#define TCNT1L5_REG          TCNT1L
#define TCNT1L6_REG          TCNT1L
#define TCNT1L7_REG          TCNT1L

/* PORTG */
#define PORTG0_REG           PORTG
#define PORTG1_REG           PORTG
#define PORTG2_REG           PORTG
#define PORTG3_REG           PORTG
#define PORTG4_REG           PORTG

/* UCSR0C */
#define UCPOL0_REG           UCSR0C
#define UCSZ00_REG           UCSR0C
#define UCSZ01_REG           UCSR0C
#define USBS0_REG            UCSR0C
#define UPM00_REG            UCSR0C
#define UPM01_REG            UCSR0C
#define UMSEL0_REG           UCSR0C

/* UCSR0B */
#define TXB80_REG            UCSR0B
#define RXB80_REG            UCSR0B
#define UCSZ02_REG           UCSR0B
#define TXEN0_REG            UCSR0B
#define RXEN0_REG            UCSR0B
#define UDRIE0_REG           UCSR0B
#define TXCIE0_REG           UCSR0B
#define RXCIE0_REG           UCSR0B

/* PORTB */
#define PORTB0_REG           PORTB
#define PORTB1_REG           PORTB
#define PORTB2_REG           PORTB
#define PORTB3_REG           PORTB
#define PORTB4_REG           PORTB
#define PORTB5_REG           PORTB
#define PORTB6_REG           PORTB
#define PORTB7_REG           PORTB

/* PORTC */
#define PORTC0_REG           PORTC
#define PORTC1_REG           PORTC
#define PORTC2_REG           PORTC
#define PORTC3_REG           PORTC
#define PORTC4_REG           PORTC
#define PORTC5_REG           PORTC
#define PORTC6_REG           PORTC
#define PORTC7_REG           PORTC

/* PORTA */
#define PORTA0_REG           PORTA
#define PORTA1_REG           PORTA
#define PORTA2_REG           PORTA
#define PORTA3_REG           PORTA
#define PORTA4_REG           PORTA
#define PORTA5_REG           PORTA
#define PORTA6_REG           PORTA
#define PORTA7_REG           PORTA

/* UDR1 */
#define UDR10_REG            UDR1
#define UDR11_REG            UDR1
#define UDR12_REG            UDR1
#define UDR13_REG            UDR1
#define UDR14_REG            UDR1
#define UDR15_REG            UDR1
#define UDR16_REG            UDR1
#define UDR17_REG            UDR1

/* UDR0 */
#define UDR00_REG            UDR0
#define UDR01_REG            UDR0
#define UDR02_REG            UDR0
#define UDR03_REG            UDR0
#define UDR04_REG            UDR0
#define UDR05_REG            UDR0
#define UDR06_REG            UDR0
#define UDR07_REG            UDR0

/* EICRB */
#define ISC40_REG            EICRB
#define ISC41_REG            EICRB
#define ISC50_REG            EICRB
#define ISC51_REG            EICRB
#define ISC60_REG            EICRB
#define ISC61_REG            EICRB
#define ISC70_REG            EICRB
#define ISC71_REG            EICRB

/* EICRA */
#define ISC00_REG            EICRA
#define ISC01_REG            EICRA
#define ISC10_REG            EICRA
#define ISC11_REG            EICRA
#define ISC20_REG            EICRA
#define ISC21_REG            EICRA
#define ISC30_REG            EICRA
#define ISC31_REG            EICRA

/* ASSR */
#define TCR0UB_REG           ASSR
#define OCR0UB_REG           ASSR
#define TCN0UB_REG           ASSR
#define AS0_REG              ASSR

/* SREG */
#define C_REG                SREG
#define Z_REG                SREG
#define N_REG                SREG
#define V_REG                SREG
#define S_REG                SREG
#define H_REG                SREG
#define T_REG                SREG
#define I_REG                SREG

/* UBRR1L */
/* #define UBRR0_REG            UBRR1L */ /* dup in UBRR0L */
/* #define UBRR1_REG            UBRR1L */ /* dup in UBRR0L */
/* #define UBRR2_REG            UBRR1L */ /* dup in UBRR0L */
/* #define UBRR3_REG            UBRR1L */ /* dup in UBRR0L */
/* #define UBRR4_REG            UBRR1L */ /* dup in UBRR0L */
/* #define UBRR5_REG            UBRR1L */ /* dup in UBRR0L */
/* #define UBRR6_REG            UBRR1L */ /* dup in UBRR0L */
/* #define UBRR7_REG            UBRR1L */ /* dup in UBRR0L */

/* DDRC */
#define DDC0_REG             DDRC
#define DDC1_REG             DDRC
#define DDC2_REG             DDRC
#define DDC3_REG             DDRC
#define DDC4_REG             DDRC
#define DDC5_REG             DDRC
#define DDC6_REG             DDRC
#define DDC7_REG             DDRC

/* OCR3AL */
#define OCR3AL0_REG          OCR3AL
#define OCR3AL1_REG          OCR3AL
#define OCR3AL2_REG          OCR3AL
#define OCR3AL3_REG          OCR3AL
#define OCR3AL4_REG          OCR3AL
#define OCR3AL5_REG          OCR3AL
#define OCR3AL6_REG          OCR3AL
#define OCR3AL7_REG          OCR3AL

/* DDRA */
#define DDA0_REG             DDRA
#define DDA1_REG             DDRA
#define DDA2_REG             DDRA
#define DDA3_REG             DDRA
#define DDA4_REG             DDRA
#define DDA5_REG             DDRA
#define DDA6_REG             DDRA
#define DDA7_REG             DDRA

/* DDRF */
#define DDF0_REG             DDRF
#define DDF1_REG             DDRF
#define DDF2_REG             DDRF
#define DDF3_REG             DDRF
#define DDF4_REG             DDRF
#define DDF5_REG             DDRF
#define DDF6_REG             DDRF
#define DDF7_REG             DDRF

/* DDRG */
#define DDG0_REG             DDRG
#define DDG1_REG             DDRG
#define DDG2_REG             DDRG
#define DDG3_REG             DDRG
#define DDG4_REG             DDRG

/* OCR3AH */
#define OCR3AH0_REG          OCR3AH
#define OCR3AH1_REG          OCR3AH
#define OCR3AH2_REG          OCR3AH
#define OCR3AH3_REG          OCR3AH
#define OCR3AH4_REG          OCR3AH
#define OCR3AH5_REG          OCR3AH
#define OCR3AH6_REG          OCR3AH
#define OCR3AH7_REG          OCR3AH

/* TCCR1B */
#define CS10_REG             TCCR1B
#define CS11_REG             TCCR1B
#define CS12_REG             TCCR1B
#define WGM12_REG            TCCR1B
#define WGM13_REG            TCCR1B
#define ICES1_REG            TCCR1B
#define ICNC1_REG            TCCR1B

/* OSCCAL */
#define CAL0_REG             OSCCAL
#define CAL1_REG             OSCCAL
#define CAL2_REG             OSCCAL
#define CAL3_REG             OSCCAL
#define CAL4_REG             OSCCAL
#define CAL5_REG             OSCCAL
#define CAL6_REG             OSCCAL
#define CAL7_REG             OSCCAL

/* SFIOR */
#define ACME_REG             SFIOR
#define PSR321_REG           SFIOR
#define PSR0_REG             SFIOR
#define PUD_REG              SFIOR
#define TSM_REG              SFIOR

/* TCNT2 */
#define TCNT2_0_REG          TCNT2
#define TCNT2_1_REG          TCNT2
#define TCNT2_2_REG          TCNT2
#define TCNT2_3_REG          TCNT2
#define TCNT2_4_REG          TCNT2
#define TCNT2_5_REG          TCNT2
#define TCNT2_6_REG          TCNT2
#define TCNT2_7_REG          TCNT2

/* UBRR1H */
/* #define UBRR8_REG            UBRR1H */ /* dup in UBRR0H */
/* #define UBRR9_REG            UBRR1H */ /* dup in UBRR0H */
/* #define UBRR10_REG           UBRR1H */ /* dup in UBRR0H */
/* #define UBRR11_REG           UBRR1H */ /* dup in UBRR0H */

/* TWAR */
#define TWGCE_REG            TWAR
#define TWA0_REG             TWAR
#define TWA1_REG             TWAR
#define TWA2_REG             TWAR
#define TWA3_REG             TWAR
#define TWA4_REG             TWAR
#define TWA5_REG             TWAR
#define TWA6_REG             TWAR

/* TIFR */
#define TOV0_REG             TIFR
#define OCF0_REG             TIFR
#define TOV1_REG             TIFR
#define OCF1B_REG            TIFR
#define OCF1A_REG            TIFR
#define ICF1_REG             TIFR
#define TOV2_REG             TIFR
#define OCF2_REG             TIFR

/* TCNT0 */
#define TCNT0_0_REG          TCNT0
#define TCNT0_1_REG          TCNT0
#define TCNT0_2_REG          TCNT0
#define TCNT0_3_REG          TCNT0
#define TCNT0_4_REG          TCNT0
#define TCNT0_5_REG          TCNT0
#define TCNT0_6_REG          TCNT0
#define TCNT0_7_REG          TCNT0

/* ETIFR */
#define OCF1C_REG            ETIFR
#define OCF3C_REG            ETIFR
#define TOV3_REG             ETIFR
#define OCF3B_REG            ETIFR
#define OCF3A_REG            ETIFR
#define ICF3_REG             ETIFR

/* SPCR */
#define SPR0_REG             SPCR
#define SPR1_REG             SPCR
#define CPHA_REG             SPCR
#define CPOL_REG             SPCR
#define MSTR_REG             SPCR
#define DORD_REG             SPCR
#define SPE_REG              SPCR
#define SPIE_REG             SPCR

/* XDIV */
#define XDIV0_REG            XDIV
#define XDIV1_REG            XDIV
#define XDIV2_REG            XDIV
#define XDIV3_REG            XDIV
#define XDIV4_REG            XDIV
#define XDIV5_REG            XDIV
#define XDIV6_REG            XDIV
#define XDIVEN_REG           XDIV

/* OCR3CH */
#define OCR3CH0_REG          OCR3CH
#define OCR3CH1_REG          OCR3CH
#define OCR3CH2_REG          OCR3CH
#define OCR3CH3_REG          OCR3CH
#define OCR3CH4_REG          OCR3CH
#define OCR3CH5_REG          OCR3CH
#define OCR3CH6_REG          OCR3CH
#define OCR3CH7_REG          OCR3CH

/* ETIMSK */
#define OCIE1C_REG           ETIMSK
#define OCIE3C_REG           ETIMSK
#define TOIE3_REG            ETIMSK
#define OCIE3B_REG           ETIMSK
#define OCIE3A_REG           ETIMSK
#define TICIE3_REG           ETIMSK

/* OCR3CL */
#define OCR3CL0_REG          OCR3CL
#define OCR3CL1_REG          OCR3CL
#define OCR3CL2_REG          OCR3CL
#define OCR3CL3_REG          OCR3CL
#define OCR3CL4_REG          OCR3CL
#define OCR3CL5_REG          OCR3CL
#define OCR3CL6_REG          OCR3CL
#define OCR3CL7_REG          OCR3CL

/* TWBR */
#define TWBR0_REG            TWBR
#define TWBR1_REG            TWBR
#define TWBR2_REG            TWBR
#define TWBR3_REG            TWBR
#define TWBR4_REG            TWBR
#define TWBR5_REG            TWBR
#define TWBR6_REG            TWBR
#define TWBR7_REG            TWBR

/* SPH */
#define SP8_REG              SPH
#define SP9_REG              SPH
#define SP10_REG             SPH
#define SP11_REG             SPH
#define SP12_REG             SPH
#define SP13_REG             SPH
#define SP14_REG             SPH
#define SP15_REG             SPH

/* TCCR3C */
#define FOC3C_REG            TCCR3C
#define FOC3B_REG            TCCR3C
#define FOC3A_REG            TCCR3C

/* TCCR3B */
#define CS30_REG             TCCR3B
#define CS31_REG             TCCR3B
#define CS32_REG             TCCR3B
#define WGM32_REG            TCCR3B
#define WGM33_REG            TCCR3B
#define ICES3_REG            TCCR3B
#define ICNC3_REG            TCCR3B

/* TCCR3A */
#define WGM30_REG            TCCR3A
#define WGM31_REG            TCCR3A
#define COM3C0_REG           TCCR3A
#define COM3C1_REG           TCCR3A
#define COM3B0_REG           TCCR3A
#define COM3B1_REG           TCCR3A
#define COM3A0_REG           TCCR3A
#define COM3A1_REG           TCCR3A

/* OCR1BL */
#define OCR1BL0_REG          OCR1BL
#define OCR1BL1_REG          OCR1BL
#define OCR1BL2_REG          OCR1BL
#define OCR1BL3_REG          OCR1BL
#define OCR1BL4_REG          OCR1BL
#define OCR1BL5_REG          OCR1BL
#define OCR1BL6_REG          OCR1BL
#define OCR1BL7_REG          OCR1BL

/* TCNT3H */
#define TCNT3H0_REG          TCNT3H
#define TCNT3H1_REG          TCNT3H
#define TCNT3H2_REG          TCNT3H
#define TCNT3H3_REG          TCNT3H
#define TCNT3H4_REG          TCNT3H
#define TCNT3H5_REG          TCNT3H
#define TCNT3H6_REG          TCNT3H
#define TCNT3H7_REG          TCNT3H

/* OCR1BH */
#define OCR1BH0_REG          OCR1BH
#define OCR1BH1_REG          OCR1BH
#define OCR1BH2_REG          OCR1BH
#define OCR1BH3_REG          OCR1BH
#define OCR1BH4_REG          OCR1BH
#define OCR1BH5_REG          OCR1BH
#define OCR1BH6_REG          OCR1BH
#define OCR1BH7_REG          OCR1BH

/* TCNT3L */
#define TCN3L0_REG           TCNT3L
#define TCN3L1_REG           TCNT3L
#define TCN3L2_REG           TCNT3L
#define TCN3L3_REG           TCNT3L
#define TCN3L4_REG           TCNT3L
#define TCN3L5_REG           TCNT3L
#define TCN3L6_REG           TCNT3L
#define TCN3L7_REG           TCNT3L

/* ICR1L */
#define ICR1L0_REG           ICR1L
#define ICR1L1_REG           ICR1L
#define ICR1L2_REG           ICR1L
#define ICR1L3_REG           ICR1L
#define ICR1L4_REG           ICR1L
#define ICR1L5_REG           ICR1L
#define ICR1L6_REG           ICR1L
#define ICR1L7_REG           ICR1L

/* EECR */
#define EERE_REG             EECR
#define EEWE_REG             EECR
#define EEMWE_REG            EECR
#define EERIE_REG            EECR

/* TWCR */
#define TWIE_REG             TWCR
#define TWEN_REG             TWCR
#define TWWC_REG             TWCR
#define TWSTO_REG            TWCR
#define TWSTA_REG            TWCR
#define TWEA_REG             TWCR
#define TWINT_REG            TWCR

/* MCUCSR */
#define PORF_REG             MCUCSR
#define EXTRF_REG            MCUCSR
#define BORF_REG             MCUCSR
#define WDRF_REG             MCUCSR
#define JTRF_REG             MCUCSR
#define JTD_REG              MCUCSR

/* UCSR0A */
#define MPCM0_REG            UCSR0A
#define U2X0_REG             UCSR0A
#define UPE0_REG             UCSR0A
#define DOR0_REG             UCSR0A
#define FE0_REG              UCSR0A
#define UDRE0_REG            UCSR0A
#define TXC0_REG             UCSR0A
#define RXC0_REG             UCSR0A

/* UBRR0H */
/* #define UBRR8_REG            UBRR0H */ /* dup in UBRR1H */
/* #define UBRR9_REG            UBRR0H */ /* dup in UBRR1H */
/* #define UBRR10_REG           UBRR0H */ /* dup in UBRR1H */
/* #define UBRR11_REG           UBRR0H */ /* dup in UBRR1H */

/* UBRR0L */
/* #define UBRR0_REG            UBRR0L */ /* dup in UBRR1L */
/* #define UBRR1_REG            UBRR0L */ /* dup in UBRR1L */
/* #define UBRR2_REG            UBRR0L */ /* dup in UBRR1L */
/* #define UBRR3_REG            UBRR0L */ /* dup in UBRR1L */
/* #define UBRR4_REG            UBRR0L */ /* dup in UBRR1L */
/* #define UBRR5_REG            UBRR0L */ /* dup in UBRR1L */
/* #define UBRR6_REG            UBRR0L */ /* dup in UBRR1L */
/* #define UBRR7_REG            UBRR0L */ /* dup in UBRR1L */

/* EEARH */
#define EEAR8_REG            EEARH
#define EEAR9_REG            EEARH
#define EEAR10_REG           EEARH
#define EEAR11_REG           EEARH

/* EEARL */
#define EEARL0_REG           EEARL
#define EEARL1_REG           EEARL
#define EEARL2_REG           EEARL
#define EEARL3_REG           EEARL
#define EEARL4_REG           EEARL
#define EEARL5_REG           EEARL
#define EEARL6_REG           EEARL
#define EEARL7_REG           EEARL

/* MCUCR */
#define IVCE_REG             MCUCR
#define IVSEL_REG            MCUCR
#define SM2_REG              MCUCR
#define SM0_REG              MCUCR
#define SM1_REG              MCUCR
#define SE_REG               MCUCR
#define SRW10_REG            MCUCR
#define SRE_REG              MCUCR

/* OCR1CL */
#define OCR1CL0_REG          OCR1CL
#define OCR1CL1_REG          OCR1CL
#define OCR1CL2_REG          OCR1CL
#define OCR1CL3_REG          OCR1CL
#define OCR1CL4_REG          OCR1CL
#define OCR1CL5_REG          OCR1CL
#define OCR1CL6_REG          OCR1CL
#define OCR1CL7_REG          OCR1CL

/* OCR1CH */
#define OCR1CH0_REG          OCR1CH
#define OCR1CH1_REG          OCR1CH
#define OCR1CH2_REG          OCR1CH
#define OCR1CH3_REG          OCR1CH
#define OCR1CH4_REG          OCR1CH
#define OCR1CH5_REG          OCR1CH
#define OCR1CH6_REG          OCR1CH
#define OCR1CH7_REG          OCR1CH

/* OCDR */
#define OCDR0_REG            OCDR
#define OCDR1_REG            OCDR
#define OCDR2_REG            OCDR
#define OCDR3_REG            OCDR
#define OCDR4_REG            OCDR
#define OCDR5_REG            OCDR
#define OCDR6_REG            OCDR
#define OCDR7_REG            OCDR

/* EIFR */
#define INTF0_REG            EIFR
#define INTF1_REG            EIFR
#define INTF2_REG            EIFR
#define INTF3_REG            EIFR
#define INTF4_REG            EIFR
#define INTF5_REG            EIFR
#define INTF6_REG            EIFR
#define INTF7_REG            EIFR

/* UCSR1B */
#define TXB81_REG            UCSR1B
#define RXB81_REG            UCSR1B
#define UCSZ12_REG           UCSR1B
#define TXEN1_REG            UCSR1B
#define RXEN1_REG            UCSR1B
#define UDRIE1_REG           UCSR1B
#define TXCIE1_REG           UCSR1B
#define RXCIE1_REG           UCSR1B

/* UCSR1C */
#define UCPOL1_REG           UCSR1C
#define UCSZ10_REG           UCSR1C
#define UCSZ11_REG           UCSR1C
#define USBS1_REG            UCSR1C
#define UPM10_REG            UCSR1C
#define UPM11_REG            UCSR1C
#define UMSEL1_REG           UCSR1C

/* UCSR1A */
#define MPCM1_REG            UCSR1A
#define U2X1_REG             UCSR1A
#define UPE1_REG             UCSR1A
#define DOR1_REG             UCSR1A
#define FE1_REG              UCSR1A
#define UDRE1_REG            UCSR1A
#define TXC1_REG             UCSR1A
#define RXC1_REG             UCSR1A

/* TCCR0 */
#define CS00_REG             TCCR0
#define CS01_REG             TCCR0
#define CS02_REG             TCCR0
#define WGM01_REG            TCCR0
#define COM00_REG            TCCR0
#define COM01_REG            TCCR0
#define WGM00_REG            TCCR0
#define FOC0_REG             TCCR0

/* TCCR2 */
#define CS20_REG             TCCR2
#define CS21_REG             TCCR2
#define CS22_REG             TCCR2
#define WGM21_REG            TCCR2
#define COM20_REG            TCCR2
#define COM21_REG            TCCR2
#define WGM20_REG            TCCR2
#define FOC2_REG             TCCR2

/* DDRB */
#define DDB0_REG             DDRB
#define DDB1_REG             DDRB
#define DDB2_REG             DDRB
#define DDB3_REG             DDRB
#define DDB4_REG             DDRB
#define DDB5_REG             DDRB
#define DDB6_REG             DDRB
#define DDB7_REG             DDRB

/* TWDR */
#define TWD0_REG             TWDR
#define TWD1_REG             TWDR
#define TWD2_REG             TWDR
#define TWD3_REG             TWDR
#define TWD4_REG             TWDR
#define TWD5_REG             TWDR
#define TWD6_REG             TWDR
#define TWD7_REG             TWDR

/* TIMSK */
#define TOIE0_REG            TIMSK
#define OCIE0_REG            TIMSK
#define TOIE1_REG            TIMSK
#define OCIE1B_REG           TIMSK
#define OCIE1A_REG           TIMSK
#define TICIE1_REG           TIMSK
#define TOIE2_REG            TIMSK
#define OCIE2_REG            TIMSK

/* EIMSK */
#define INT0_REG             EIMSK
#define INT1_REG             EIMSK
#define INT2_REG             EIMSK
#define INT3_REG             EIMSK
#define INT4_REG             EIMSK
#define INT5_REG             EIMSK
#define INT6_REG             EIMSK
#define INT7_REG             EIMSK

/* TCCR1A */
#define WGM10_REG            TCCR1A
#define WGM11_REG            TCCR1A
#define COM1C0_REG           TCCR1A
#define COM1C1_REG           TCCR1A
#define COM1B0_REG           TCCR1A
#define COM1B1_REG           TCCR1A
#define COM1A0_REG           TCCR1A
#define COM1A1_REG           TCCR1A

/* ACSR */
#define ACIS0_REG            ACSR
#define ACIS1_REG            ACSR
#define ACIC_REG             ACSR
#define ACIE_REG             ACSR
#define ACI_REG              ACSR
#define ACO_REG              ACSR
#define ACBG_REG             ACSR
#define ACD_REG              ACSR

/* PORTF */
#define PORTF0_REG           PORTF
#define PORTF1_REG           PORTF
#define PORTF2_REG           PORTF
#define PORTF3_REG           PORTF
#define PORTF4_REG           PORTF
#define PORTF5_REG           PORTF
#define PORTF6_REG           PORTF
#define PORTF7_REG           PORTF

/* TCCR1C */
#define FOC1C_REG            TCCR1C
#define FOC1B_REG            TCCR1C
#define FOC1A_REG            TCCR1C

/* ICR3H */
#define ICR3H0_REG           ICR3H
#define ICR3H1_REG           ICR3H
#define ICR3H2_REG           ICR3H
#define ICR3H3_REG           ICR3H
#define ICR3H4_REG           ICR3H
#define ICR3H5_REG           ICR3H
#define ICR3H6_REG           ICR3H
#define ICR3H7_REG           ICR3H

/* DDRE */
#define DDE0_REG             DDRE
#define DDE1_REG             DDRE
#define DDE2_REG             DDRE
#define DDE3_REG             DDRE
#define DDE4_REG             DDRE
#define DDE5_REG             DDRE
#define DDE6_REG             DDRE
#define DDE7_REG             DDRE

/* PORTD */
#define PORTD0_REG           PORTD
#define PORTD1_REG           PORTD
#define PORTD2_REG           PORTD
#define PORTD3_REG           PORTD
#define PORTD4_REG           PORTD
#define PORTD5_REG           PORTD
#define PORTD6_REG           PORTD
#define PORTD7_REG           PORTD

/* ICR3L */
#define ICR3L0_REG           ICR3L
#define ICR3L1_REG           ICR3L
#define ICR3L2_REG           ICR3L
#define ICR3L3_REG           ICR3L
#define ICR3L4_REG           ICR3L
#define ICR3L5_REG           ICR3L
#define ICR3L6_REG           ICR3L
#define ICR3L7_REG           ICR3L

/* PORTE */
#define PORTE0_REG           PORTE
#define PORTE1_REG           PORTE
#define PORTE2_REG           PORTE
#define PORTE3_REG           PORTE
#define PORTE4_REG           PORTE
#define PORTE5_REG           PORTE
#define PORTE6_REG           PORTE
#define PORTE7_REG           PORTE

/* SPMCSR */
#define SPMEN_REG            SPMCSR
#define PGERS_REG            SPMCSR
#define PGWRT_REG            SPMCSR
#define BLBSET_REG           SPMCSR
#define RWWSRE_REG           SPMCSR
#define RWWSB_REG            SPMCSR
#define SPMIE_REG            SPMCSR

/* TCNT1H */
#define TCNT1H0_REG          TCNT1H
#define TCNT1H1_REG          TCNT1H
#define TCNT1H2_REG          TCNT1H
#define TCNT1H3_REG          TCNT1H
#define TCNT1H4_REG          TCNT1H
#define TCNT1H5_REG          TCNT1H
#define TCNT1H6_REG          TCNT1H
#define TCNT1H7_REG          TCNT1H

/* ADCL */
#define ADCL0_REG            ADCL
#define ADCL1_REG            ADCL
#define ADCL2_REG            ADCL
#define ADCL3_REG            ADCL
#define ADCL4_REG            ADCL
#define ADCL5_REG            ADCL
#define ADCL6_REG            ADCL
#define ADCL7_REG            ADCL

/* ADCH */
#define ADCH0_REG            ADCH
#define ADCH1_REG            ADCH
#define ADCH2_REG            ADCH
#define ADCH3_REG            ADCH
#define ADCH4_REG            ADCH
#define ADCH5_REG            ADCH
#define ADCH6_REG            ADCH
#define ADCH7_REG            ADCH

/* OCR3BL */
#define OCR3BL0_REG          OCR3BL
#define OCR3BL1_REG          OCR3BL
#define OCR3BL2_REG          OCR3BL
#define OCR3BL3_REG          OCR3BL
#define OCR3BL4_REG          OCR3BL
#define OCR3BL5_REG          OCR3BL
#define OCR3BL6_REG          OCR3BL
#define OCR3BL7_REG          OCR3BL

/* OCR3BH */
#define OCR3BH0_REG          OCR3BH
#define OCR3BH1_REG          OCR3BH
#define OCR3BH2_REG          OCR3BH
#define OCR3BH3_REG          OCR3BH
#define OCR3BH4_REG          OCR3BH
#define OCR3BH5_REG          OCR3BH
#define OCR3BH6_REG          OCR3BH
#define OCR3BH7_REG          OCR3BH

/* ADCSRA */
#define ADPS0_REG            ADCSRA
#define ADPS1_REG            ADCSRA
#define ADPS2_REG            ADCSRA
#define ADIE_REG             ADCSRA
#define ADIF_REG             ADCSRA
#define ADFR_REG             ADCSRA
#define ADSC_REG             ADCSRA
#define ADEN_REG             ADCSRA

/* XMCRB */
#define XMM0_REG             XMCRB
#define XMM1_REG             XMCRB
#define XMM2_REG             XMCRB
#define XMBK_REG             XMCRB

/* XMCRA */
#define SRW11_REG            XMCRA
#define SRW00_REG            XMCRA
#define SRW01_REG            XMCRA
#define SRL0_REG             XMCRA
#define SRL1_REG             XMCRA
#define SRL2_REG             XMCRA

/* PINC */
#define PINC0_REG            PINC
#define PINC1_REG            PINC
#define PINC2_REG            PINC
#define PINC3_REG            PINC
#define PINC4_REG            PINC
#define PINC5_REG            PINC
#define PINC6_REG            PINC
#define PINC7_REG            PINC

/* PINB */
#define PINB0_REG            PINB
#define PINB1_REG            PINB
#define PINB2_REG            PINB
#define PINB3_REG            PINB
#define PINB4_REG            PINB
#define PINB5_REG            PINB
#define PINB6_REG            PINB
#define PINB7_REG            PINB

/* PINA */
#define PINA0_REG            PINA
#define PINA1_REG            PINA
#define PINA2_REG            PINA
#define PINA3_REG            PINA
#define PINA4_REG            PINA
#define PINA5_REG            PINA
#define PINA6_REG            PINA
#define PINA7_REG            PINA

/* PING */
#define PING0_REG            PING
#define PING1_REG            PING
#define PING2_REG            PING
#define PING3_REG            PING
#define PING4_REG            PING

/* PINF */
#define PINF0_REG            PINF
#define PINF1_REG            PINF
#define PINF2_REG            PINF
#define PINF3_REG            PINF
#define PINF4_REG            PINF
#define PINF5_REG            PINF
#define PINF6_REG            PINF
#define PINF7_REG            PINF

/* PINE */
#define PINE0_REG            PINE
#define PINE1_REG            PINE
#define PINE2_REG            PINE
#define PINE3_REG            PINE
#define PINE4_REG            PINE
#define PINE5_REG            PINE
#define PINE6_REG            PINE
#define PINE7_REG            PINE

/* PIND */
#define PIND0_REG            PIND
#define PIND1_REG            PIND
#define PIND2_REG            PIND
#define PIND3_REG            PIND
#define PIND4_REG            PIND
#define PIND5_REG            PIND
#define PIND6_REG            PIND
#define PIND7_REG            PIND

/* OCR1AH */
#define OCR1AH0_REG          OCR1AH
#define OCR1AH1_REG          OCR1AH
#define OCR1AH2_REG          OCR1AH
#define OCR1AH3_REG          OCR1AH
#define OCR1AH4_REG          OCR1AH
#define OCR1AH5_REG          OCR1AH
#define OCR1AH6_REG          OCR1AH
#define OCR1AH7_REG          OCR1AH

/* OCR1AL */
#define OCR1AL0_REG          OCR1AL
#define OCR1AL1_REG          OCR1AL
#define OCR1AL2_REG          OCR1AL
#define OCR1AL3_REG          OCR1AL
#define OCR1AL4_REG          OCR1AL
#define OCR1AL5_REG          OCR1AL
#define OCR1AL6_REG          OCR1AL
#define OCR1AL7_REG          OCR1AL

/* OCR0 */
#define OCR0_0_REG           OCR0
#define OCR0_1_REG           OCR0
#define OCR0_2_REG           OCR0
#define OCR0_3_REG           OCR0
#define OCR0_4_REG           OCR0
#define OCR0_5_REG           OCR0
#define OCR0_6_REG           OCR0
#define OCR0_7_REG           OCR0

/* OCR2 */
#define OCR2_0_REG           OCR2
#define OCR2_1_REG           OCR2
#define OCR2_2_REG           OCR2
#define OCR2_3_REG           OCR2
#define OCR2_4_REG           OCR2
#define OCR2_5_REG           OCR2
#define OCR2_6_REG           OCR2
#define OCR2_7_REG           OCR2
