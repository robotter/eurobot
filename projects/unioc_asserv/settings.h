/*  
 *  Copyright RobOtter (2010)
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

/** \file settings.h
  * \author JD
  *
  * Main configuration file 
  *
  */

#ifndef SETTINGS_H
#define SETTINGS_H

// -- SECURITY --

//#define SETTING_UART_UI_ENABLED
#define SETTING_COMPILE_CALIBRATION
#define SETTING_AVOIDANCE_ENABLED 

// -- match --

#define SETTING_MATCH_DURATION_SECS (89)

// -- PWM --
#define SETTING_PWM_FREQUENCY_KHZ (20)

// -- motors --

#define SETTING_MOTORS_MAXPWM (2000)

#define SETTING_MOTOR_A_DIR (1)
#define SETTING_MOTOR_B_DIR (-1)
#define SETTING_MOTOR_C_DIR (1)

// -- pid (x,y,a) --

#if defined(GALIPEUR)
#define SETTING_PID_X_GAIN_P (3000)
#elif defined(GALIPETTE)
#define SETTING_PID_X_GAIN_P (1400)
#endif
#define SETTING_PID_X_GAIN_I (0)
#define SETTING_PID_X_GAIN_D (0)

#define SETTING_PID_X_MAX_IN  (50000)
#define SETTING_PID_X_MAX_I   (100000)
#define SETTING_PID_X_MAX_OUT (0)

#define SETTING_PID_X_SHIFT (1)

// --
#if defined(GALIPEUR)
#define SETTING_PID_Y_GAIN_P (3000)
#elif defined(GALIPETTE)
#define SETTING_PID_Y_GAIN_P (1400)
#endif

#define SETTING_PID_Y_GAIN_I (0)
#define SETTING_PID_Y_GAIN_D (0)

#define SETTING_PID_Y_MAX_IN  (50000)
#define SETTING_PID_Y_MAX_I   (100000)
#define SETTING_PID_Y_MAX_OUT (0)

#define SETTING_PID_Y_SHIFT (1)

// --

#define SETTING_PID_A_GAIN_P (2500)
#define SETTING_PID_A_GAIN_I (0)
#define SETTING_PID_A_GAIN_D (0)

#define SETTING_PID_A_MAX_IN  (50000)
#define SETTING_PID_A_MAX_I   (1000)
#define SETTING_PID_A_MAX_OUT (0)

#define SETTING_PID_A_SHIFT (1)

// -- quadramp angle --

#define SETTING_QRAMP_A_SPEED (10)
#define SETTING_QRAMP_A_ACC   (10)

// -- trajectory manager --

#define SETTING_TRAJECTORY_A_SPEED (200.0)
#define SETTING_TRAJECTORY_A_ACC   (100.0)

#define SETTING_TRAJECTORY_XYCRUISE_SPEED (20.0)
#define SETTING_TRAJECTORY_XYCRUISE_ACC   (0.1)

#define SETTING_TRAJECTORY_XYSTEERING_SPEED (5.0)
#define SETTING_TRAJECTORY_XYSTEERING_ACC   (0.1)

#define SETTING_TRAJECTORY_XYSTOP_SPEED (1.0)
#define SETTING_TRAJECTORY_XYSTOP_ACC   (0.1)

#define SETTING_TRAJECTORY_STEERING_XYWIN (50.0)

#define SETTING_TRAJECTORY_STOP_XYWIN (20.0)
#define SETTING_TRAJECTORY_STOP_AWIN  (0.03)

// Control system update period, in microseconds
#define CONTROL_SYSTEM_PERIOD_US  15000
/// Interrupt level for control system update
#define CONTROL_SYSTEM_INTLVL  INTLVL_LO
// ADXRS updated period, in microseconds
#define ADXRS_PERIOD_US  10000
#define ADXRS_INTLVL INTLVL_MED

// -- position manager --

#define SETTING_POSITION_INIT_X (0.0)
#define SETTING_POSITION_INIT_Y (0.0)
#define SETTING_POSITION_INIT_A (0.0)

// -- paddock/pwm test --

#define SETTING_PADDOCK_PWMTEST_INC (500)

// -- avoidance --

#define SETTING_AVOIDANCE_GP2ARRAY_SIZE (3)
#define SETTING_AVOIDANCE_GP2ARRAY_ADC  (ADCB)
#define SETTING_AVOIDANCE_GP2ARRAY_30   (ADC_CH_MUXPOS_PIN1_gc)
#define SETTING_AVOIDANCE_GP2ARRAY_150  (ADC_CH_MUXPOS_PIN2_gc)
#define SETTING_AVOIDANCE_GP2ARRAY_270  (ADC_CH_MUXPOS_PIN3_gc)

#if defined(GALIPEUR)
#define SETTING_AVOIDANCE_GP2ARRAY_THRESHOLD (300)
#define SETTING_AVOIDANCE_GP2ARRAY_COUNT     (2)
#elif defined(GALIPETTE)
#define SETTING_AVOIDANCE_GP2ARRAY_THRESHOLD (1200)
#define SETTING_AVOIDANCE_GP2ARRAY_COUNT     (20)
#endif

// -- autoset --

#define SETTING_AUTOSET_SPEED (30000000.0)
#define SETTING_AUTOSET_ZEROCOUNT (100)

#endif/*SETTINGS_H*/
