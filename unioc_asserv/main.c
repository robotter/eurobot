/*  
 *  Copyright RobOtter (2013)
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

#include <avarix.h>
#include <clock/clock.h>
#include <util/delay.h>

#include <stdio.h>
#include <stdlib.h>
#include <uart/uart.h>
#include <math.h>
#include <timer/timer.h>

#include "logging.h"

#include "hrobot_manager.h"
#include "cs.h"
#include "robot_cs.h"
#include "htrajectory.h"
#include "logging.h"
#include "cli.h"
#include "motor_encoders.h"

#include "settings.h"

// error code
#define MAIN_ERROR 0x30


//-----

void safe_key_pressed(void* dummy);

void paddock_manualControl(void);
void paddock_adnsFeedback(void);
void paddock_positionTest(void);
void paddock_testCode(void);
void paddock_pwmTest(void);
void paddock_calibration(void);
void paddock_colors(void);
//-----

// log level
extern uint8_t log_level;

// CSs cpu usage in percent (0-100)
extern uint8_t cs_cpuUsage;

void vcs_update(void)
{
  cs_update(NULL);
}

int main(void)
{
  // Booting

  // Initialize clocks
  clock_init();

  // Initialize UART
  uart_init();
  uart_fopen(CLI_USER_UART);

  // Initialize Timer
  timer_init();
 // timer_set_callback(timerE0, 'A', TIMER_US_TO_TICKS(E0,CONTROL_SYSTEM_PERIOD_US), CONTROL_SYSTEM_INTLVL, vcs_update);

  INTLVL_ENABLE_ALL();
  __asm__("sei");

  // Clear screen
#ifdef SETTING_UART_UI_ENABLED
  printf("%c[2J",0x1B);
  printf("%c[0;0H",0x1B);
#endif

  // Some advertisment :p
  NOTICE(0,"Robotter 2013 - Galipeur - SUPER-UNIOC-NG PROPULSION\n");
  NOTICE(0,"Compiled "__DATE__" at "__TIME__".");

  //--------------------------------------------------------
  // CS
  //--------------------------------------------------------

  NOTICE(0,"Initializing CS");
  cs_initialize();

  //--------------------------------------------------------
  // For ploting purposes
  NOTICE(0,"<PLOTMARK>");

  // remove break
  hrobot_break(0);

  //----------------------------------------------------------------------

#ifdef SETTING_UART_UI_ENABLED
  NOTICE(0,"'x' to reboot / 'c' manual control / 'z' position test / 'p' PWM test / 'l' calibration / 't' test code");

  int c;
  while(1)
  {
    // get input key
    c = cli_getkey_nowait();
    if(c == -1)
      continue;

    // ------------------

    if(c == 0)
      continue;
    if(c == 'x')
      EMERG(MAIN_ERROR,"safe key 'x' pressed");
    if(c == 'c')
      paddock_manualControl();
    if(c == 'z')
      paddock_positionTest();
    if(c == 't')
      paddock_testCode();
    if(c == 'p')
      paddock_pwmTest();
    if(c == 'l')
      paddock_calibration();
    if(c != 0xFF)
      break;
  }
#endif
  for(;;) ;
  return 0;
}

void paddock_testCode(void)
{
  NOTICE(0,"TEST CODE");

  // doing nothing

  for(;;);
}

void paddock_positionTest(void)
{
  NOTICE(0, "Entering position test");
  
  vect_xy_t vxy;
  double a;

  while(1)
  {
    hposition_get_xy(&position, &vxy);
    hposition_get_a(&position, &a);
    NOTICE(0,"POSITION (X,Y,A) : x=%3.3f y=%3.3f a=%3.3f (%3.3f°)",
                vxy.x, vxy.y, a, a*180/M_PI);

  }

}

void paddock_manualControl(void)
{
  uint8_t key;
  double x = SETTING_POSITION_INIT_X;
  double y = SETTING_POSITION_INIT_Y;
  double a = SETTING_POSITION_INIT_A;

  NOTICE(0,"Entering manual control");

  while(1)
  {
    key = cli_getkey();

    if(key=='x')
      EMERG(MAIN_ERROR,"safe key 'x' pressed");

    switch(key)
    {
      case 'z':
        x=SETTING_POSITION_INIT_X;
        y=SETTING_POSITION_INIT_Y;
        a=SETTING_POSITION_INIT_A;
        break;

      case 'j':
        x-=10.0;
        break;

      case 'l':
        x+=10.0;
        break;

      case 'k':
        y-=10.0;
        break;

      case 'i':
        y+=10.0;
        break;

      case 'u':
        a+=0.05*M_PI;
        break;

      case 'o':
        a-=0.05*M_PI;
        break;
    }

    NOTICE(0,"manual control : (%2.2f, %2.2f, %2.2f)",x,y,a);

    robot_cs_set_xy_consigns(&robot_cs, x*RCS_MM_TO_CSUNIT,
                                        y*RCS_MM_TO_CSUNIT);

    robot_cs_set_a_consign(&robot_cs, a*RCS_RAD_TO_CSUNIT);


  } 
}

void paddock_pwmTest(void)
{
  uint8_t key;
  int16_t pwm1 = 0;
  int16_t pwm2 = 0;
  int16_t pwm3 = 0;
  uint16_t frq_khz = 20;
  
  NOTICE(0,"Entering PWM test mode");
  NOTICE(0,"PWM1 +/- : i/k | PWM2 +/- : o/l | PWM3 +/- : p/m | FRQ +/- : u/j | zero all : z");

  while(1)
  {
    key = cli_getkey();

    if(key=='x')
      EMERG(MAIN_ERROR,"safe key 'x' pressed");
    
    switch(key)
    {
      case 'i': pwm1 += SETTING_PADDOCK_PWMTEST_INC; break;
      case 'k': pwm1 -= SETTING_PADDOCK_PWMTEST_INC; break;
      case 'o': pwm2 += SETTING_PADDOCK_PWMTEST_INC; break;
      case 'l': pwm2 -= SETTING_PADDOCK_PWMTEST_INC; break;
      case 'p': pwm3 += SETTING_PADDOCK_PWMTEST_INC; break;
      case 'm': pwm3 -= SETTING_PADDOCK_PWMTEST_INC; break;
      case 'u': frq_khz += 1; break;
      case 'j': frq_khz -= 1; break;
      case 'z':
        pwm1 = 0;
        pwm2 = 0;
        pwm3 = 0;
        break;
    }

    pwm_motor_set(system.pwms+0, pwm1);
    pwm_motor_set(system.pwms+1, pwm2);
    pwm_motor_set(system.pwms+2, pwm3);

    uint8_t it;
    for(it=0;it<3;it++)
      pwm_motor_set_frequency(system.pwms+it, frq_khz*1000);

    NOTICE(0,"PWM1=%5i PWM2=%5i PWM3=%5i FRQ=%u_kHz",pwm1,pwm2,pwm3,frq_khz);

  }

  for(;;);
}

void paddock_calibration(void)
{
  NOTICE(0,"Entering calibration");

#ifdef SETTING_COMPILE_CALIBRATION
  int16_t *vectors;

  NOTICE(0,"<CALIBRATION>");
  while(1)
  {
    vectors = motor_encoders_get_value();
    printf("%i,%i,%i\n",
              vectors[0],
              vectors[1],
              vectors[2]);
    _delay_ms(10);
    motor_encoders_update();
  }

#endif

  for(;;);
}

void safe_key_pressed(void* dummy)
{
  if(cli_getkey_nowait() == 'x') 
    EMERG(MAIN_ERROR,"safe key 'x' pressed");
}
