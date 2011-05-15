/*  
 *  Copyright RobOtter (2009) 
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

#include <aversive.h>
#include <aversive/wait.h>
#include <aversive/error.h>

#include <stdio.h>
#include <stdlib.h>
#include <uart.h>
#include <math.h>
#include <scheduler.h>
#include <adc.h>
#include <time.h>
#include <i2cs.h>
#include <timer.h>

#include <adns9500.h>

#include "fpga.h"
#include "cs.h"
#include "robot_cs.h"
#include "htrajectory.h"
#include "logging.h"
#include "cli.h"
#include "cord.h"
#include "pwm.h"
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
//-----

// log level
extern uint8_t log_level;

// trajectory management 
// XXX TBMoved to a strat dedicated source file
extern volatile htrajectory_t trajectory;

// manual control
// XXX TBMoved to a manual control dedicated source file
extern robot_cs_t robot_cs;

// XXX 
extern hrobot_position_t position;

// XXX
extern hrobot_system_t system;

// CSs cpu usage in percent (0-100)
extern uint8_t cs_cpuUsage;

// scheduler events
uint8_t event_position;
uint8_t event_cs;

// time in seconds at startup
seconds time_startup;
uint8_t time_startup_ok = 0;

int main(void)
{
  //--------------------------------------------------------------------------
  // Booting

  // Turn interruptions ON
  sei();

  // Initialize UART
  uart_init();
  fdevopen(uart1_dev_send, uart1_dev_recv);

  //--------------------------------------------------------
  // Error configuration
  error_register_emerg(log_event);
  error_register_error(log_event);
  error_register_warning(log_event);
  error_register_notice(log_event);
  error_register_debug(log_event);

  //log_level = ERROR_SEVERITY_NOTICE;
  log_level = ERROR_SEVERITY_DEBUG;

  // Clear screen
  printf("%c[2J",0x1B);
  printf("%c[0;0H",0x1B);

  // Some advertisment :p
  NOTICE(0,"Robotter 2010 - Galipeur - UNIOC-NG PROPULSION");
  NOTICE(0,"Compiled "__DATE__" at "__TIME__".");

  //--------------------------------------------------------
  // Initialize scheduler
  scheduler_init();

  //--------------------------------------------------------
  // Initialize time
  time_init(160);

  //--------------------------------------------------------
  // Initialize FPGA
  fpga_init();

  // turn off led
  _SFR_MEM8(0x1800) = 1;
  
  //--------------------------------------------------------
  // ADNS9500
  //--------------------------------------------------------

  #ifdef SETTING_OVERRIDE_ADNSBOOT
  WARNING(MAIN_ERROR, "ADNS9500 BOOT OVERRIDE");
  #else
  NOTICE(0,"Initializing ADNS9500s");
  adns9500_init();

  NOTICE(0,"Checking ADNS9500s firmware");
  adns9500_check_firmware();

  NOTICE(0,"Booting ADNS9500s");
  adns9500_boot();

  NOTICE(0,"Checking ADNS9500s");
  adns9500_checks();

  NOTICE(0,"ADNS9500s are GO");
  #endif//SETTING_OVERRIDE_ADNSBOOT

  //--------------------------------------------------------
  // CS
  //--------------------------------------------------------

  NOTICE(0,"Initializing CS");
  cs_initialize();

  //--------------------------------------------------------
  // MISC
  //--------------------------------------------------------

  NOTICE(0,"Initializing TIMER3");
  timer_init();

  NOTICE(0,"Initializing ADCs");
  adc_init();
 
  NOTICE(0,"Initializing I2C slave");
  i2cs_init(SETTING_I2C_ADDRESS);
 
  NOTICE(0,"Initializing startup cord");
  cord_init();

  NOTICE(0,"Initializing communications");
  //ppp_init();

  //--------------------------------------------------------
  // For ploting purposes
  NOTICE(0,"<PLOTMARK>");

  // Set ADNS9500 system to automatic
  adns9500_set_mode(ADNS9500_BHVR_MODE_AUTOMATIC);

  // Unleash control systems


  event_cs = 
    scheduler_add_periodical_event_priority(&cs_update, NULL,
                                              SETTING_SCHED_CS_PERIOD,
                                              SETTING_SCHED_CS_PRIORITY);

  // Safe key event
  scheduler_add_periodical_event_priority(&safe_key_pressed, NULL,
                                              SETTING_SCHED_SAFEKEY_PERIOD,
                                              SETTING_SCHED_SAFEKEY_PRIORITY);

  // remove break
  hrobot_break(NULL,0);

  //----------------------------------------------------------------------

  NOTICE(0,"'x' to reboot / 'c' manual control / 'a' ADNS test / 'z' position test / 'p' PWM test / 'l' calibration / 't' test code / (key/cord) to go ");
 
  int c;
  uint8_t cord_status, lock;
  lock = 0;
  while(1)
  {
    // get input key
    c = cli_getkey_nowait();
    if(c == -1)
      continue;

    // get cord status
    cord_status = cord_isPlugged();

    if(cord_status == 1)
      lock = 1;

    // if cord not plugged
    if( (cord_status == 0) && lock )
      break;

    // ------------------

    if(c == 0)
      continue;
    
    if(c == 'x')
      EMERG(MAIN_ERROR,"safe key 'x' pressed");
  
    if(c == 'c')
      paddock_manualControl();

    
    if(c == 'a')
      paddock_adnsFeedback();
    
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

  //----------------------------------------------------------------------
  //----------------------------------------------------------------------
  NOTICE(0,"Starting *homologuation*");
  
  // get time at startup
  time_startup_ok = 1;
  time_startup = time_get_s();

  vect_xy_t path[2];

  double color = -1.0; // 1.0 = jaune | -1.0 = bleu

  path[0] = (vect_xy_t){color*(1500 + 18 - 1125), 1050 + 12 - 453.0};
  if(color == 1.0) // jaune
    path[1] = (vect_xy_t){color*(1500 + 18 + 1075), 1050 + 12 + 797.0};
  else // bleu
    path[1] = (vect_xy_t){color*(1500 + 18 + 1185), 1050 + 12 + 797.0};

  htrajectory_run(&trajectory, path, 2);
  while(!htrajectory_doneXY(&trajectory)) nop();

  if(color == 1.0) // jaune
    htrajectory_gotoXY_R(&trajectory, color*120, 70);
  else
    htrajectory_gotoXY_R(&trajectory, color*150, 70);

  while(!htrajectory_doneXY(&trajectory)) nop();

  htrajectory_gotoA_R(&trajectory, color*M_PI/3);

  NOTICE(0,"Done");

  while(1);

  return 0;
}

void paddock_testCode(void)
{
  vect_xy_t path[3];
  adns9500_encoders_t adns;
  int16_t squal;

  while(1)
  {
    adns9500_encoders_get_value(&adns);
    squal = adns.squals[1];

    if(squal > 80)
    {
      NOTICE(0,"SQUAL !");
    }

    wait_ms(10);
  }
  
  while(1) nop();
}

void paddock_adnsFeedback(void)
{
  adns9500_encoders_t adns;

  NOTICE(0, "Entering ANDS feedback mode");

  // breaking motors
  hrobot_break(&system,1);
  scheduler_del_event(event_cs);

  while(1)
  {
    adns9500_encoders_get_value(&adns);
    
    NOTICE(0,"ADNS 0x%2.2x | %6ld %6ld %6ld %6ld %6ld %6ld | %3.3f %3.3f %3.3f | %d %d %d",
              adns.fault,
              adns.vectors[0],
              adns.vectors[1],
              adns.vectors[2],
              adns.vectors[3],
              adns.vectors[4],
              adns.vectors[5],
              sqrt(adns.vectors[0]*adns.vectors[0] + adns.vectors[1]*adns.vectors[1]),
              sqrt(adns.vectors[2]*adns.vectors[2] + adns.vectors[3]*adns.vectors[3]),
              sqrt(adns.vectors[4]*adns.vectors[4] + adns.vectors[5]*adns.vectors[5]),
              adns.squals[0],
              adns.squals[1],
              adns.squals[2]);
  }
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
  int32_t pwm1 = 0;
  int32_t pwm2 = 0;
  int32_t pwm3 = 0;
  uint16_t period = 5000;
  
  NOTICE(0,"Entering PWM test mode");

  // kill CSs
  scheduler_del_event(event_cs);

  NOTICE(0,"PWM1 +/- : i/k | PWM2 +/- : o/l | PWM3 +/- : p/m | zero all : z");

  while(1)
  {
    key = cli_getkey();

    if(key=='x')
      EMERG(MAIN_ERROR,"safe key 'x' pressed");
    
    switch(key)
    {
      case 'i': 
        pwm1 += SETTING_PADDOCK_PWMTEST_INC;
        break;
        
      case 'k':
        pwm1 -= SETTING_PADDOCK_PWMTEST_INC;
        break;

      case 'o':
        pwm2 += SETTING_PADDOCK_PWMTEST_INC;
        break;

      case 'l':
        pwm2 -= SETTING_PADDOCK_PWMTEST_INC;
        break;

      case 'p':
        pwm3 += SETTING_PADDOCK_PWMTEST_INC;
        break;

      case 'm':
        pwm3 -= SETTING_PADDOCK_PWMTEST_INC;
        break;

      case 'u':
        period += 10;
        break;

      case 'j':
        period -= 10;
        break;

      case 'z': 
        pwm1 = 0;
        pwm2 = 0;
        pwm3 = 0;
        break;
    }


    _SFR_MEM16(PWM_REGISTER_PERIOD_L1) = period;
    _SFR_MEM16(PWM_REGISTER_PERIOD_L2) = period;
    _SFR_MEM16(PWM_REGISTER_PERIOD_L3) = period;   
    pwm_set_A(pwm1);
    pwm_set_B(pwm2);
    pwm_set_C(pwm3);

    NOTICE(0,"PWM1=%5ld PWM2=%5ld PWM3=%5ld PERIOD=%d",pwm1,pwm2,pwm3,period);

  }

  while(1) nop();
}

void paddock_calibration(void)
{
  uint8_t key;
  
  NOTICE(0,"Entering calibration");

  // kill CSs
  scheduler_del_event(event_cs);

#ifdef SETTING_COMPILE_CALIBRATION

  adns9500_encoders_t adns;
  motor_encoders_t me;

  NOTICE(0,"<CALIBRATION>");
  while(1)
  {
    adns9500_encoders_get_value(&adns);
    motor_encoders_get_value(&me);

    printf("%d,%d,%ld,%ld,%ld\n",
              adns.squals[0],
              adns.squals[1],
              me.vectors[0],
              me.vectors[1],
              me.vectors[2]);

    wait_ms(10);
  }

#endif

  while(1) nop();
}

void safe_key_pressed(void* dummy)
{
  if(cli_getkey_nowait() == 'x') 
    EMERG(MAIN_ERROR,"safe key 'x' pressed");
}
