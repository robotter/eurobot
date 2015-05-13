#include <math.h>
#include <stdlib.h>
#include <clock/clock.h>
#include <util/delay.h>
#include <avarix/portpin.h>
#include <rome/rome.h>
#include <timer/uptime.h>
#include <idle/idle.h>
#include "strat.h"
#include "config.h"

#define SPOT_ELEVATOR_LENGTH 83

extern rome_intf_t rome_asserv;
extern rome_intf_t rome_meca;
extern rome_intf_t rome_paddock;

robot_state_t robot_state;
// we don't put 'kx' on robot_state to allow it to be static
// this avoids optimization errors
static int16_t robot_kx;


typedef enum {
  ROBOT_SIDE_LEFT = 0,
  ROBOT_SIDE_RIGHT,
  ROBOT_SIDE_BACK,
} robot_side_t;

#define ROBOT_SIDE_MAIN (robot_state.team == TEAM_GREEN ? ROBOT_SIDE_RIGHT : ROBOT_SIDE_LEFT)
#define ROBOT_SIDE_AUX  (robot_state.team == TEAM_GREEN ? ROBOT_SIDE_LEFT : ROBOT_SIDE_RIGHT)
#define AUTOSET_MAIN (robot_state.team == TEAM_GREEN ? AUTOSET_RIGHT : AUTOSET_LEFT)
#define AUTOSET_AUX  (robot_state.team == TEAM_GREEN ? AUTOSET_LEFT : AUTOSET_RIGHT)
#define KX(x) (robot_kx*(x))
#define KA(a) (robot_kx*(a))

typedef enum {
  MECA_LEFT = 0,
  MECA_RIGHT = 1
}spot_elevator_t;

typedef enum {
  SPOT_ELEV_S_BUSY = 0,
  SPOT_ELEV_S_GROUND_CLEAR,
  SPOT_ELEV_S_READY,
}spot_elevator_state_t;

// Return true if an opponent is detected
bool opponent_detected(void)
{
  for(uint8_t i=0; i<R3D2_OBJECTS_MAX; i++) {
    r3d2_object_t *object = &robot_state.r3d2.objects[i];
    if(object->detected && object->r < R3D2_AVOID_DISTANCE) {
      return true;
    }
  }
  return false;
}

// Return true if an opponent is detected within an arc
// We assume 0 <= a1 < a2 < 2Pi
bool opponent_detected_arc(float a1, float a2)
{
  for(uint8_t i=0; i<R3D2_OBJECTS_MAX; i++) {
    r3d2_object_t *object = &robot_state.r3d2.objects[i];
    // note: object->a is already in [0;2Pi[
    if(object->detected && object->r < R3D2_AVOID_DISTANCE &&
       object->a >= a1 && object->a >= a2) {
      return true;
    }
  }
  return false;
}

void calibrate_gyro(bool b)
{
  ROME_SENDWAIT_ASSERV_CALIBRATE(&rome_asserv, b);
}

void activate_asserv(bool b)
{
  ROME_SENDWAIT_ASSERV_ACTIVATE(&rome_asserv, b);
}

void ext_arm_set(int16_t pos)
{
  ROME_SENDWAIT_MECA_SET_ARM(&rome_meca, pos);
}

/// Set arm position
void ext_arm_raise(void) { ext_arm_set(430); }
void ext_arm_lower(void) { ext_arm_set(100); }
void ext_arm_clap(void)  { ext_arm_set(300); }
void ext_arm_galipette(void)  { ext_arm_set(180); }

void _wait_meca_ready(void){
  for (;;){
    if((robot_state.left_elev.state == SPOT_ELEV_S_READY)&&
       (robot_state.right_elev.state == SPOT_ELEV_S_READY))
      return;
    idle();
  }
}

void _wait_meca_ground_clear(void){
  for (;;){
    if((robot_state.left_elev.state != SPOT_ELEV_S_BUSY)&&
       (robot_state.right_elev.state != SPOT_ELEV_S_BUSY))
      return;
    idle();
  }
}


static void _meca_order_blocking_left_right(uint8_t cmd_left, uint8_t cmd_right){
  _wait_meca_ready();
  //send orders
  if (cmd_left){
    ROME_SENDWAIT_MECA_CMD(&rome_meca, cmd_left, MECA_LEFT);
    robot_state.left_elev.state = SPOT_ELEV_S_BUSY;
  }
  if (cmd_right){
    ROME_SENDWAIT_MECA_CMD(&rome_meca, cmd_right, MECA_RIGHT);
    robot_state.right_elev.state = SPOT_ELEV_S_BUSY;
  }
  //wait for meca to compute orders...
  idle();
  _wait_meca_ground_clear();
}

static void _meca_order_blocking_main_aux(uint8_t cmd_main, uint8_t cmd_aux){
  if(robot_state.team == TEAM_GREEN)
    _meca_order_blocking_left_right(cmd_aux,cmd_main);
  else if(robot_state.team == TEAM_YELLOW)
    _meca_order_blocking_left_right(cmd_main,cmd_aux);
}

#define _meca_order_blocking_ma(m,a) _meca_order_blocking_main_aux(ROME_ENUM_MECA_COMMAND_##m,ROME_ENUM_MECA_COMMAND_##a)
#define _meca_order_blocking_both(cmd) _meca_order_blocking_left_right(ROME_ENUM_MECA_COMMAND_##cmd,ROME_ENUM_MECA_COMMAND_##cmd)

/// Go to given position, avoid opponents
static void goto_xya(int16_t x, int16_t y, float a)
{
  for(;;) {
    ROME_SENDWAIT_ASSERV_GOTO_XY(&rome_asserv, x, y, 1000*a);
    robot_state.asserv.xy = 0;
    robot_state.asserv.a = 0;
    for(;;) {
      if(robot_state.asserv.xy && robot_state.asserv.a) {
        return;
      }
      if(opponent_detected()) {
        ROME_LOG(&rome_paddock,INFO,"goto_xya : opponent detected");
        ROME_SENDWAIT_ASSERV_GOTO_XY_REL(&rome_asserv, 0, 0, 0);
        ROME_SENDWAIT_ASSERV_ACTIVATE(&rome_asserv, 0);
        //TODO use opponent_detected_arc()
        while(opponent_detected()) {
          idle();
        }
        ROME_SENDWAIT_ASSERV_ACTIVATE(&rome_asserv, 1);
        break; // to resend goto order
      }
      idle();
    }
  }
}

/// Execute trajectory, avoid opponents
#define goto_traj(xy,a) goto_traj_n((xy), sizeof(xy)/sizeof(int16_t), (a))
static void goto_traj_n(int16_t* xy, uint8_t n, float a)
{
  uint8_t path_i = 0;
  for(;;) {
    ROME_SENDWAIT_ASSERV_RUN_TRAJ(&rome_asserv,1000*a,xy+path_i*2,n);
    robot_state.asserv.xy = 0;
    robot_state.asserv.a = 0;
    robot_state.asserv.path_i = path_i;
    robot_state.asserv.path_n = n/2;
    for(;;) {
      if(robot_state.asserv.xy && robot_state.asserv.a) {
        return;
      }
      if(opponent_detected()) {
        ROME_LOG(&rome_paddock,INFO,"goto_traj : opponent detected");
        path_i = robot_state.asserv.path_i;
        ROME_SENDWAIT_ASSERV_GOTO_XY_REL(&rome_asserv, 0, 0, 0);
        ROME_SENDWAIT_ASSERV_ACTIVATE(&rome_asserv, 0);
        //TODO use opponent_detected_arc()
        while(opponent_detected()) {
          idle();
        }
        ROME_SENDWAIT_ASSERV_ACTIVATE(&rome_asserv, 1);
        break; // to resend goto order
      }
      idle();
    }
  }
}

/// Go to given position, with robot panning and scanning, avoid opponents
void goto_xya_panning(int16_t x, int16_t y, float pan_angle)
{
  for(;;) {
    ROME_SENDWAIT_ASSERV_GOTO_XYA_PANNING(&rome_asserv, x, y, 1000*pan_angle);
    robot_state.asserv.xy = 0;
    robot_state.asserv.a = 0;
    for(;;) {
      if(robot_state.asserv.xy && robot_state.asserv.a) {
        return;
      }
      if(opponent_detected()) {
        ROME_LOG(&rome_paddock,INFO,"goto_xya_panning : opponent detected");
        ROME_SENDWAIT_ASSERV_GOTO_XY_REL(&rome_asserv, 0, 0, 0);
        ROME_SENDWAIT_ASSERV_ACTIVATE(&rome_asserv, 0);
        //TODO use opponent_detected_arc()
        while(opponent_detected()) {
          idle();
        }
        ROME_SENDWAIT_ASSERV_ACTIVATE(&rome_asserv, 1);
        break; // to resend goto order
      }
      idle();
    }
  }
}

/// Go to given relative position, avoid opponents
static void goto_xya_rel(int16_t x, int16_t y, float a)
{
  for(;;) {
    ROME_SENDWAIT_ASSERV_GOTO_XY_REL(&rome_asserv, x, y, 1000*a);
    robot_state.asserv.xy = 0;
    robot_state.asserv.a = 0;
    for(;;) {
      if(robot_state.asserv.xy && robot_state.asserv.a) {
        return;
      }
      if(opponent_detected()) {
        ROME_LOG(&rome_paddock,INFO,"goto_xya_rel : opponent detected");
        ROME_SENDWAIT_ASSERV_GOTO_XY_REL(&rome_asserv, 0, 0, 0);
        ROME_SENDWAIT_ASSERV_ACTIVATE(&rome_asserv, 0);
        //TODO use opponent_detected_arc()
        while(opponent_detected()) {
          idle();
        }
        ROME_SENDWAIT_ASSERV_ACTIVATE(&rome_asserv, 1);
        break; // to resend goto order
      }
      idle();
    }
  }
}

void goto_xy_rel_align_course(int16_t x, int16_t y, bool claws_first){

  float angle;
  //get angle of movement
  angle = atan2(y,x)-M_PI/2;
  if(claws_first)
    angle += M_PI;

  //compute relative angle consign
  angle -= robot_state.current_pos.a;

  goto_xya_rel(x,y,angle);
}

#define CLAW_X 70
#define CLAW_Y -200
#define CLAW_APPROACH 50
#define CLAW_PUSH_SPOT -100

void go_pick_spot(int16_t x, int16_t y){
  ROME_LOGF(&rome_paddock,DEBUG,"spot :%d,%d",x,y);

  _meca_order_blocking_ma(PREPARE_PICK_SPOT,NONE);
  int16_t rx = robot_state.current_pos.x ;
  int16_t ry = robot_state.current_pos.y ;
  int16_t rs_x = x - rx;
  int16_t rs_y = y - ry;
 
  double beta = atan2(rs_y,rs_x);

  double alpha = beta - 3*M_PI/2;

  // spot position in robot frame
  int16_t dx = KX(CLAW_X);
  int16_t dy = (CLAW_Y - CLAW_APPROACH);
  ROME_LOGF(&rome_paddock,DEBUG,"dxdy :%d,%d",dx,dy);
  // spot position in table frame
  int16_t tdx = dx*cos(alpha) - dy*sin(alpha);
  int16_t tdy = dx*sin(alpha) + dy*cos(alpha);
  goto_xya(x-tdx,y-tdy,beta+M_PI/2);

  // spot position in robot frame
  dx = KX(CLAW_X);
  dy = CLAW_Y - CLAW_PUSH_SPOT;
  ROME_LOGF(&rome_paddock,DEBUG,"dxdy :%d,%d",dx,dy);
  // spot position in table frame
  tdx = dx*cos(alpha) - dy*sin(alpha);
  tdy = dx*sin(alpha) + dy*cos(alpha);
  _wait_meca_ready();
  goto_xya(x-tdx,y-tdy,beta+M_PI/2);
  _meca_order_blocking_ma(PICK_SPOT,NONE);
}

/// Do an autoset
void autoset(robot_side_t robot_side, autoset_side_t table_side, float x, float y)
{
  ROME_SENDWAIT_ASSERV_AUTOSET(&rome_asserv, robot_side, table_side, x, y);
  robot_state.asserv.autoset = 0;
  while(!robot_state.asserv.autoset) {
    //TODO avoid opponent
    idle();
  }
}

/// delay for some ms
void strat_delay_ms(uint32_t ms) {
  uint32_t tend = uptime_us() + 1000*ms;
  for(;;) {
    if(uptime_us() >= tend) {
      return;
    }
    idle();
  }
}

bool starting_cord_plugged(void)
{
  for(;;) {
    bool b = !portpin_in(&STARTING_CORD_PP);
    bool ok = true;
    for(uint16_t i=0; i<1000; i++) {
      if(b != !portpin_in(&STARTING_CORD_PP)) {
        ok = false;
        break;
      }
    }
    if(ok) {
      return b;
    }
  }
}

team_t strat_select_team(void)
{
  // wait for starting cord to be unplugged
  while(starting_cord_plugged()) {
    if((uptime_us() / 500000) % 2 == 0) {
      portpin_outset(&LED_B_PP);
    } else {
      portpin_outclr(&LED_B_PP);
    }
    idle();
  }

  // wait for color to be selected
  // color is selected when starting cord is plugged
  team_t team = TEAM_NONE;
  portpin_outclr(&LED_B_PP);
  for(;;) {
    if(portpin_in(&COLOR_SELECTOR_PP)) {
      portpin_outclr(&LED_R_PP);
      portpin_outset(&LED_G_PP);
      portpin_outclr(&LED_B_PP);
      team = TEAM_GREEN;
    } else {
      portpin_outset(&LED_R_PP);
      portpin_outset(&LED_G_PP);
      portpin_outclr(&LED_B_PP);
      team = TEAM_YELLOW;
    }
    if(starting_cord_plugged()) {
      portpin_outclr(&LED_B_PP);
      break;
    }
    idle();
  }

  // wait 2s before next step
  uint32_t tend = uptime_us() + 2e6;
  while(uptime_us() < tend) {
    idle();
  }

  return team;
}

void strat_wait_start(void)
{

  while(starting_cord_plugged()) {
    if((uptime_us() / 500000) % 2 == 0) {
      if(robot_state.team == TEAM_YELLOW) {
        portpin_outset(&LED_R_PP);
        portpin_outset(&LED_G_PP);
      } else {
        portpin_outset(&LED_G_PP);
      }
      portpin_outclr(&LED_B_PP);
    } else {
      portpin_outclr(&LED_R_PP);
      portpin_outclr(&LED_G_PP);
      portpin_outset(&LED_B_PP);
    }
    idle();
  }

  portpin_outclr(&LED_R_PP);
  portpin_outclr(&LED_G_PP);
  portpin_outclr(&LED_B_PP);
  ROME_SENDWAIT_START_TIMER(&rome_asserv);
#if (defined GALIPEUR)
  ROME_SENDWAIT_START_TIMER(&rome_meca);
#endif
}


void strat_init_galipeur(void)
{
  ROME_LOG(&rome_paddock,INFO,"Strat init");
  // disable asserv
  ROME_SENDWAIT_ASSERV_ACTIVATE(&rome_asserv, 0);
  ROME_SENDWAIT_ASSERV_SET_HTRAJ_XY_STEERING(&rome_asserv, 2.5, 0.1);
  ROME_SENDWAIT_ASSERV_SET_HTRAJ_XY_CRUISE(&rome_asserv, 20, 0.1);

  // initialize meca
  ROME_SENDWAIT_MECA_SET_POWER(&rome_meca, 1);

  ext_arm_lower(); 

  ROME_SENDWAIT_MECA_CARPET_UNLOCK(&rome_meca, MECA_RIGHT);
  ROME_SENDWAIT_MECA_CARPET_UNLOCK(&rome_meca, MECA_LEFT);
  _meca_order_blocking_both(PREPARE_BULB);
  for(;;) {
    update_rome_interfaces();
    if(!robot_state.gyro_calibration)
      break;
  }
  _meca_order_blocking_both(PICK_BULB);
  ROME_SENDWAIT_MECA_CARPET_LOCK(&rome_meca, MECA_RIGHT);
  ROME_SENDWAIT_MECA_CARPET_LOCK(&rome_meca, MECA_LEFT);

}

void strat_prepare_galipeur(void)
{
  //initalise kx factor
  robot_kx = robot_state.team == TEAM_GREEN ? 1 : -1;

  ROME_LOG(&rome_paddock,DEBUG,"Strat prepare");
  // initialize asserv
  ROME_SENDWAIT_ASSERV_ACTIVATE(&rome_asserv, 1);
  ROME_SENDWAIT_ASSERV_SET_XYA(&rome_asserv, 0, 0, 0);
  ROME_SENDWAIT_ASSERV_GOTO_XY(&rome_asserv, 0, 0, 0);

#if 1
  // autoset robot
  autoset(ROBOT_SIDE_MAIN,AUTOSET_MAIN, KX(1500-100), 0);
  goto_xya_rel(KX(-500), 0, KA(0));
  goto_xya_rel(KX(0), -400, KA(-M_PI/4));
  autoset(ROBOT_SIDE_MAIN,AUTOSET_DOWN, 0, 100);
  // move out (in relative coordinates)
  goto_xya_rel(KX(0), 200, KA(0));

#else
#endif
 // approach front of start area
  goto_xya(KX(1500-600), 1000, KA(0));

  // stack in start area
  goto_xya(KX(1500-400),1050,KA(0));

  //prepare meca
  _meca_order_blocking_both(RESET_ELEVATOR);
  _meca_order_blocking_both(PICK_BULB);
}

void strat_homologation_galipeur(void)
{
  int16_t traj[] = {KX(1500-600), 1030};
  goto_traj(traj,0);
  go_pick_spot(KX(1500-870),645);
  go_pick_spot(KX(1500-1300),600);
  goto_xya(KX(1500-600), 1030, KA(M_PI/2));
  _meca_order_blocking_ma(DISCHARGE_SPOT_STACK,NONE);
  goto_xya_rel(KX(200),0,KA(0));
  _meca_order_blocking_ma(RELEASE_SPOT_STACK,NONE);
  goto_xya_rel(KX(-500),0,KA(0));
}


void galipeur_se_spots(void) {
  ROME_LOG(&rome_paddock,DEBUG,"SE spots");
    // autoset against right side
  autoset(ROBOT_SIDE_MAIN,AUTOSET_MAIN, KX(1500-100), 0);
  _meca_order_blocking_ma(PREPARE_PICK_SPOT,PREPARE_CUP);
  goto_xya_rel(KX(-100),0,KA(0));

  // -- SE SPOTS --
  // pick one spot
  _wait_meca_ready();
  goto_xya(KX(1500-160), 450, KA(0));
  _meca_order_blocking_ma(PICK_SPOT,PICK_CUP);
  _meca_order_blocking_ma(PREPARE_PICK_SPOT,NONE);
  _wait_meca_ready();
  // pick one spot
  _wait_meca_ready();
  goto_xya(KX(1500-160), 300, KA(0));
  _meca_order_blocking_ma(PICK_SPOT,NONE);
}

void galipeur_do_claps(void) {
  ROME_LOG(&rome_paddock,DEBUG,"Claps");
  // -- SE CLAPS -- 
  goto_xya(KX(1500-160), 300, KA(0));
  // translate along SE bordea
  ext_arm_clap();
  goto_xya(KX(1500-300), 300, KA(0));
  ext_arm_raise();
  goto_xya(KX(1500-700), 300, KA(0));
  ext_arm_clap();
  goto_xya(KX(1500-950), 300, KA(0));
  goto_xya_rel(KX(+100), 100,KA(0));
  ext_arm_lower();
}

void galipeur_pick_south_spot(void) {
  ROME_LOG(&rome_paddock,DEBUG,"South spot");
  _meca_order_blocking_ma(PREPARE_PICK_SPOT,NONE);
  goto_xya(KX(1500-820), 230, KA(-2*M_PI/3));
  _wait_meca_ready();
  goto_xya_rel(KX(-150), 0, KA(0));
  _meca_order_blocking_ma(PICK_SPOT,NONE);
}

void galipeur_discharge_pile_red(void) {
  ROME_LOG(&rome_paddock,NOTICE,"Discharge pile in red area");
  // -- DISCHARGE ON THE RED AREA --
  goto_xya(KX(1500-1000), 300, KA(-M_PI/4));
  ROME_LOG(&rome_paddock,DEBUG,"Discharge");
  _meca_order_blocking_ma(DISCHARGE_SPOT_STACK,NONE);
  goto_xy_rel_align_course(KX(-60), -60, true);
  ROME_LOG(&rome_paddock,DEBUG,"Release");
  _meca_order_blocking_ma(RELEASE_SPOT_STACK,UNLOAD_CUP);
  goto_xy_rel_align_course(KX(60*1.2), 60*1.2, false);
  _meca_order_blocking_both(PREPARE_PICK_SPOT);
  goto_xya(KX(1500-880), 250, KA(-M_PI/4));
  autoset(ROBOT_SIDE_MAIN,AUTOSET_DOWN, 0, 100);
  goto_xya_rel(KX(0),100,KA(0));
}

void galipeur_take_galipette_and_bulb(void) {
  ROME_LOG(&rome_paddock,DEBUG,"Galipette and bulb");
  // -- BACK TO START AREA --
  goto_xya(KX(1500-700), 1000, KA(M_PI/2));
  // -- TAKE GALIPETTE --
  _meca_order_blocking_ma(PREPARE_BULB,NONE);
  ext_arm_galipette();
  goto_xya(KX(1500-420), 1010, KA(M_PI/2));
  goto_xya(KX(1500-420), 970, KA(M_PI/2+M_PI/4));
  ext_arm_raise();
  _delay_ms(500);

  // -- TAKE BULB --
  goto_xya(KX(1500-350), 1000, KA(M_PI/2));
  autoset(ROBOT_SIDE_BACK,AUTOSET_MAIN, KX(1500-227), 0);
  _meca_order_blocking_ma(PICK_BULB,NONE);
  goto_xya(KX(1500-700), 1030, KA(M_PI/2));
}

void galipeur_go_to_stairs(void) {
  ROME_LOG(&rome_paddock,DEBUG,"Stairs");
  // -- GO TO STAIRS ! --
  goto_xya(KX(1500-800), 1600, KA(M_PI));
  goto_xya(KX(1500-830), 1700, KA(M_PI));
  _meca_order_blocking_ma(PICK_SPOT,NONE);
  _meca_order_blocking_ma(PREPARE_PICK_SPOT,NONE);
  _wait_meca_ready();
  goto_xya(KX(1500-830), 1800, KA(M_PI));
  _meca_order_blocking_ma(PICK_SPOT,NONE);
  
  goto_xya_rel(KX(0),-160,KA(0));
  autoset(ROBOT_SIDE_MAIN,AUTOSET_AUX, KX(1500-967-100), 0);
  goto_xya_rel(KX(100), 0,KA(0));
  goto_xya(KX(1500-950), 1600, KA(M_PI - M_PI/6));
  ROME_SENDWAIT_MECA_CARPET_UNLOCK(&rome_meca, MECA_RIGHT);
  ROME_SENDWAIT_MECA_CARPET_UNLOCK(&rome_meca, MECA_LEFT);
}

void galipeur_put_galipette_on_podium(void){
  // -- PUT GALIPETTE ON THE PODIUM --
  goto_xya(KX(1500-880), 1800, KA(-M_PI/2));
  ext_arm_clap();
  _delay_ms(500);
  goto_xya_rel(KX(200), 0, KA(0));
}

void strat_run_galipeur(void)
{
  ROME_LOG(&rome_paddock,DEBUG,"Go !!!");

  strat_homologation_galipeur();
#if 0
  //first, get out of starting area
  int16_t traj[] = {
    KX(1500-600), 1030,
    KX(1500-700), 930,
    KX(1500-700), 750,
    KX(1500-600), 650,
    KX(1500-200), 650,
    };
  goto_traj(traj,0); 
  

  galipeur_se_spots();
  galipeur_do_claps();
  galipeur_pick_south_spot();
  //go_pick_spot(KX(1500-1110),230);
  go_pick_spot(KX(1500-1310),650);
  if (robot_state.right_elev.nb_spots == 4)
    galipeur_discharge_pile_red();

  go_pick_spot(KX(1500-870),645);
  //galipeur_take_galipette_and_bulb();
  //galipeur_go_to_stairs();
  //galipeur_put_galipette_on_podium();
#endif

  _delay_ms(3000);
  ROME_SENDWAIT_ASSERV_ACTIVATE(&rome_asserv, 0);
  ROME_SENDWAIT_MECA_SET_POWER(&rome_meca, 0);
}

void strat_test_galipeur(void)
{
}

/****************************************************************************/

void strat_init_galipette(void)
{
}

void strat_prepare_galipette(void)
{
}

void strat_run_galipette(void)
{
}

void strat_test_galipette(void)
{
}

#if (defined GALIPEUR)
# define ROBOT_SUFFIX  galipeur
#elif (defined GALIPETTE)
# define ROBOT_SUFFIX  galipette
#else
# error Either GALIPEUR or GALIPETTE must be defined
#endif

#define CONCAT__(a,b)  a ## b
#define CONCAT_(a,b)  CONCAT__(a,b)
#define ROBOT_FUNCTION(f)  CONCAT_(f,ROBOT_SUFFIX)


void strat_init(void)
{
  ROBOT_FUNCTION(strat_init_)();
}

void strat_prepare(void)
{
  ROBOT_FUNCTION(strat_prepare_)();
}

void strat_run(void)
{
  ROBOT_FUNCTION(strat_run_)();
}

void strat_test(void)
{
  ROBOT_FUNCTION(strat_test_)();
}

