
#include "servos.h"

#define ROBOT_SIDE_CUBE_CLAW (ROBOT_SIDE_LEFT)

typedef enum{
  NONE = 0,
  CUBE_CLAW_LEFT = 1,
  CUBE_CLAW_ELEVATOR = 2,
  CUBE_CLAW_RIGHT = 3,
} cube_claw_servo_t;

#define CUBE_CLAW_LEFT_START 1500
#define CUBE_CLAW_LEFT_CLOSED 2100
#define CUBE_CLAW_LEFT_OPENED 3000

#define CUBE_CLAW_RIGHT_START 3000
#define CUBE_CLAW_RIGHT_CLOSED 2400
#define CUBE_CLAW_RIGHT_OPENED 1500

#define CUBE_CLAW_ELEVATOR_DOWN 1300
#define CUBE_CLAW_ELEVATOR_BUTTON 2600
#define CUBE_CLAW_ELEVATOR_BUTTON_WITHCUBE 3800
#define CUBE_CLAW_ELEVATOR_MID 3250
#define CUBE_CLAW_ELEVATOR_UP 4000

typedef enum {
  SABOTTER_SERVO_BEE_LAUCHER = 3,
} sabotter_servo_t;

#define SABOTTER_SERVO_BEE_LAUCHER_DOWN 2450
#define SABOTTER_SERVO_BEE_LAUCHER_RIGHT 1600
#define SABOTTER_SERVO_BEE_LAUCHER_LEFT 3300

#define BUMPER_TO_CENTER_DIST 100 // distance from the edge of galipette to the center of the robot (in mm)

void set_speed(robot_speed_t s){
  switch (s){
    case RS_SLOW:
      ROME_SENDWAIT_ASSERV_SET_HTRAJ_XY_STEERING(&rome_asserv, 1.5, 0.03);
      ROME_SENDWAIT_ASSERV_SET_HTRAJ_XY_CRUISE(&rome_asserv, 15, 0.03);
      ROME_SENDWAIT_ASSERV_SET_HTRAJ_XY_STOP(&rome_asserv, 0.5, 0.05);
      break;
    case RS_NORMAL:
      ROME_SENDWAIT_ASSERV_SET_HTRAJ_XY_STEERING(&rome_asserv, 2.5, 0.05);
      ROME_SENDWAIT_ASSERV_SET_HTRAJ_XY_CRUISE(&rome_asserv, 20, 0.05);
      ROME_SENDWAIT_ASSERV_SET_HTRAJ_XY_STOP(&rome_asserv, 0.5, 0.05);
      break;
    case RS_FAST:
      ROME_SENDWAIT_ASSERV_SET_HTRAJ_XY_STEERING(&rome_asserv, 3, 0.05);
      ROME_SENDWAIT_ASSERV_SET_HTRAJ_XY_CRUISE(&rome_asserv, 80, 0.05);
      ROME_SENDWAIT_ASSERV_SET_HTRAJ_XY_STOP(&rome_asserv, 0.5, 0.05);
      break;
    default:
      break;
  }
}

void cube_claw_start(void){
  ROME_SENDWAIT_ASSERV_SET_SERVO(&rome_asserv, CUBE_CLAW_LEFT, CUBE_CLAW_LEFT_START);
  ROME_SENDWAIT_ASSERV_SET_SERVO(&rome_asserv, CUBE_CLAW_RIGHT, CUBE_CLAW_RIGHT_START);
}

void cube_claw_close(void){
  ROME_SENDWAIT_ASSERV_SET_SERVO(&rome_asserv, CUBE_CLAW_LEFT, CUBE_CLAW_LEFT_CLOSED);
  ROME_SENDWAIT_ASSERV_SET_SERVO(&rome_asserv, CUBE_CLAW_RIGHT, CUBE_CLAW_RIGHT_CLOSED);
}

void cube_claw_open(void){
  ROME_SENDWAIT_ASSERV_SET_SERVO(&rome_asserv, CUBE_CLAW_LEFT, CUBE_CLAW_LEFT_OPENED);
  ROME_SENDWAIT_ASSERV_SET_SERVO(&rome_asserv, CUBE_CLAW_RIGHT, CUBE_CLAW_RIGHT_OPENED);
}

void bee_launcher_down(void) {
  servo_set(SABOTTER_SERVO_BEE_LAUCHER, SABOTTER_SERVO_BEE_LAUCHER_DOWN);
}

void bee_launcher_push(void) {
  switch(robot_state.team) {
    case TEAM_GREEN:
      servo_set(SABOTTER_SERVO_BEE_LAUCHER, SABOTTER_SERVO_BEE_LAUCHER_RIGHT);
      break;
    case TEAM_ORANGE:
      servo_set(SABOTTER_SERVO_BEE_LAUCHER, SABOTTER_SERVO_BEE_LAUCHER_LEFT);
      break;
    
    default:
      break;
  }
}

void bee_launcher_papush(void) {
  switch(robot_state.team) {
    case TEAM_ORANGE:
      servo_set(SABOTTER_SERVO_BEE_LAUCHER, SABOTTER_SERVO_BEE_LAUCHER_RIGHT);
      break;
    case TEAM_GREEN:
      servo_set(SABOTTER_SERVO_BEE_LAUCHER, SABOTTER_SERVO_BEE_LAUCHER_LEFT);
      break;
    
    default:
      break;
  }
}

void galipette_autoset(robot_side_t robot_side, autoset_side_t table_side, float x, float y) {
  bee_launcher_push();
  autoset(robot_side,table_side,x,y);
}

void galipette_autoset_papush(robot_side_t robot_side, autoset_side_t table_side, float x, float y) {
  bee_launcher_papush();
  autoset(robot_side,table_side,x,y);
}

void strat_init(void)
{
  ROME_LOG(&rome_paddock,INFO,"Galipette : Strat init");
  // disable asserv
  ROME_SENDWAIT_ASSERV_ACTIVATE(&rome_asserv, 0);

  ROME_SENDWAIT_ASSERV_SET_SERVO(&rome_asserv, CUBE_CLAW_ELEVATOR, CUBE_CLAW_ELEVATOR_UP);
  cube_claw_open();

  // set R3D2 parameters
  ROME_SENDWAIT_R3D2_SET_ROTATION(&rome_asserv,350,25);

  for(;;) {
    idle();
    if(!robot_state.gyro_calibration)
      break;
  }


  cube_claw_open();
  ROME_SENDWAIT_ASSERV_SET_SERVO(&rome_asserv, CUBE_CLAW_ELEVATOR, CUBE_CLAW_ELEVATOR_DOWN);
  bee_launcher_down();
}

void strat_prepare(void)
{
  set_speed(RS_NORMAL);
  //initalise kx factor
  robot_kx = robot_state.team == TEAM_GREEN ? -1 : 1;

  ROME_LOG(&rome_paddock,DEBUG,"Strat prepare");

  cube_claw_close();
  idle_delay_ms(200);
  ROME_SENDWAIT_ASSERV_SET_SERVO(&rome_asserv, CUBE_CLAW_ELEVATOR, CUBE_CLAW_ELEVATOR_UP);

  ROME_SENDWAIT_ASSERV_ACTIVATE(&rome_asserv, 1);
  //boomotter is on the table !
  update_score(5);

  set_xya_wait(KX(0), 0, arfast(ROBOT_SIDE_BACK,TABLE_SIDE_MAIN));
  galipette_autoset(ROBOT_SIDE_BACK,AUTOSET_MAIN, KX(1500-BUMPER_TO_CENTER_DIST), 0);
  goto_xya(KX(1000), 0, arfast(ROBOT_SIDE_BACK,TABLE_SIDE_MAIN));
  goto_xya(KX(1000), 0, arfast(ROBOT_SIDE_BACK,TABLE_SIDE_UP));
  galipette_autoset(ROBOT_SIDE_BACK,AUTOSET_UP, 0, 2000-BUMPER_TO_CENTER_DIST);
  goto_xya(KX(1000), 1800, arfast(ROBOT_SIDE_BACK,TABLE_SIDE_UP));

  bee_launcher_down();
  goto_xya(KX(1350), 1800, arfast(ROBOT_SIDE_CUBE_CLAW,TABLE_SIDE_AUX));

  //bee is on the table !
  update_score(5);

  ROME_SENDWAIT_ASSERV_ACTIVATE(&rome_asserv, 0);
  ROME_SENDWAIT_ASSERV_GYRO_INTEGRATION(&rome_asserv, 0);
}

#define BOOMOTTER_MAX_AGE_US 3e6

bool boomotter_connected(void){
  return (robot_state.boom_age + BOOMOTTER_MAX_AGE_US > uptime_us());
}


order_result_t switch_on_boomotter(void){
  ROME_LOG(&rome_paddock,INFO,"go switching on boomotter");
  order_result_t or = ORDER_FAILURE;
  //go switching domotic panel first
  or = goto_pathfinding_node(PATHFINDING_GRAPH_NODE_CONSTRUCTION_AREA,arfast(ROBOT_SIDE_BACK,TABLE_SIDE_UP));
  or = goto_xya(KX(370),1750, arfast(ROBOT_SIDE_BACK,TABLE_SIDE_UP));
  uint32_t try_time = 0;
  for(int16_t y = 1890; y < 2000; y+=30){
    or = goto_xya_wait(KX(370), y, arfast(ROBOT_SIDE_BACK,TABLE_SIDE_UP), 2000);
    try_time = uptime_us(); 

    bool done = false;
    for(;;){
      idle(); 
      if (uptime_us() - try_time > BOOMOTTER_MAX_AGE_US)
        break;

      if (boomotter_connected()){
        done = true;
        update_score(25);
        break;
      }
    }
    if (done)
      break;
  }
  or = goto_xya(KX(370),1850, arfast(ROBOT_SIDE_BACK,TABLE_SIDE_UP));
  galipette_autoset(ROBOT_SIDE_BACK,AUTOSET_UP, 0, 2000-BUMPER_TO_CENTER_DIST);
  or = goto_xya(KX(370),1750, arfast(ROBOT_SIDE_BACK,TABLE_SIDE_UP));
  bee_launcher_down();
  return or;
}

order_result_t launch_bee(void){
  ROME_LOG(&rome_paddock,INFO,"go lauching the bee");
  ROME_SENDWAIT_ASSERV_SET_SERVO(&rome_asserv, CUBE_CLAW_ELEVATOR, CUBE_CLAW_ELEVATOR_UP);
  order_result_t or = ORDER_FAILURE;
  or = goto_pathfinding_node(PATHFINDING_GRAPH_NODE_BEE,arfast(ROBOT_SIDE_BACK,TABLE_SIDE_UP));
  if (or != ORDER_SUCCESS){
    goto_pathfinding_node(PATHFINDING_GRAPH_NODE_WATER_DISPENSER_NEAR,arfast(ROBOT_SIDE_BACK,TABLE_SIDE_DOWN));
    return or;
  }

  or = goto_xya(KX(1350),150, arfast(ROBOT_SIDE_BACK,TABLE_SIDE_MAIN));
  galipette_autoset_papush(ROBOT_SIDE_BACK,AUTOSET_MAIN, KX(1500-BUMPER_TO_CENTER_DIST), 0);
  or = goto_xya(KX(1350),150, arfast(ROBOT_SIDE_BACK,TABLE_SIDE_MAIN));
  bee_launcher_down();
  or = goto_xya(KX(1350),150, arfast(ROBOT_SIDE_BACK,TABLE_SIDE_DOWN));
  galipette_autoset(ROBOT_SIDE_BACK,AUTOSET_DOWN, 0, BUMPER_TO_CENTER_DIST);
  or = goto_xya(KX(1350),150, arfast(ROBOT_SIDE_BACK,TABLE_SIDE_DOWN));
  bee_launcher_down();
  or = goto_xya(KX(1350),200, arfast(ROBOT_SIDE_BACK,TABLE_SIDE_MAIN));
  bee_launcher_push();
  or = goto_xya(KX(1350),140, arfast(ROBOT_SIDE_BACK,TABLE_SIDE_MAIN));
  idle_delay_ms(200);
  float a=0;
  if (robot_state.team == TEAM_GREEN)
    a = arfast(ROBOT_SIDE_LEFT,TABLE_SIDE_UP);
  else
    a = arfast(ROBOT_SIDE_RIGHT,TABLE_SIDE_UP);
  or = goto_xya(KX(1100),140, a);
  update_score(50);
  or = goto_xya(KX(1250),300, a);
  bee_launcher_down();

  return or;

}

order_result_t look_at_the_card(void){
  order_result_t or = ORDER_FAILURE;
  or = goto_pathfinding_node(PATHFINDING_GRAPH_NODE_MIDDLE_MIDH,arfast(ROBOT_SIDE_CUBE_CLAW,TABLE_SIDE_UP));
  //wait for jeVois to see the card
  return or;
}


order_result_t take_cubes_in_front_of_contruction_area(void){
  order_result_t or = ORDER_FAILURE;
  ROME_SENDWAIT_ASSERV_SET_SERVO(&rome_asserv, CUBE_CLAW_ELEVATOR, CUBE_CLAW_ELEVATOR_UP);
  or = goto_pathfinding_node(PATHFINDING_GRAPH_NODE_CONSTRUCTION_AREA_CUBES,arfast(ROBOT_SIDE_CUBE_CLAW,TABLE_SIDE_UP));

  //take a cube
  ROME_SENDWAIT_ASSERV_SET_SERVO(&rome_asserv, CUBE_CLAW_ELEVATOR, CUBE_CLAW_ELEVATOR_MID);
  or = goto_xya(KX(680),1250,arfast(ROBOT_SIDE_CUBE_CLAW,TABLE_SIDE_UP));
  idle_delay_ms(500);
  cube_claw_open();
  goto_xya(KX(680),1230,arfast(ROBOT_SIDE_CUBE_CLAW,TABLE_SIDE_UP));
  idle_delay_ms(500);
  ROME_SENDWAIT_ASSERV_SET_SERVO(&rome_asserv, CUBE_CLAW_ELEVATOR, CUBE_CLAW_ELEVATOR_DOWN);
  idle_delay_ms(500);
  or = goto_xya(KX(680),1260,arfast(ROBOT_SIDE_CUBE_CLAW,TABLE_SIDE_UP));
  idle_delay_ms(500);
  cube_claw_close();
  idle_delay_ms(500);
  ROME_SENDWAIT_ASSERV_SET_SERVO(&rome_asserv, CUBE_CLAW_ELEVATOR, CUBE_CLAW_ELEVATOR_UP);
  goto_xya(KX(680),1230,arfast(ROBOT_SIDE_CUBE_CLAW,TABLE_SIDE_UP));
  //take another one
  goto_xya(KX(1000),1500,arfast(ROBOT_SIDE_CUBE_CLAW,TABLE_SIDE_AUX));
  or = goto_xya(KX(880),1500,arfast(ROBOT_SIDE_CUBE_CLAW,TABLE_SIDE_AUX));
  ROME_SENDWAIT_ASSERV_SET_SERVO(&rome_asserv, CUBE_CLAW_ELEVATOR, CUBE_CLAW_ELEVATOR_MID);
  idle_delay_ms(500);
  cube_claw_open();
  goto_xya(KX(910),1500,arfast(ROBOT_SIDE_CUBE_CLAW,TABLE_SIDE_AUX));
  idle_delay_ms(500);
  ROME_SENDWAIT_ASSERV_SET_SERVO(&rome_asserv, CUBE_CLAW_ELEVATOR, CUBE_CLAW_ELEVATOR_DOWN);
  idle_delay_ms(500);
  or = goto_xya(KX(880),1500,arfast(ROBOT_SIDE_CUBE_CLAW,TABLE_SIDE_AUX));
  idle_delay_ms(500);
  cube_claw_close();
  idle_delay_ms(500);
  ROME_SENDWAIT_ASSERV_SET_SERVO(&rome_asserv, CUBE_CLAW_ELEVATOR, CUBE_CLAW_ELEVATOR_UP);
  goto_xya(KX(1000),1500,arfast(ROBOT_SIDE_CUBE_CLAW,TABLE_SIDE_UP));

  //go constructing !
  goto_xya(KX(900),1800,arfast(ROBOT_SIDE_CUBE_CLAW,TABLE_SIDE_UP));
  ROME_SENDWAIT_ASSERV_SET_SERVO(&rome_asserv, CUBE_CLAW_ELEVATOR, CUBE_CLAW_ELEVATOR_DOWN);
  idle_delay_ms(500);
  cube_claw_open();
  goto_xya(KX(910),1810,arfast(ROBOT_SIDE_CUBE_CLAW,TABLE_SIDE_UP));
  goto_xya(KX(910),1700,arfast(ROBOT_SIDE_CUBE_CLAW,TABLE_SIDE_UP));

  //yeah we scored !!!!!
  update_score(1+2+3);

  for(int i=0; i<10; i++) {
    cube_claw_open();
    idle_delay_ms(500);
    cube_claw_close();
    idle_delay_ms(500);
  }

  return or;
}

void strat_run(void)
{
  ROME_SENDWAIT_ASSERV_GYRO_INTEGRATION(&rome_asserv, 1);
  ROME_SENDWAIT_ASSERV_ACTIVATE(&rome_asserv, 1);
  cube_claw_close();

  //wait for galipeur to go
  idle_delay_ms(1000);

  switch_on_boomotter();

  //wait for galipeur to go trough the table to go for opposite dispensers
  for(;;){
    idle();
    if(KX(robot_state.partner_pos.x) < -100)
      break;

    if (robot_state.match_time > 50)
      break;
  }

  order_result_t or = ORDER_FAILURE;
  while (or != ORDER_SUCCESS){
    or = launch_bee();
    idle();
  }

  //look_at_the_card();

  //take_cubes_in_front_of_contruction_area();

  //idle_delay_ms(2000);
  //cube_claw_open();
  //ROME_SENDWAIT_ASSERV_SET_SERVO(&rome_asserv, CUBE_CLAW_ELEVATOR, CUBE_CLAW_ELEVATOR_UP);
 
  //reswitch on boomotter if it was switched off
  //for(;;){
  //  for(;;){
  //    idle();
  //    if(KX(robot_state.partner_pos.x) < 0)
  //      break;

  //    if(KX(robot_state.partner_pos.y) > 1300)
  //      break;
  //  }
  //  if(!boomotter_connected()){
  //    switch_on_boomotter(false);
  //    idle_delay_ms(BOOMOTTER_MAX_AGE);
  //  }
  //  idle_delay_ms(1000);
  //}

  idle_delay_ms(500);
  goto_xya(KX(1300),500,arfast(ROBOT_SIDE_BACK,TABLE_SIDE_MAIN));

  ROME_LOG(&rome_paddock,INFO,"That's all folks !");
  idle_delay_ms(3000);
  ROME_SENDWAIT_ASSERV_ACTIVATE(&rome_asserv, 0);
}

void strat_test(void)
{
  ROME_LOG(&rome_paddock,INFO,"Strat test stuff");
  for(;;) {
    update_rome_interfaces();
    if(!robot_state.gyro_calibration)
      break;
  }

  set_speed(RS_NORMAL);
  ROME_SENDWAIT_ASSERV_ACTIVATE(&rome_asserv, 1);
  ROME_SENDWAIT_ASSERV_GYRO_INTEGRATION(&rome_asserv, 1);

  for(;;){
    idle_delay_ms(200);
  }
}
