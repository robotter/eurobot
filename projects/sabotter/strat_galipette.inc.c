
#include "servos.h"

typedef enum {
  SABOTTER_PUMP_PWM = 1,
} sabotter_servo_t;

#define SABOTTER_PUMP_ON 6000

#define BUMPER_TO_CENTER_DIST 100 // distance from the edge of galipette to the center of the robot (in mm)

void set_speed(robot_speed_t s){
  switch (s){
    case RS_SLOW:
      ROME_SENDWAIT_ASSERV_SET_HTRAJ_XY_STEERING(ROME_DST_ASSERV, 1.5, 0.03);
      ROME_SENDWAIT_ASSERV_SET_HTRAJ_XY_CRUISE(ROME_DST_ASSERV, 15, 0.03);
      ROME_SENDWAIT_ASSERV_SET_HTRAJ_XY_STOP(ROME_DST_ASSERV, 0.5, 0.05);
      break;
    case RS_NORMAL:
      ROME_SENDWAIT_ASSERV_SET_HTRAJ_XY_STEERING(ROME_DST_ASSERV, 2.5, 0.05);
      ROME_SENDWAIT_ASSERV_SET_HTRAJ_XY_CRUISE(ROME_DST_ASSERV, 20, 0.05);
      ROME_SENDWAIT_ASSERV_SET_HTRAJ_XY_STOP(ROME_DST_ASSERV, 0.5, 0.05);
      break;
    case RS_FAST:
      ROME_SENDWAIT_ASSERV_SET_HTRAJ_XY_STEERING(ROME_DST_ASSERV, 3, 0.05);
      ROME_SENDWAIT_ASSERV_SET_HTRAJ_XY_CRUISE(ROME_DST_ASSERV, 80, 0.05);
      ROME_SENDWAIT_ASSERV_SET_HTRAJ_XY_STOP(ROME_DST_ASSERV, 0.5, 0.05);
      break;
    default:
      break;
  }
}

void pump_off(void) {
  ROME_LOG(ROME_DST_PADDOCK, INFO,"Galipette : pump off...");
  servo_set(SABOTTER_PUMP_PWM, 0);
}

void pump_on(void) {
  ROME_LOG(ROME_DST_PADDOCK, INFO,"Galipette : pump on !!!");
  servo_set(SABOTTER_PUMP_PWM, SABOTTER_PUMP_ON);
}

void galipette_autoset(robot_side_t robot_side, autoset_side_t table_side, float x, float y) {
  autoset(robot_side,table_side,x,y);
}

void strat_init(void)
{
  ROME_LOG(ROME_DST_PADDOCK, INFO,"Galipette : Strat init");
  // disable asserv
  ROME_SENDWAIT_ASSERV_ACTIVATE(ROME_DST_ASSERV, 0);

  pump_off();

  // set R3D2 parameters
  ROME_SENDWAIT_R3D2_SET_ROTATION(ROME_DST_ASSERV,350,25);

  for(;;) {
    idle();
    if(!robot_state.gyro_calibration)
      break;
  }

}

void strat_prepare(void)
{
  set_speed(RS_NORMAL);
  //initalise kx factor
  robot_kx = TEAM_SIDE_VALUE(-1, 1);

  ROME_LOG(ROME_DST_PADDOCK, DEBUG,"Strat prepare");

  idle_delay_ms(200);

  ROME_SENDWAIT_ASSERV_ACTIVATE(ROME_DST_ASSERV, 1);
  //boomotter is on the table !
  update_score(5);

  set_xya_wait(KX(0), 0, arfast(ROBOT_SIDE_BACK,TABLE_SIDE_MAIN));
  galipette_autoset(ROBOT_SIDE_BACK,AUTOSET_MAIN, KX(1500-BUMPER_TO_CENTER_DIST), 0);
  goto_xya(KX(1300), 0, arfast(ROBOT_SIDE_BACK,TABLE_SIDE_MAIN));
  goto_xya(KX(1300), 0, arfast(ROBOT_SIDE_BACK,TABLE_SIDE_UP));
  galipette_autoset(ROBOT_SIDE_BACK,AUTOSET_UP, 0, 2000-BUMPER_TO_CENTER_DIST);
  goto_xya(KX(1170), 1580, arfast(ROBOT_SIDE_BACK,TABLE_SIDE_UP));


  //experience is on the table !
  update_score(5);

  ROME_SENDWAIT_ASSERV_ACTIVATE(ROME_DST_ASSERV, 0);
  ROME_SENDWAIT_ASSERV_GYRO_INTEGRATION(ROME_DST_ASSERV, 0);
}

#define BOOMOTTER_MAX_AGE_US 3e6

bool boomotter_connected(void){
  return (robot_state.boom_age + BOOMOTTER_MAX_AGE_US > uptime_us());
}


#if 0 // 2018
order_result_t switch_on_boomotter(void){
  ROME_LOG(ROME_DST_PADDOCK, INFO,"go switching on boomotter");
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
  ROME_LOG(ROME_DST_PADDOCK, INFO,"go lauching the bee");
  ROME_SENDWAIT_ASSERV_SET_SERVO(ROME_DST_ASSERV, CUBE_CLAW_ELEVATOR, CUBE_CLAW_ELEVATOR_UP);
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
  float a = arfast(ROBOT_SIDE_AUX, TABLE_SIDE_UP);
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
  ROME_SENDWAIT_ASSERV_SET_SERVO(ROME_DST_ASSERV, CUBE_CLAW_ELEVATOR, CUBE_CLAW_ELEVATOR_UP);
  or = goto_pathfinding_node(PATHFINDING_GRAPH_NODE_CONSTRUCTION_AREA_CUBES,arfast(ROBOT_SIDE_CUBE_CLAW,TABLE_SIDE_UP));

  //take a cube
  ROME_SENDWAIT_ASSERV_SET_SERVO(ROME_DST_ASSERV, CUBE_CLAW_ELEVATOR, CUBE_CLAW_ELEVATOR_MID);
  or = goto_xya(KX(680),1250,arfast(ROBOT_SIDE_CUBE_CLAW,TABLE_SIDE_UP));
  idle_delay_ms(500);
  cube_claw_open();
  goto_xya(KX(680),1230,arfast(ROBOT_SIDE_CUBE_CLAW,TABLE_SIDE_UP));
  idle_delay_ms(500);
  ROME_SENDWAIT_ASSERV_SET_SERVO(ROME_DST_ASSERV, CUBE_CLAW_ELEVATOR, CUBE_CLAW_ELEVATOR_DOWN);
  idle_delay_ms(500);
  or = goto_xya(KX(680),1260,arfast(ROBOT_SIDE_CUBE_CLAW,TABLE_SIDE_UP));
  idle_delay_ms(500);
  cube_claw_close();
  idle_delay_ms(500);
  ROME_SENDWAIT_ASSERV_SET_SERVO(ROME_DST_ASSERV, CUBE_CLAW_ELEVATOR, CUBE_CLAW_ELEVATOR_UP);
  goto_xya(KX(680),1230,arfast(ROBOT_SIDE_CUBE_CLAW,TABLE_SIDE_UP));
  //take another one
  goto_xya(KX(1000),1500,arfast(ROBOT_SIDE_CUBE_CLAW,TABLE_SIDE_AUX));
  or = goto_xya(KX(880),1500,arfast(ROBOT_SIDE_CUBE_CLAW,TABLE_SIDE_AUX));
  ROME_SENDWAIT_ASSERV_SET_SERVO(ROME_DST_ASSERV, CUBE_CLAW_ELEVATOR, CUBE_CLAW_ELEVATOR_MID);
  idle_delay_ms(500);
  cube_claw_open();
  goto_xya(KX(910),1500,arfast(ROBOT_SIDE_CUBE_CLAW,TABLE_SIDE_AUX));
  idle_delay_ms(500);
  ROME_SENDWAIT_ASSERV_SET_SERVO(ROME_DST_ASSERV, CUBE_CLAW_ELEVATOR, CUBE_CLAW_ELEVATOR_DOWN);
  idle_delay_ms(500);
  or = goto_xya(KX(880),1500,arfast(ROBOT_SIDE_CUBE_CLAW,TABLE_SIDE_AUX));
  idle_delay_ms(500);
  cube_claw_close();
  idle_delay_ms(500);
  ROME_SENDWAIT_ASSERV_SET_SERVO(ROME_DST_ASSERV, CUBE_CLAW_ELEVATOR, CUBE_CLAW_ELEVATOR_UP);
  goto_xya(KX(1000),1500,arfast(ROBOT_SIDE_CUBE_CLAW,TABLE_SIDE_UP));

  //go constructing !
  goto_xya(KX(900),1800,arfast(ROBOT_SIDE_CUBE_CLAW,TABLE_SIDE_UP));
  ROME_SENDWAIT_ASSERV_SET_SERVO(ROME_DST_ASSERV, CUBE_CLAW_ELEVATOR, CUBE_CLAW_ELEVATOR_DOWN);
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
#endif

void strat_run(void)
{
  ROME_SENDWAIT_ASSERV_GYRO_INTEGRATION(ROME_DST_ASSERV, 1);
  ROME_SENDWAIT_ASSERV_ACTIVATE(ROME_DST_ASSERV, 1);

  ROME_LOG(ROME_DST_PADDOCK, DEBUG,"Gooooooo !!!!");

  order_result_t or = ORDER_FAILURE;
  or = goto_pathfinding_node(PATHFINDING_GRAPH_NODE_ACCELERATED_BLUE,arfast(ROBOT_SIDE_LEFT,TABLE_SIDE_UP));
  (void) or;

  ROME_LOG(ROME_DST_PADDOCK, DEBUG,"Push blue atom");

  goto_xya(KX(-150),1860,arfast(ROBOT_SIDE_LEFT,TABLE_SIDE_UP));
  goto_xya(KX(-300),1860,arfast(ROBOT_SIDE_LEFT,TABLE_SIDE_UP));
  goto_xya(KX(-150),1800,arfast(ROBOT_SIDE_LEFT,TABLE_SIDE_UP));

  goto_xya(KX(-200),1800,arfast(ROBOT_SIDE_BACK,TABLE_SIDE_UP));
  goto_xya(KX(-200),1860,arfast(ROBOT_SIDE_BACK,TABLE_SIDE_UP));
  galipette_autoset(ROBOT_SIDE_BACK,AUTOSET_UP, 0, 2000-BUMPER_TO_CENTER_DIST-30);
  goto_xya(KX(-200),1800,arfast(ROBOT_SIDE_BACK,TABLE_SIDE_UP));

  ROME_LOG(ROME_DST_PADDOCK, DEBUG,"Take goldenium");
  or = goto_pathfinding_node(PATHFINDING_GRAPH_NODE_GOLDENIUM,arfast(ROBOT_SIDE_BACK,TABLE_SIDE_UP));
  pump_on();
  goto_xya(KX(-740),(1880),arfast(ROBOT_SIDE_BACK,TABLE_SIDE_UP));
  idle_delay_ms(500);
  goto_xya(KX(-740),1800,arfast(ROBOT_SIDE_BACK,TABLE_SIDE_UP));

  ROME_LOG(ROME_DST_PADDOCK, DEBUG,"Put goldenium in the balance");
  or = goto_pathfinding_node(PATHFINDING_GRAPH_NODE_BALANCE,arfast(ROBOT_SIDE_BACK,TABLE_SIDE_DOWN));
  goto_xya(KX(140),520,arfast(ROBOT_SIDE_BACK,TABLE_SIDE_DOWN));
  pump_off();
  idle_delay_ms(2000);
  goto_xya(KX(200),700,arfast(ROBOT_SIDE_BACK,TABLE_SIDE_DOWN));
#if 0 // 2018
  switch_on_boomotter();
#endif


#if 0 // 2018
  order_result_t or = ORDER_FAILURE;
  while (or != ORDER_SUCCESS){
    or = launch_bee();
    idle();
  }
#endif

  //look_at_the_card();

  //take_cubes_in_front_of_contruction_area();

  //idle_delay_ms(2000);
  //cube_claw_open();
  //ROME_SENDWAIT_ASSERV_SET_SERVO(ROME_DST_ASSERV, CUBE_CLAW_ELEVATOR, CUBE_CLAW_ELEVATOR_UP);
 
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


  //go back for prog
  for(;;){
    idle();

    if (robot_state.match_time > 70)
      break;
  }
  idle_delay_ms(500);
  or = goto_pathfinding_node(PATHFINDING_GRAPH_NODE_GALIPETTE_START,arfast(ROBOT_SIDE_BACK,TABLE_SIDE_UP));
  goto_xya(KX(1300),1800,arfast(ROBOT_SIDE_BACK,TABLE_SIDE_UP));

  ROME_LOG(ROME_DST_PADDOCK, INFO,"That's all folks !");
  ROME_SENDWAIT_ASSERV_ACTIVATE(ROME_DST_ASSERV, 0);
}

void strat_test(void)
{
  ROME_LOG(ROME_DST_PADDOCK, INFO,"Strat test stuff");
  for(;;) {
    update_rome_interfaces();
    if(!robot_state.gyro_calibration)
      break;
  }

  set_speed(RS_NORMAL);
  ROME_SENDWAIT_ASSERV_ACTIVATE(ROME_DST_ASSERV, 1);
  ROME_SENDWAIT_ASSERV_GYRO_INTEGRATION(ROME_DST_ASSERV, 1);

  for(;;){
    idle_delay_ms(200);
  }
}
