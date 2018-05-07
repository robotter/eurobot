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
#define CUBE_CLAW_ELEVATOR_BUTTON_WITHCUBE 2800
#define CUBE_CLAW_ELEVATOR_MID 3250
#define CUBE_CLAW_ELEVATOR_UP 4000

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

void strat_init(void)
{
  ROME_LOG(&rome_paddock,INFO,"Galipette : Strat init");
  // disable asserv
  ROME_SENDWAIT_ASSERV_ACTIVATE(&rome_asserv, 0);

  ROME_SENDWAIT_ASSERV_SET_SERVO(&rome_asserv, CUBE_CLAW_ELEVATOR, CUBE_CLAW_ELEVATOR_BUTTON);
  cube_claw_start();

  // set R3D2 parameters
  ROME_SENDWAIT_R3D2_SET_ROTATION(&rome_asserv,350,25);

  for(;;) {
    idle();
    if(!robot_state.gyro_calibration)
      break;
  }


  cube_claw_open();
  ROME_SENDWAIT_ASSERV_SET_SERVO(&rome_asserv, CUBE_CLAW_ELEVATOR, CUBE_CLAW_ELEVATOR_DOWN);
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

  set_xya_wait(KX(0), 0, arfast(ROBOT_SIDE_BACK,TABLE_SIDE_MAIN));
  autoset(ROBOT_SIDE_BACK,AUTOSET_MAIN, KX(1500-BUMPER_TO_CENTER_DIST), 0);
  goto_xya(KX(1200), 0, arfast(ROBOT_SIDE_BACK,TABLE_SIDE_MAIN));
  goto_xya(KX(1200), 0, arfast(ROBOT_SIDE_BACK,TABLE_SIDE_UP));
  autoset(ROBOT_SIDE_BACK,AUTOSET_UP, 0, 2000-BUMPER_TO_CENTER_DIST);
  goto_xya(KX(1200), 1800, arfast(ROBOT_SIDE_BACK,TABLE_SIDE_UP));
  goto_xya(KX(1350), 1800, arfast(ROBOT_SIDE_CUBE_CLAW,TABLE_SIDE_AUX));


  ROME_SENDWAIT_ASSERV_ACTIVATE(&rome_asserv, 0);
  ROME_SENDWAIT_ASSERV_GYRO_INTEGRATION(&rome_asserv, 0);
}


order_result_t switch_on_boomotter(bool cube_present){
  ROME_LOG(&rome_paddock,INFO,"go switching on boomotter");
  order_result_t or = ORDER_FAILURE;
  //go switching domotic panel first
  if (cube_present){
    ROME_SENDWAIT_ASSERV_SET_SERVO(&rome_asserv, CUBE_CLAW_ELEVATOR, CUBE_CLAW_ELEVATOR_BUTTON_WITHCUBE);
  }
  else{
    cube_claw_start();
    ROME_SENDWAIT_ASSERV_SET_SERVO(&rome_asserv, CUBE_CLAW_ELEVATOR, CUBE_CLAW_ELEVATOR_BUTTON);
  }
  or = goto_pathfinding_node(PATHFINDING_GRAPH_NODE_CONSTRUCTION_AREA,arfast(ROBOT_SIDE_CUBE_CLAW,TABLE_SIDE_UP));
  int16_t traj[] = {
      KX(370), 1750,
      KX(370), 1810,
      };
  or = goto_traj(traj, arfast(ROBOT_SIDE_CUBE_CLAW,TABLE_SIDE_UP));
  //TODO check that boomotter is on before updating score
  update_score(25);
  or = goto_xya(KX(370),1750, arfast(ROBOT_SIDE_CUBE_CLAW,TABLE_SIDE_UP));
  return or;
}

order_result_t launch_bee(void){
  ROME_LOG(&rome_paddock,INFO,"go lauching the bee");
  order_result_t or = ORDER_FAILURE;
  or = goto_pathfinding_node(PATHFINDING_GRAPH_NODE_BEE,arfast(ROBOT_SIDE_BACK,TABLE_SIDE_DOWN));
  if (or != ORDER_SUCCESS)
    return or;

  or = goto_xya(KX(1300),200, arfast(ROBOT_SIDE_BACK,TABLE_SIDE_DOWN));
  autoset(ROBOT_SIDE_BACK,AUTOSET_DOWN, 0, BUMPER_TO_CENTER_DIST);
  or = goto_xya(KX(1300),200, arfast(ROBOT_SIDE_BACK,TABLE_SIDE_DOWN));
  autoset(ROBOT_SIDE_BACK,AUTOSET_MAIN, KX(1500-BUMPER_TO_CENTER_DIST), 0);
  or = goto_xya(KX(1300),150, arfast(ROBOT_SIDE_BACK,TABLE_SIDE_MAIN));
  or = goto_xya(KX(1100),150, arfast(ROBOT_SIDE_BACK,TABLE_SIDE_MAIN));
  update_score(50);
  or = goto_xya_synced(KX(1250),300, arfast(ROBOT_SIDE_RIGHT,TABLE_SIDE_UP));

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
  or = goto_pathfinding_node(PATHFINDING_GRAPH_NODE_CONSTRUCTION_AREA_CUBES,arfast(ROBOT_SIDE_CUBE_CLAW,TABLE_SIDE_UP));
  return or;
}

void strat_run(void)
{
  ROME_SENDWAIT_ASSERV_GYRO_INTEGRATION(&rome_asserv, 1);
  ROME_SENDWAIT_ASSERV_ACTIVATE(&rome_asserv, 1);
  cube_claw_close();

  //boomotter is on the table !
  update_score(5);
  //bee is on the table !
  update_score(5);

  switch_on_boomotter(true);
  ROME_SENDWAIT_ASSERV_SET_SERVO(&rome_asserv, CUBE_CLAW_ELEVATOR, CUBE_CLAW_ELEVATOR_UP);

  launch_bee();

  look_at_the_card();

  take_cubes_in_front_of_contruction_area();

  //idle_delay_ms(2000);
  //cube_claw_open();
  //ROME_SENDWAIT_ASSERV_SET_SERVO(&rome_asserv, CUBE_CLAW_ELEVATOR, CUBE_CLAW_ELEVATOR_UP);

  ROME_LOG(&rome_paddock,INFO,"That's all folks !");
  _delay_ms(3000);
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
    goto_xya(500,-500,0);
    idle_delay_ms(10000);
    goto_xya(0,0,0);
    idle_delay_ms(10000);
  }
}
