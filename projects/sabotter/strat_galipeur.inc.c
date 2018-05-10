#define AUTOSET_OFFSET 112

#define ROBOT_SIDE_BALLEATER (ROBOT_SIDE_LEFT)
#define ROBOT_SIDE_TURBINE (ROBOT_SIDE_RIGHT)

void _wait_meca_ready(void){
  //force meca state busy
  robot_state.meca_state = ROME_ENUM_MECA_STATE_BUSY;
  ROME_LOG(&rome_paddock,DEBUG,"strat : wait meca ready");
  for (;;){
    idle();
    if((robot_state.meca_state == ROME_ENUM_MECA_STATE_READY)){
      ROME_LOG(&rome_paddock,DEBUG,"strat : meca ready");
      return;
    }
  }
}

void _wait_meca_ground_clear(void){
  //force meca state busy
  robot_state.meca_state = ROME_ENUM_MECA_STATE_BUSY;
  ROME_LOG(&rome_paddock,DEBUG,"strat : wait meca ground clear");
  for (;;){
    idle();
    if((robot_state.meca_state == ROME_ENUM_MECA_STATE_GROUND_CLEAR ||
      robot_state.meca_state == ROME_ENUM_MECA_STATE_READY)){
      ROME_LOG(&rome_paddock,DEBUG,"strat : meca ground clear");
      return;
    }
  }
}

bool cylinder_is_empty(void){
  return (robot_state.cylinder_nb_empty == robot_state.cylinder_nb_slots);
}

void strat_init(void)
{
  ROME_LOG(&rome_paddock,INFO,"Galipeur : Strat init");
  // disable asserv
  ROME_SENDWAIT_ASSERV_ACTIVATE(&rome_asserv, 0);
  ROME_SENDWAIT_ASSERV_SET_HTRAJ_XY_STEERING(&rome_asserv, 2.5, 0.1);
  ROME_SENDWAIT_ASSERV_SET_HTRAJ_XY_CRUISE(&rome_asserv, 20, 0.1);

  // initialize meca
  ROME_LOG(&rome_paddock,INFO,"Init meca");
  ROME_SENDWAIT_MECA_SET_POWER(&rome_meca, 1);
  // set R3D2 parameters
  ROME_SENDWAIT_R3D2_SET_ROTATION(&rome_asserv,0,25);

  for(;;) {
    update_rome_interfaces();
    if(!robot_state.gyro_calibration)
      break;
  }

  // set R3D2 parameters
  ROME_SENDWAIT_R3D2_SET_ROTATION(&rome_asserv,200,25);
}

void strat_prepare(void)
{
  //initalise kx factor
  robot_kx = robot_state.team == TEAM_GREEN ? -1 : 1;

  //send color to meca
  ROME_SENDWAIT_MECA_SET_ROBOT_COLOR(&rome_meca, robot_state.team == TEAM_GREEN);

  ROME_LOG(&rome_paddock,DEBUG,"Strat prepare");
  // initialize asserv
  ROME_SENDWAIT_ASSERV_ACTIVATE(&rome_asserv, 1);
  set_xya_wait(0,0,0);
  ROME_SENDWAIT_ASSERV_GOTO_XY(&rome_asserv, 0, 0, 0);
  ROME_SENDWAIT_ASSERV_SET_HTRAJ_XY_STEERING(&rome_asserv, 1.5, 0.03);
  ROME_SENDWAIT_ASSERV_SET_HTRAJ_XY_CRUISE(&rome_asserv, 15, 0.03);

  // autoset robot
  // x in starting area
  set_xya_wait(KX(0), 0, arfast(ROBOT_SIDE_BACK,TABLE_SIDE_MAIN));
  autoset(ROBOT_SIDE_BACK,AUTOSET_MAIN, KX(1500-AUTOSET_OFFSET), 0);
  goto_xya(KX(1000), 0, arfast(ROBOT_SIDE_BACK,TABLE_SIDE_MAIN));
  // y on building area
  goto_xya(KX(800), 400, arfast(ROBOT_SIDE_BACK,TABLE_SIDE_UP));
  autoset(ROBOT_SIDE_BACK,AUTOSET_UP, 0, 2000-AUTOSET_OFFSET);

  // check the state of the cylinder
  _wait_meca_ready();
  ROME_SENDWAIT_MECA_CMD(&rome_meca,ROME_ENUM_MECA_COMMAND_CHECK_EMPTY);

  //evade the border, leaving space for galipette
  goto_xya(KX(900), 1600, arfast(ROBOT_SIDE_BACK,TABLE_SIDE_UP));

  //go in front of starting area
  goto_xya(KX(1300), 1500, arfast(ROBOT_SIDE_BALLEATER,TABLE_SIDE_DOWN));

  //wait for meca to end checking cylinder
  _wait_meca_ready();
  if (!cylinder_is_empty()){
    _wait_meca_ready();
    ROME_SENDWAIT_MECA_CMD(&rome_meca,ROME_ENUM_MECA_COMMAND_TRASH_BEGINMATCH);
  }

  //wait for meca to end all orders before shutting down asserv
  _wait_meca_ready();

  ROME_SENDWAIT_ASSERV_ACTIVATE(&rome_asserv, 0);
  ROME_SENDWAIT_ASSERV_GYRO_INTEGRATION(&rome_asserv, 0);

}


void set_speed(robot_speed_t s){
  switch (s){
    case RS_VERYSLOW:
      ROME_SENDWAIT_ASSERV_SET_HTRAJ_XY_STEERING(&rome_asserv, 1.5, 0.03);
      ROME_SENDWAIT_ASSERV_SET_HTRAJ_XY_CRUISE(&rome_asserv, 10, 0.03);
      break;
    case RS_SLOW:
      ROME_SENDWAIT_ASSERV_SET_HTRAJ_XY_STEERING(&rome_asserv, 1.5, 0.03);
      ROME_SENDWAIT_ASSERV_SET_HTRAJ_XY_CRUISE(&rome_asserv, 15, 0.03);
      break;
    case RS_NORMAL:
      ROME_SENDWAIT_ASSERV_SET_HTRAJ_XY_STEERING(&rome_asserv, 2.5, 0.1);
      ROME_SENDWAIT_ASSERV_SET_HTRAJ_XY_CRUISE(&rome_asserv, 20, 0.1);
      break;
    case RS_FAST:
      ROME_SENDWAIT_ASSERV_SET_HTRAJ_XY_STEERING(&rome_asserv, 5, 0.1);
      ROME_SENDWAIT_ASSERV_SET_HTRAJ_XY_CRUISE(&rome_asserv, 60, 0.05);
      break;
    default:
      break;
  }
}

float compute_throw_angle(int16_t x, int16_t y){
  float target = atan2(KX(1320)-KX(x),2200-y);
  float robot = arfast(ROBOT_SIDE_TURBINE,TABLE_SIDE_UP);
  //if (robot_state.team == TEAM_GREEN)
  //  return robot + target;
  //else
    return robot - target;

}

typedef enum{
  DISPENSER_NEAR = 0,
  DISPENSER_FAR,
} dispenser_t;

order_result_t take_water(dispenser_t dispenser){
  order_result_t or;

  //save the amount of water we already have to check if we managed to "open" a dispenser
  uint8_t nb_empty = robot_state.cylinder_nb_empty;

  //dispenser positions
  int16_t near_pos = 2000-840;
  int16_t far_pos = -(1500-620);

  //balleater configuration
  int16_t balleater_depth = AUTOSET_OFFSET + 35;
  int16_t approach_depth = balleater_depth + 60;
  int16_t approach_side = 150;

  int16_t traj1[4];
  int16_t traj2[2];
  float angle;

  //prepare meca to take water
  _wait_meca_ready();
  ROME_SENDWAIT_MECA_CMD(&rome_meca,ROME_ENUM_MECA_COMMAND_PREPARE_LOAD_WATER);
  _wait_meca_ground_clear();

  set_speed(RS_FAST);

  switch(dispenser){
    case DISPENSER_NEAR:{
      ROME_LOG(&rome_paddock,DEBUG,"Going to start area dispenser");
      angle = arfast(ROBOT_SIDE_BALLEATER, TABLE_SIDE_MAIN);
      //send first position order
      or = goto_xya(KX(1200), near_pos + approach_side, angle);
      if (or!= ORDER_SUCCESS)
        return or;
      //prepare next move orders
      //dispenser near is alongside Y axis
      traj1[0] = KX(1500-approach_depth);
      traj1[1] = near_pos + approach_side;
      traj1[2] = KX(1500-(balleater_depth+10));
      traj1[3] = near_pos;

      traj2[0] = KX(1500-300);
      traj2[1] = near_pos;
      break;
    }
    case DISPENSER_FAR:{
      ROME_LOG(&rome_paddock,INFO,"Going to opposite dispenser");
      //send first position order
      //go to the bee corner to be near borders for autoset
      or = goto_pathfinding_node(PATHFINDING_GRAPH_NODE_WATER_DISPENSER_FAR, arfast(ROBOT_SIDE_BACK, TABLE_SIDE_DOWN));
      if (or!= ORDER_SUCCESS){
        ROME_LOG(&rome_paddock,INFO,"Aborting opposite dispenser");
        //go back near start area
        goto_pathfinding_node(PATHFINDING_GRAPH_NODE_WATER_TOWER, arfast(ROBOT_SIDE_BACK, TABLE_SIDE_DOWN));
        return or;
        }
      //we did a very long move, so try to launch an autoset
      or = goto_xya(KX(-1200), 150, arfast(ROBOT_SIDE_BACK, TABLE_SIDE_DOWN));
      if (or == ORDER_SUCCESS){
        autoset(ROBOT_SIDE_BACK,AUTOSET_DOWN, 0, AUTOSET_OFFSET);
        or = goto_xya(KX(-1300), 300, arfast(ROBOT_SIDE_BACK, TABLE_SIDE_DOWN));
        autoset(ROBOT_SIDE_BALLEATER,AUTOSET_AUX, KX(-1500+AUTOSET_OFFSET), 0);
        or = goto_xya(KX(-1200), 300, arfast(ROBOT_SIDE_BALLEATER, TABLE_SIDE_AUX));
      }
      //prepare next move orders
      //dispenser far is alongside X axis
      angle = arfast(ROBOT_SIDE_BALLEATER, TABLE_SIDE_DOWN);
      traj1[0] = KX(far_pos-approach_side);
      traj1[1] = approach_depth;
      traj1[2] = KX(far_pos);
      traj1[3] = balleater_depth-10;

      traj2[0] = KX(far_pos);
      traj2[1] = 500;
      break;
    }
    default:
      return ORDER_FAILURE;
  }

  set_speed(RS_SLOW);
  //go to dispenser
  or = goto_traj(traj1, angle);
  if (or!= ORDER_SUCCESS)
    return or;

  //take the water
  _wait_meca_ready();
  ROME_SENDWAIT_MECA_CMD(&rome_meca,ROME_ENUM_MECA_COMMAND_LOAD_WATER);
  _wait_meca_ground_clear();

  set_speed(RS_NORMAL);
  //just go back a little bit
  or = goto_traj(traj2, angle);

  //if the water in cylinder changed, we scored !
  if (nb_empty != robot_state.cylinder_nb_empty)
    update_score(10);

  return or;
}

order_result_t throw_water_watertower(void){
  ROME_LOG(&rome_paddock,INFO,"Throwing water in watertower");
  order_result_t or;
  set_speed(RS_FAST);

  _wait_meca_ready();
  ROME_SENDWAIT_MECA_CMD(&rome_meca,ROME_ENUM_MECA_COMMAND_PREPARE_THROW_WATERTOWER);
  _wait_meca_ground_clear();

  uint8_t balls_loaded = robot_state.cylinder_nb_good;
  or = goto_pathfinding_node(PATHFINDING_GRAPH_NODE_WATER_DISPENSER_NEAR,arfast(ROBOT_SIDE_TURBINE, TABLE_SIDE_MAIN));
  if (or != ORDER_SUCCESS)
    return or;
  or = goto_xya(KX(1100), 1450, compute_throw_angle(1100,1450));
  ROME_SENDWAIT_MECA_SET_THROW_POWER(&rome_meca,1950);
  _wait_meca_ready();
  ROME_SENDWAIT_MECA_CMD(&rome_meca,ROME_ENUM_MECA_COMMAND_THROW_WATERTOWER);
  _wait_meca_ground_clear();
  update_score(5*balls_loaded);

  return or;
}


order_result_t trash_water_treatment(void){
  ROME_LOG(&rome_paddock,INFO,"Trashing water in treatment area");
  order_result_t or;
  set_speed(RS_FAST);

  _wait_meca_ready();
  ROME_SENDWAIT_MECA_CMD(&rome_meca,ROME_ENUM_MECA_COMMAND_PREPARE_TRASH_TREATMENT);
  _wait_meca_ground_clear();

  //store nb balls for scoring purpose
  uint8_t balls_loaded = robot_state.cylinder_nb_bad;

  //push away the cubes in front of treatment area
  //the angle is changed to avoid pushing cubes towards recyling area
  float angle;
  if (robot_state.team == TEAM_GREEN)
    angle = arfast(ROBOT_SIDE_BACK,TABLE_SIDE_MAIN);
  else
    angle = arfast(ROBOT_SIDE_BALLEATER,TABLE_SIDE_MAIN);

  or = goto_pathfinding_node(PATHFINDING_GRAPH_NODE_MIDDLE_BOT, angle);
  if (or != ORDER_SUCCESS)
    return or;

  //go in position to trash the bad water
  or = goto_xya(KX(-250),250+130, arfast(ROBOT_SIDE_TURBINE,TABLE_SIDE_DOWN));
  if (or != ORDER_SUCCESS)
    return or;
  _wait_meca_ready();
  ROME_SENDWAIT_MECA_CMD(&rome_meca,ROME_ENUM_MECA_COMMAND_TRASH_TREATMENT);
  _wait_meca_ground_clear();
  update_score(10*balls_loaded);

  or = goto_xya(KX(-250), 400, arfast(ROBOT_SIDE_TURBINE, TABLE_SIDE_DOWN));
  or = goto_xya(KX(-250), 400, arfast(ROBOT_SIDE_BACK, TABLE_SIDE_DOWN));
  autoset(ROBOT_SIDE_BACK, AUTOSET_DOWN, 0, 250+AUTOSET_OFFSET);
  or = goto_xya(KX(-250), 550, arfast(ROBOT_SIDE_BACK, TABLE_SIDE_DOWN));

  return or;
}

order_result_t empty_cylinder(void){
  order_result_t or_throw_water = ORDER_FAILURE;
  order_result_t or_trash_water = ORDER_FAILURE;

  while(!cylinder_is_empty()){
    if (robot_state.cylinder_nb_bad !=0){
      or_trash_water = trash_water_treatment();
    }
    else
      if (robot_state.cylinder_nb_good !=0)
        or_throw_water = throw_water_watertower();
    //wait to be sure that cylinder counts are updated
    idle_delay_ms(200);
  }

  (void) or_throw_water;
  (void) or_trash_water;

  return ORDER_SUCCESS;
}

void strat_run(void)
{
  ROME_LOG(&rome_paddock,INFO,"Go !!!");
  ROME_SENDWAIT_ASSERV_GYRO_INTEGRATION(&rome_asserv, 1);
  ROME_SENDWAIT_ASSERV_ACTIVATE(&rome_asserv, 1);

  order_result_t or_take_water_near  = ORDER_FAILURE;
  order_result_t or_empty_water_near = ORDER_FAILURE;
  order_result_t or_take_water_far   = ORDER_FAILURE;
  order_result_t or_empty_water_far  = ORDER_FAILURE;

  bool force_far_dispenser_first = false;

  while (!( or_take_water_near  == ORDER_SUCCESS &&
            or_take_water_far   == ORDER_SUCCESS &&
            or_empty_water_near == ORDER_SUCCESS &&
            or_empty_water_far  == ORDER_SUCCESS )  ){

    if (force_far_dispenser_first &&
        or_take_water_far != ORDER_SUCCESS){

      or_take_water_far = take_water(DISPENSER_FAR);

      or_empty_water_far = empty_cylinder();
    }

    if (force_far_dispenser_first == true){
      order_result_t or = ORDER_FAILURE;
      or = goto_pathfinding_node(PATHFINDING_GRAPH_NODE_WATER_TOWER, arfast(ROBOT_SIDE_BACK,TABLE_SIDE_MAIN));
      if (or == ORDER_SUCCESS){
        goto_xya(KX(1350), 1650, arfast(ROBOT_SIDE_BACK,TABLE_SIDE_MAIN));
        autoset(ROBOT_SIDE_BACK,AUTOSET_MAIN, KX(1500-AUTOSET_OFFSET), 0);
        goto_xya(KX(1200), 1500, arfast(ROBOT_SIDE_BACK,TABLE_SIDE_MAIN));
      }
    }
    else
      force_far_dispenser_first = true;

    if (or_take_water_near != ORDER_SUCCESS){
      or_take_water_near = take_water(DISPENSER_NEAR);

      or_empty_water_near = empty_cylinder();
    }

    idle_delay_ms(200);
  }

  ROME_LOG(&rome_paddock,INFO,"That's all folks !");
  _delay_ms(3000);
  ROME_SENDWAIT_ASSERV_ACTIVATE(&rome_asserv, 0);
  ROME_SENDWAIT_MECA_SET_POWER(&rome_meca, 0);
}

void strat_test(void)
{
  ROME_LOG(&rome_paddock,INFO,"Strat test stuff");
  for(;;) {
    idle();
    if(!robot_state.gyro_calibration)
      break;
  }

  set_speed(RS_NORMAL);
  #if 0
  set_xya_wait(0,0,0);
  goto_xya(0,0,0);
  ROME_SENDWAIT_ASSERV_ACTIVATE(&rome_asserv, 1);
  ROME_SENDWAIT_ASSERV_GYRO_INTEGRATION(&rome_asserv, 1);

  ROME_LOG(&rome_paddock,INFO,"go");
  goto_xya(0,700,0);
  ROME_LOG(&rome_paddock,INFO,"back");
  goto_xya(0,0,0);

  #else
  ROME_SENDWAIT_ASSERV_ACTIVATE(&rome_asserv, 1);
  ROME_SENDWAIT_ASSERV_GYRO_INTEGRATION(&rome_asserv, 1);
  ROME_LOG(&rome_paddock,INFO,"go");
  goto_pathfinding_node(PATHFINDING_GRAPH_NODE_WATER_DISPENSER_FAR,
                        arfast(ROBOT_SIDE_BALLEATER,TABLE_SIDE_DOWN));


  idle_delay_ms(5000);
  ROME_LOG(&rome_paddock,INFO,"back");
  goto_pathfinding_node(PATHFINDING_GRAPH_NODE_ORANGE_WATER_TOWER,
                        arfast(ROBOT_SIDE_BACK,TABLE_SIDE_MAIN));

  ROME_LOG(&rome_paddock,INFO,"align to start a new test");
  goto_xya(KX(1300), 1500, arfast(ROBOT_SIDE_BACK,TABLE_SIDE_MAIN));
  #endif

  ROME_LOG(&rome_paddock,INFO,"Strat test stuff end ...");

  for(;;) {
    idle();
  }

}

