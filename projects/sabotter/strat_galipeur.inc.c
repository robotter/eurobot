#define AUTOSET_OFFSET 112

#define ARM_POS_TOP  0
#define ARM_POS_BOTTOM  165


void wait_meca_ready(void){
  //force meca state busy
  robot_state.meca_state = ROME_ENUM_MECA_STATE_BUSY;
  ROME_LOG(ROME_DST_PADDOCK, DEBUG, "strat : wait meca ready");
  for (;;){
    idle();
    if((robot_state.meca_state == ROME_ENUM_MECA_STATE_READY)){
      ROME_LOG(ROME_DST_PADDOCK, DEBUG, "strat : meca ready");
      return;
    }
  }
}

void wait_meca_ground_clear(void){
  //force meca state busy
  robot_state.meca_state = ROME_ENUM_MECA_STATE_BUSY;
  ROME_LOG(ROME_DST_PADDOCK, DEBUG, "strat : wait meca ground clear");
  for (;;){
    idle();
    if((robot_state.meca_state == ROME_ENUM_MECA_STATE_GROUND_CLEAR ||
      robot_state.meca_state == ROME_ENUM_MECA_STATE_READY)){
      ROME_LOG(ROME_DST_PADDOCK, DEBUG, "strat : meca ground clear");
      return;
    }
  }
}

void strat_init(void)
{
  ROME_LOG(ROME_DST_PADDOCK, INFO, "Galipeur : Strat init");
  // disable asserv
  ROME_SENDWAIT_ASSERV_ACTIVATE(ROME_DST_ASSERV, 0);
  ROME_SENDWAIT_ASSERV_SET_HTRAJ_XY_STEERING(ROME_DST_ASSERV, 2.5, 0.1);
  ROME_SENDWAIT_ASSERV_SET_HTRAJ_XY_CRUISE(ROME_DST_ASSERV, 20, 0.1);

  // initialize meca
  ROME_LOG(ROME_DST_PADDOCK, INFO,"Init meca");
  ROME_SENDWAIT_MECA_SET_POWER(ROME_DST_MECA, 1);
  // set R3D2 parameters
  ROME_SENDWAIT_R3D2_SET_ROTATION(ROME_DST_ASSERV,0,25);

  for(;;) {
    update_rome_interfaces();
    if(!robot_state.gyro_calibration)
      break;
  }

  // set R3D2 parameters
  ROME_SENDWAIT_R3D2_SET_ROTATION(ROME_DST_ASSERV,200,25);
}

void strat_prepare(void)
{
  //initalise kx factor
  robot_kx = TEAM_SIDE_VALUE(-1, 1);

  ROME_LOG(ROME_DST_PADDOCK, DEBUG,"Strat prepare");
  // initialize asserv
  ROME_SENDWAIT_ASSERV_ACTIVATE(ROME_DST_ASSERV, 1);
  set_xya_wait(0,0,0);
  ROME_SENDWAIT_ASSERV_GOTO_XY(ROME_DST_ASSERV, 0, 0, 0);
  ROME_SENDWAIT_ASSERV_SET_HTRAJ_XY_STEERING(ROME_DST_ASSERV, 1.5, 0.03);
  ROME_SENDWAIT_ASSERV_SET_HTRAJ_XY_CRUISE(ROME_DST_ASSERV, 15, 0.03);

  // autoset robot
  // x in starting area
  set_xya_wait(KX(0), 0, arfast(ROBOT_SIDE_BACK, TABLE_SIDE_MAIN));
  autoset(ROBOT_SIDE_BACK, AUTOSET_MAIN, KX(1500-AUTOSET_OFFSET), 0);
  goto_xya(KX(1250), 0, arfast(ROBOT_SIDE_BACK, TABLE_SIDE_MAIN));
  // y on small dispenser
  goto_xya(KX(1250), 0, arfast(ROBOT_SIDE_BACK, TABLE_SIDE_DOWN));
  autoset(ROBOT_SIDE_BACK, AUTOSET_DOWN, 0, AUTOSET_OFFSET);

  // go to green starting areau
  goto_xya(KX(1300), 1250, arfast(ROBOT_SIDE_BACK, TABLE_SIDE_MAIN));

  // move both arms down
  ROME_SENDWAIT_MECA_MOVE_ELEVATOR(ROME_DST_MECA, true, ARM_POS_BOTTOM);
  ROME_SENDWAIT_MECA_MOVE_ELEVATOR(ROME_DST_MECA, false, ARM_POS_BOTTOM);

  // wait for meca to end all orders before shutting down asserv
  wait_meca_ready();

  ROME_SENDWAIT_ASSERV_ACTIVATE(ROME_DST_ASSERV, 0);
  ROME_SENDWAIT_ASSERV_GYRO_INTEGRATION(ROME_DST_ASSERV, 0);
}


void set_speed(robot_speed_t s){
  switch (s){
    case RS_VERYSLOW:
      ROME_SENDWAIT_ASSERV_SET_HTRAJ_XY_STEERING(ROME_DST_ASSERV, 1.5, 0.03);
      ROME_SENDWAIT_ASSERV_SET_HTRAJ_XY_CRUISE(ROME_DST_ASSERV, 10, 0.03);
      break;
    case RS_SLOW:
      ROME_SENDWAIT_ASSERV_SET_HTRAJ_XY_STEERING(ROME_DST_ASSERV, 1.5, 0.03);
      ROME_SENDWAIT_ASSERV_SET_HTRAJ_XY_CRUISE(ROME_DST_ASSERV, 15, 0.03);
      break;
    case RS_NORMAL:
      ROME_SENDWAIT_ASSERV_SET_HTRAJ_XY_STEERING(ROME_DST_ASSERV, 2.5, 0.1);
      ROME_SENDWAIT_ASSERV_SET_HTRAJ_XY_CRUISE(ROME_DST_ASSERV, 20, 0.1);
      break;
    case RS_FAST:
      ROME_SENDWAIT_ASSERV_SET_HTRAJ_XY_STEERING(ROME_DST_ASSERV, 5, 0.1);
      ROME_SENDWAIT_ASSERV_SET_HTRAJ_XY_CRUISE(ROME_DST_ASSERV, 60, 0.05);
      break;
    default:
      break;
  }
}

void strat_run(void)
{
  ROME_LOG(ROME_DST_PADDOCK, INFO, "Go !!!");
  ROME_SENDWAIT_ASSERV_GYRO_INTEGRATION(ROME_DST_ASSERV, 1);
  ROME_SENDWAIT_ASSERV_ACTIVATE(ROME_DST_ASSERV, 1);

  order_result_t or = ORDER_FAILURE;
  (void)or;

  // Go to the large dispenser, nearest group of 3 atoms
  ROME_LOG(ROME_DST_PADDOCK, INFO, "Go to large dispenser (near side)");
  {
    const float angle = arfast(ROBOT_SIDE_MAIN, TABLE_SIDE_DOWN);
    const bool arm_side = TEAM_SIDE_VALUE(true, false);
    const int16_t arm_x_offset = TEAM_SIDE_VALUE(0, 0);  //TODO

    or = goto_pathfinding_node(PATHFINDING_GRAPH_NODE_LARGE_DISPENSER_NEAR, angle);
    // stick to the table
    wait_meca_ready();
    goto_xya(KX(600) + arm_x_offset, 500, angle);
    // take the atoms
    ROME_LOG(ROME_DST_PADDOCK, INFO, "Take atoms");
    ROME_SENDWAIT_MECA_TAKE_ATOMS(ROME_DST_MECA, arm_side);
    // move the arm then go back
    ROME_SENDWAIT_MECA_MOVE_ELEVATOR(ROME_DST_MECA, arm_side, ARM_POS_TOP);
    goto_xya(KX(600) + arm_x_offset, 650, angle);
  }

  // Go to the large dispenser, next group of 3 atoms
  ROME_LOG(ROME_DST_PADDOCK, INFO, "Get the second group of atoms");
  {
    const float angle = arfast(ROBOT_SIDE_AUX, TABLE_SIDE_DOWN);
    const bool arm_side = TEAM_SIDE_VALUE(false, true);
    const int16_t arm_x_offset = TEAM_SIDE_VALUE(0, 0);  //TODO

    or = goto_pathfinding_node(PATHFINDING_GRAPH_NODE_LARGE_DISPENSER_FAR, angle);
    // stick to the table
    goto_xya(KX(900) + arm_x_offset, 500, angle);
    // take the atoms
    ROME_LOG(ROME_DST_PADDOCK, INFO, "Take atoms");
    ROME_SENDWAIT_MECA_TAKE_ATOMS(ROME_DST_MECA, arm_side);
    // move the arm then go back
    ROME_SENDWAIT_MECA_MOVE_ELEVATOR(ROME_DST_MECA, arm_side, ARM_POS_TOP);
    goto_xya(KX(900) + arm_x_offset, 650, angle);
  }

  ROME_LOG(ROME_DST_PADDOCK, INFO, "That's all folks !");
  idle_delay_ms(3000);
  ROME_SENDWAIT_ASSERV_ACTIVATE(ROME_DST_ASSERV, 0);
  ROME_SENDWAIT_MECA_SET_POWER(ROME_DST_MECA, 0);
}

void strat_test(void)
{
  ROME_LOG(ROME_DST_PADDOCK, INFO, "Strat test stuff");
  while(robot_state.gyro_calibration) {
    idle();
  }

  set_speed(RS_NORMAL);
  ROME_SENDWAIT_ASSERV_ACTIVATE(ROME_DST_ASSERV, 1);
  ROME_SENDWAIT_ASSERV_GYRO_INTEGRATION(ROME_DST_ASSERV, 1);
  ROME_LOG(ROME_DST_PADDOCK, INFO, "go");

  ROME_LOG(ROME_DST_PADDOCK, INFO, "Strat test stuff end ...");

  for(;;) {
    idle();
  }
}

