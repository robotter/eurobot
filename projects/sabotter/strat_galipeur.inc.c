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
  goto_xya(KX(1230), 0, arfast(ROBOT_SIDE_BACK, TABLE_SIDE_MAIN));
  // y on small dispenser
  goto_xya(KX(1230), 0, arfast(ROBOT_SIDE_BACK, TABLE_SIDE_DOWN));
  autoset(ROBOT_SIDE_BACK, AUTOSET_DOWN, 0, AUTOSET_OFFSET);
  goto_xya(KX(1280), 300, arfast(ROBOT_SIDE_BACK, TABLE_SIDE_DOWN));

  // go to green starting areau
  goto_xya(KX(1365), 1250, arfast(ROBOT_SIDE_BACK, TABLE_SIDE_MAIN));

  // shutdown elevators to preserve battery
  ROME_SENDWAIT_MECA_SHUTDOWN_ELEVATOR(ROME_DST_MECA, true);
  ROME_SENDWAIT_MECA_SHUTDOWN_ELEVATOR(ROME_DST_MECA, false);

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

  ROME_SENDWAIT_MECA_MOVE_ELEVATOR(ROME_DST_MECA, true, ARM_POS_BOTTOM);
  ROME_SENDWAIT_MECA_MOVE_ELEVATOR(ROME_DST_MECA, false, ARM_POS_BOTTOM);

  order_result_t or = ORDER_FAILURE;
  (void)or;

  goto_xya(KX(1250), 1000, arfast(ROBOT_SIDE_BACK, TABLE_SIDE_MAIN));
  const int16_t arm_x_offset = TEAM_SIDE_VALUE(0, 30);

  // Go to the large dispenser, nearest group of 3 atoms
  ROME_LOG(ROME_DST_PADDOCK, INFO, "Go to large dispenser (near side)");
  {
    const float angle = arfast(ROBOT_SIDE_MAIN, TABLE_SIDE_DOWN);
    const bool arm_side = TEAM_SIDE_VALUE(false, true);

    or = goto_pathfinding_node(PATHFINDING_GRAPH_NODE_LARGE_DISPENSER_NEAR, angle);
    // stick to the table
    wait_meca_ready();
    goto_xya(KX(900) + arm_x_offset, 600, angle);
    goto_xya(KX(900) + arm_x_offset, 540, angle);
    // take the atoms
    ROME_LOG(ROME_DST_PADDOCK, INFO, "Take atoms");
    ROME_SENDWAIT_MECA_TAKE_ATOMS(ROME_DST_MECA, arm_side);
    wait_meca_ground_clear();
    // move the arm then go back
    ROME_SENDWAIT_MECA_MOVE_ELEVATOR(ROME_DST_MECA, arm_side, ARM_POS_TOP);
    goto_xya(KX(900) + arm_x_offset, 650, angle);
  }

  // Go to the large dispenser, next group of 3 atoms
  ROME_LOG(ROME_DST_PADDOCK, INFO, "Get the second group of atoms");
  {
    const float angle = arfast(ROBOT_SIDE_AUX, TABLE_SIDE_DOWN);
    const bool arm_side = TEAM_SIDE_VALUE(true, false);

    or = goto_pathfinding_node(PATHFINDING_GRAPH_NODE_LARGE_DISPENSER_FAR, angle);
    // stick to the table
    wait_meca_ready();
    goto_xya(KX(600) + arm_x_offset, 600, angle);
    goto_xya(KX(600) + arm_x_offset, 520, angle);
    // take the atoms
    ROME_LOG(ROME_DST_PADDOCK, INFO, "Take atoms");
    ROME_SENDWAIT_MECA_TAKE_ATOMS(ROME_DST_MECA, arm_side);
    wait_meca_ground_clear();
    // move the arm then go back
    ROME_SENDWAIT_MECA_MOVE_ELEVATOR(ROME_DST_MECA, arm_side, ARM_POS_TOP);
    goto_xya(KX(600) + arm_x_offset, 700, angle);
  }

  // after taking atoms, do an autoset
  // Y on atom dispenser
  goto_xya(KX(800), 700, arfast(ROBOT_SIDE_BACK, TABLE_SIDE_DOWN));
  goto_xya(KX(800), 600, arfast(ROBOT_SIDE_BACK, TABLE_SIDE_DOWN));
  autoset(ROBOT_SIDE_BACK, AUTOSET_DOWN, 0, 450+AUTOSET_OFFSET);
  goto_xya(KX(800), 650, arfast(ROBOT_SIDE_BACK, TABLE_SIDE_DOWN));

  // X on terrain border
  goto_xya(KX(1300), 700, arfast(ROBOT_SIDE_BACK, TABLE_SIDE_MAIN));
  autoset(ROBOT_SIDE_BACK, AUTOSET_MAIN, KX(1500-AUTOSET_OFFSET), 0);
  goto_xya(KX(1200), 800, arfast(ROBOT_SIDE_BACK, TABLE_SIDE_MAIN));

  // drop the atoms in start area
  // try to go to start area, go back near dispensers if it failed, the oponent can be on it's goldenium
  do {
    or = goto_xya(KX(1250), 1350, arfast(TEAM_SIDE_VALUE(ROBOT_SIDE_RIGHT,ROBOT_SIDE_BACK), TABLE_SIDE_AUX));
    if (or != ORDER_SUCCESS)
      goto_xya(KX(1000), 800, arfast(ROBOT_SIDE_LEFT, TABLE_SIDE_UP));
  }
  while(or != ORDER_SUCCESS);

  ROME_SENDWAIT_MECA_MOVE_ELEVATOR(ROME_DST_MECA, true, ARM_POS_BOTTOM);
  wait_meca_ready();
  ROME_SENDWAIT_MECA_RELEASE_ATOMS(ROME_DST_MECA, true);

  ROME_SENDWAIT_MECA_MOVE_ELEVATOR(ROME_DST_MECA, false, ARM_POS_BOTTOM);
  wait_meca_ready();
  goto_xya(KX(1250), 1100, arfast(TEAM_SIDE_VALUE(ROBOT_SIDE_RIGHT,ROBOT_SIDE_BACK), TABLE_SIDE_AUX));
  goto_xya(KX(1250), 1300, arfast(ROBOT_SIDE_BACK, TEAM_SIDE_VALUE(TABLE_SIDE_AUX,TABLE_SIDE_DOWN)));
  ROME_SENDWAIT_MECA_RELEASE_ATOMS(ROME_DST_MECA, false);
  wait_meca_ready();

  // we may have scored 3 red atoms at the right spot and 3 others
  // TODO : use meca informations to be better than that
  update_score(SCORE_ATOM_IN_START_AREA * 6);
  update_score(SCORE_ATOM_IN_START_AREA_COLOR_BONUS * 3);

  goto_xya(KX(1250), 1000, arfast(ROBOT_SIDE_LEFT, TABLE_SIDE_UP));

  ROME_LOG(ROME_DST_PADDOCK, INFO, "That's all folks !");
  ROME_SENDWAIT_MECA_SHUTDOWN_ELEVATOR(ROME_DST_MECA, true);
  ROME_SENDWAIT_MECA_SHUTDOWN_ELEVATOR(ROME_DST_MECA, false);
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

