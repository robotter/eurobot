#define AUTOSET_OFFSET 112

#define ARM_POS_TOP  0
#define ARM_POS_BOTTOM 200
#define ARM_POS_BANNER 75
#define ARM_POS_LIFT_CANS 180
#define ARM_POS_LIFT_FLOOR_CANS 50
#define ARM_POS_FIST_FLOOR_CANS 70

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
  //we send orders only to right arm because there is no left arm for now

  //initalise kx factor
  robot_kx = TEAM_SIDE_VALUE(-1, 1);

  ROME_SENDWAIT_MECA_FOLD_WINGS(ROME_DST_MECA, false);
  ROME_SENDWAIT_MECA_MOVE_ELEVATOR(ROME_DST_MECA, false, ARM_POS_TOP);

  ROME_LOG(ROME_DST_PADDOCK, DEBUG,"Strat prepare");
  // initialize asserv
  ROME_SENDWAIT_ASSERV_ACTIVATE(ROME_DST_ASSERV, 1);
  set_xya_wait(0,0,0);
  ROME_SENDWAIT_ASSERV_GOTO_XY(ROME_DST_ASSERV, 0, 0, 0);
  ROME_SENDWAIT_ASSERV_SET_HTRAJ_XY_STEERING(ROME_DST_ASSERV, 1.5, 0.03);
  ROME_SENDWAIT_ASSERV_SET_HTRAJ_XY_CRUISE(ROME_DST_ASSERV, 15, 0.03);

  // autoset robot in color side bot corner
  // x in wall (elements must be removed)
  set_xya_wait(KX(0), 0, arfast(ROBOT_SIDE_BACK, TABLE_SIDE_MAIN));
  autoset(ROBOT_SIDE_BACK, AUTOSET_MAIN, KX(1500-AUTOSET_OFFSET), 0);
  goto_xya(KX(1230), 0, arfast(ROBOT_SIDE_BACK, TABLE_SIDE_MAIN));
  // y in wall in oponent construction area
  goto_xya(KX(1230), -150, arfast(ROBOT_SIDE_BACK, TABLE_SIDE_DOWN));
  autoset(ROBOT_SIDE_BACK, AUTOSET_DOWN, 0, AUTOSET_OFFSET);
  goto_xya(KX(1280), 300, arfast(ROBOT_SIDE_BACK, TABLE_SIDE_DOWN));

  ROME_SENDWAIT_MECA_FOLD_WINGS(ROME_DST_MECA, false);
  ROME_SENDWAIT_MECA_RELEASE_CANS(ROME_DST_MECA, false);
  ROME_SENDWAIT_MECA_MOVE_ELEVATOR(ROME_DST_MECA, false, ARM_POS_BANNER);

  // go to central starting area
  goto_pathfinding_node(PATHFINDING_GRAPH_NODE_MAIN_START, arfast(ROBOT_SIDE_RIGHT, TABLE_SIDE_DOWN));
  goto_xya(KX(290), 250, arfast(ROBOT_SIDE_RIGHT, TABLE_SIDE_DOWN));

  // shutdown elevators to preserve battery
  // or not, the banner falls on restart
  //ROME_SENDWAIT_MECA_SHUTDOWN_ELEVATOR(ROME_DST_MECA, true);
  //ROME_SENDWAIT_MECA_SHUTDOWN_ELEVATOR(ROME_DST_MECA, false);

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

order_result_t push_bot_cm(void){
  order_result_t or = ORDER_FAILURE;

  ROME_LOG(ROME_DST_PADDOCK, INFO, "Push bot construction material in construction area");
  // for now there is only an arm on the right side
  const float angle = arfast(ROBOT_SIDE_RIGHT, TABLE_SIDE_DOWN);
  or = goto_pathfinding_node(PATHFINDING_GRAPH_NODE_BOT_STOCK, angle);
  if (or != ORDER_SUCCESS)
    return or;

  ROME_SENDWAIT_MECA_DEPLOY_WINGS(ROME_DST_MECA, false);
  ROME_SENDWAIT_MECA_TAKE_CANS(ROME_DST_MECA, false);
  ROME_SENDWAIT_MECA_MOVE_ELEVATOR(ROME_DST_MECA, false, ARM_POS_BOTTOM);
  wait_meca_ready();
  goto_xya(KX(730), 400, arfast(ROBOT_SIDE_RIGHT, TABLE_SIDE_DOWN));
  //push construction material
  goto_xya(KX(730), 250, arfast(ROBOT_SIDE_RIGHT, TABLE_SIDE_DOWN));
  update_score(4);
  ROME_SENDWAIT_MECA_RELEASE_CANS(ROME_DST_MECA, false);
  wait_meca_ready();
  //clear new construction
  goto_xya(KX(730), 400, arfast(ROBOT_SIDE_RIGHT, TABLE_SIDE_DOWN));

  ROME_SENDWAIT_MECA_MOVE_ELEVATOR(ROME_DST_MECA, false, ARM_POS_TOP);
  ROME_SENDWAIT_MECA_FOLD_WINGS(ROME_DST_MECA, false);
  wait_meca_ground_clear();

  //go out of the corner to be ready for next order
  goto_pathfinding_node(PATHFINDING_GRAPH_NODE_BOT_STOCK, angle);
  return ORDER_SUCCESS;
}

order_result_t goto_stage_and_autoset(void){
  order_result_t or = ORDER_FAILURE;
  ROME_LOG(ROME_DST_PADDOCK, INFO, "Move near the stage and do an autoset on it");
  float angle = arfast(ROBOT_SIDE_BACK, TABLE_SIDE_UP);
  or = goto_pathfinding_node(PATHFINDING_GRAPH_NODE_STAGE, angle);
  if (or != ORDER_SUCCESS)
    return or;

  goto_xya(KX(0), 1450, arfast(ROBOT_SIDE_BACK, TABLE_SIDE_UP));
  //autoset Y on the stage
  autoset(ROBOT_SIDE_BACK, AUTOSET_UP, 0, 1550-AUTOSET_OFFSET);
  goto_xya(KX(0), 1250, arfast(ROBOT_SIDE_BACK, TABLE_SIDE_UP));

  return ORDER_SUCCESS;
}

order_result_t push_main_central_cm(bool first){
  order_result_t or = ORDER_FAILURE;

  ROME_LOG(ROME_DST_PADDOCK, INFO, "Push main central construction material in start area");

  // for now there is only an arm on the right side
  float angle = arfast(ROBOT_SIDE_RIGHT, TABLE_SIDE_DOWN);
  or = goto_pathfinding_node(PATHFINDING_GRAPH_NODE_MAIN_CENTRAL_STOCK_TOP, angle);
  if (or != ORDER_SUCCESS)
    return or;
  //push construction material
  ROME_SENDWAIT_MECA_DEPLOY_WINGS(ROME_DST_MECA, false);
  ROME_SENDWAIT_MECA_TAKE_CANS(ROME_DST_MECA, false);
  ROME_SENDWAIT_MECA_MOVE_ELEVATOR(ROME_DST_MECA, false, ARM_POS_BOTTOM);
  wait_meca_ready();

  int16_t traj1[4];
      traj1[0] = KX(430);
      traj1[1] = 930;
      traj1[2] = KX(310);
      traj1[3] = 550;
  goto_traj(traj1, arfast(ROBOT_SIDE_RIGHT, TABLE_SIDE_DOWN));

  if(first)
    goto_xya(KX(280), 250, arfast(ROBOT_SIDE_RIGHT, TABLE_SIDE_DOWN));
  else
    goto_xya(KX(280), 330, arfast(ROBOT_SIDE_RIGHT, TABLE_SIDE_DOWN));
  ROME_SENDWAIT_MECA_RELEASE_CANS(ROME_DST_MECA, false);
  update_score(4);
  //clear new construction
  goto_xya(KX(280), 550, arfast(ROBOT_SIDE_RIGHT, TABLE_SIDE_DOWN));

  //go out of the start area to be ready for next order
  ROME_SENDWAIT_MECA_MOVE_ELEVATOR(ROME_DST_MECA, false, ARM_POS_TOP);
  ROME_SENDWAIT_MECA_MOVE_ELEVATOR(ROME_DST_MECA, false, ARM_POS_TOP);
  ROME_SENDWAIT_MECA_FOLD_WINGS(ROME_DST_MECA, false);
  wait_meca_ground_clear();
  goto_pathfinding_node(PATHFINDING_GRAPH_NODE_MAIN_START, angle);
  return ORDER_SUCCESS;
}

order_result_t push_aux_central_cm(bool first){
  order_result_t or = ORDER_FAILURE;

  ROME_LOG(ROME_DST_PADDOCK, INFO, "Push aux central construction material in start area");

  // for now there is only an arm on the right side
  float angle = arfast(ROBOT_SIDE_RIGHT, TABLE_SIDE_DOWN);
  or = goto_pathfinding_node(PATHFINDING_GRAPH_NODE_AUX_CENTRAL_STOCK_TOP, angle);
  if (or != ORDER_SUCCESS)
    return or;
  //push construction material
  ROME_SENDWAIT_MECA_DEPLOY_WINGS(ROME_DST_MECA, false);
  ROME_SENDWAIT_MECA_TAKE_CANS(ROME_DST_MECA, false);
  ROME_SENDWAIT_MECA_MOVE_ELEVATOR(ROME_DST_MECA, false, ARM_POS_BOTTOM);
  wait_meca_ready();
  goto_xya(KX(-430), 930, arfast(ROBOT_SIDE_RIGHT, TABLE_SIDE_DOWN));
  set_speed(RS_VERYSLOW);
  goto_xya_synced(KX(-500), 700, arfast(TEAM_SIDE_VALUE(ROBOT_SIDE_LEFT,ROBOT_SIDE_BACK), TABLE_SIDE_AUX));
  goto_xya_synced(0,        725, arfast(ROBOT_SIDE_RIGHT, TABLE_SIDE_MAIN));
  goto_xya_synced(KX(100),  650, arfast(TEAM_SIDE_VALUE(ROBOT_SIDE_LEFT,ROBOT_SIDE_BACK), TABLE_SIDE_AUX));
  goto_xya_synced(KX(310), 550, arfast(ROBOT_SIDE_RIGHT, TABLE_SIDE_DOWN));
  set_speed(RS_NORMAL);

  if(first)
    goto_xya(KX(280), 250, arfast(ROBOT_SIDE_RIGHT, TABLE_SIDE_DOWN));
  else
    goto_xya(KX(280), 330, arfast(ROBOT_SIDE_RIGHT, TABLE_SIDE_DOWN));
  ROME_SENDWAIT_MECA_RELEASE_CANS(ROME_DST_MECA, false);
  update_score(4);
  //clear new construction
  goto_xya(KX(280), 550, arfast(ROBOT_SIDE_RIGHT, TABLE_SIDE_DOWN));

  //go out of the start area to be ready for next order
  ROME_SENDWAIT_MECA_MOVE_ELEVATOR(ROME_DST_MECA, false, ARM_POS_TOP);
  ROME_SENDWAIT_MECA_FOLD_WINGS(ROME_DST_MECA, false);
  wait_meca_ground_clear();
  goto_pathfinding_node(PATHFINDING_GRAPH_NODE_MAIN_START, angle);
  return ORDER_SUCCESS;
}

void strat_run(void)
{
  ROME_LOG(ROME_DST_PADDOCK, INFO, "Go !!!");
  ROME_SENDWAIT_ASSERV_GYRO_INTEGRATION(ROME_DST_ASSERV, 1);
  ROME_SENDWAIT_ASSERV_ACTIVATE(ROME_DST_ASSERV, 1);
  ROME_SENDWAIT_MECA_MOVE_ELEVATOR(ROME_DST_MECA, false, ARM_POS_BANNER);
  wait_meca_ground_clear();

  order_result_t or_push_bot_cm           = ORDER_FAILURE;
  order_result_t or_push_main_central_cm  = ORDER_FAILURE;
  order_result_t or_push_aux_central_cm   = ORDER_FAILURE;

  //place banner
  goto_xya(KX(290), 100, arfast(ROBOT_SIDE_RIGHT, TABLE_SIDE_DOWN));

  //move in fornt of starting area
  goto_xya(KX(280), 550, arfast(ROBOT_SIDE_RIGHT, TABLE_SIDE_DOWN));


  while (!( or_push_bot_cm          == ORDER_SUCCESS &&
            or_push_main_central_cm == ORDER_SUCCESS &&
            or_push_aux_central_cm  == ORDER_SUCCESS )  ){
    idle();

    //near end of match, just stop everything and prepare to go to arrival area
    if (robot_state.match_time > 80) {
      ROME_LOG(ROME_DST_PADDOCK, INFO, "Out of time");
      break;
    }

    //first task, try to push bot construction material. this shouldn't fail as is it near the start area.
    if (or_push_bot_cm != ORDER_SUCCESS)
      or_push_bot_cm = push_bot_cm();

    //make and autoset on the stage
    goto_stage_and_autoset();

    //try to push the central construction material of our color in start area
    if (or_push_main_central_cm != ORDER_SUCCESS)
      or_push_main_central_cm = push_main_central_cm(or_push_aux_central_cm != ORDER_SUCCESS);

    //do again an autoset on the stage, it will also be an evading maneuver if oponenet prevented the first try on center
    goto_stage_and_autoset();

    //try to push the central construction material of other color in start area
    if (or_push_aux_central_cm != ORDER_SUCCESS)
      or_push_aux_central_cm = push_aux_central_cm(or_push_main_central_cm != ORDER_SUCCESS);

  }

  #if 1
  ROME_LOG(ROME_DST_PADDOCK, INFO, "Job done, go to backstage");
  ROME_SENDWAIT_MECA_FOLD_WINGS(ROME_DST_MECA, false);
  ROME_SENDWAIT_MECA_RELEASE_CANS(ROME_DST_MECA, false);
  ROME_SENDWAIT_MECA_MOVE_ELEVATOR(ROME_DST_MECA, false, ARM_POS_TOP);
  goto_pathfinding_node(PATHFINDING_GRAPH_NODE_BACKSTAGE, arfast(ROBOT_SIDE_BACK, TABLE_SIDE_UP));
  ROME_SENDWAIT_MECA_TAKE_CANS(ROME_DST_MECA, false);

  //wait for the superstar and the groupies to clear the area
  while (robot_state.match_time < 95){
    idle();
    goto_xya_synced(KX(1150),1320,arfast(ROBOT_SIDE_MAIN, TABLE_SIDE_UP));
    ROME_SENDWAIT_MECA_FOLD_WINGS(ROME_DST_MECA, false);
    ROME_SENDWAIT_MECA_RELEASE_CANS(ROME_DST_MECA, false);
    goto_xya_synced(KX(1250),1320,arfast(ROBOT_SIDE_AUX, TABLE_SIDE_DOWN));
    ROME_SENDWAIT_MECA_DEPLOY_WINGS(ROME_DST_MECA, false);
    ROME_SENDWAIT_MECA_TAKE_CANS(ROME_DST_MECA, false);
  }
  set_speed(RS_FAST);
  ROME_SENDWAIT_MECA_FOLD_WINGS(ROME_DST_MECA, false);
  ROME_SENDWAIT_MECA_MOVE_ELEVATOR(ROME_DST_MECA, false, ARM_POS_BOTTOM);
  goto_xya(KX(1250), 1410, arfast(ROBOT_SIDE_BACK, TABLE_SIDE_MAIN));
  update_score(10);

  #else
  ROME_LOG(ROME_DST_PADDOCK, INFO, "Go back for reprogramming");
  or = goto_pathfinding_node(PATHFINDING_GRAPH_NODE_SIDE_STOCK_BOT, arfast(ROBOT_SIDE_BACK,TABLE_SIDE_MAIN));
  goto_xya(1200, 400, arfast(ROBOT_SIDE_BACK, TABLE_SIDE_MAIN));
  #endif

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

