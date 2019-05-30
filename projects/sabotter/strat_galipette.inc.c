
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

  // autoset on both sides
  set_xya_wait(KX(0), 0, arfast(ROBOT_SIDE_BACK, TABLE_SIDE_MAIN));
  galipette_autoset(ROBOT_SIDE_BACK, AUTOSET_MAIN, KX(1500-BUMPER_TO_CENTER_DIST), 0);
  goto_xya(KX(1300), 0, arfast(ROBOT_SIDE_BACK, TABLE_SIDE_MAIN));
  goto_xya(KX(1300), 0, arfast(ROBOT_SIDE_BACK, TABLE_SIDE_UP));
  galipette_autoset(ROBOT_SIDE_BACK, AUTOSET_UP, 0, 2000-BUMPER_TO_CENTER_DIST);
  // move to the starting zone
  goto_xya(KX(1220), 1550, arfast(ROBOT_SIDE_BACK, TABLE_SIDE_UP));

  // experience is on the table
  update_score(SCORE_EXPERIENCE_INSTALLED);

  ROME_SENDWAIT_ASSERV_ACTIVATE(ROME_DST_ASSERV, 0);
  ROME_SENDWAIT_ASSERV_GYRO_INTEGRATION(ROME_DST_ASSERV, 0);
}


#if 0 // 2018
#define BOOMOTTER_MAX_AGE_US 3e6

bool boomotter_connected(void){
  return (robot_state.boom_age + BOOMOTTER_MAX_AGE_US > uptime_us());
}

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

#endif

void strat_run(void)
{
  ROME_SENDWAIT_ASSERV_GYRO_INTEGRATION(ROME_DST_ASSERV, 1);
  ROME_SENDWAIT_ASSERV_ACTIVATE(ROME_DST_ASSERV, 1);

  ROME_LOG(ROME_DST_PADDOCK, DEBUG, "Gooooooo !!!!");

  order_result_t or = ORDER_FAILURE;

  // Go to the blue atom to accelerate
  ROME_LOG(ROME_DST_PADDOCK, DEBUG, "Push blue atom");
  {
    // If blocked on the way, move toward the center of the table to let the
    // opponent pass, then try go to the atom again.
    for(;;) {
      or = goto_pathfinding_node(PATHFINDING_GRAPH_NODE_ACCELERATED_BLUE, arfast(ROBOT_SIDE_BACK, TABLE_SIDE_UP));
      if(or == ORDER_SUCCESS) {
        break;
      }
      ROME_LOG(ROME_DST_PADDOCK, INFO, "Path to blue atom blocked, let opponent pass");
      goto_xya(robot_state.current_pos.x, 1000, arfast(ROBOT_SIDE_BACK, TABLE_SIDE_UP));
    }

    goto_xya(KX(-100), 1840, arfast(ROBOT_SIDE_BACK, TABLE_SIDE_UP));
    galipette_autoset(ROBOT_SIDE_BACK, AUTOSET_UP, 0, 2000-BUMPER_TO_CENTER_DIST-30);
    goto_xya(KX(-100), 1840, arfast(ROBOT_SIDE_BACK, TABLE_SIDE_UP));

    // Stick to the table, push the atom, move back (along the accelerator)
    goto_xya(KX(-100), 1840, arfast(ROBOT_SIDE_LEFT, TABLE_SIDE_UP));
    goto_xya(KX(-240), 1840, arfast(ROBOT_SIDE_LEFT, TABLE_SIDE_UP));
    goto_xya(KX(-150), 1700, arfast(ROBOT_SIDE_LEFT, TABLE_SIDE_UP));
    //TODO check action success
    update_score(SCORE_ATOM_IN_ACCELERATOR);
    update_score(SCORE_GOLDENIUM_FREED);

    // Autoset on the border
    goto_xya(KX(-100), 1800, arfast(ROBOT_SIDE_BACK, TABLE_SIDE_UP));
    goto_xya(KX(-100), 1860, arfast(ROBOT_SIDE_BACK, TABLE_SIDE_UP));
    galipette_autoset(ROBOT_SIDE_BACK, AUTOSET_UP, 0, 2000-BUMPER_TO_CENTER_DIST-30);
    goto_xya(KX(-100), 1700, arfast(ROBOT_SIDE_BACK, TABLE_SIDE_UP));
  }

  ROME_LOG(ROME_DST_PADDOCK, DEBUG, "Take goldenium");
  {
    // Go to the goldenium
    or = goto_pathfinding_node(PATHFINDING_GRAPH_NODE_GOLDENIUM, arfast(ROBOT_SIDE_BACK, TABLE_SIDE_UP));
    // Switch on the pump, stick to the goldenium, then move back
    pump_on();
    goto_xya(KX(-760), (1880), arfast(ROBOT_SIDE_BACK, TABLE_SIDE_UP));
    idle_delay_ms(500);
    goto_xya(KX(-760), 1800, arfast(ROBOT_SIDE_BACK, TABLE_SIDE_UP));
    //TODO check action success
    update_score(SCORE_GOLDENIUM_EXTRACTED);
  }

  ROME_LOG(ROME_DST_PADDOCK, DEBUG, "Put goldenium in the balance");
  {
    // Go to the balance
    or = goto_pathfinding_node(PATHFINDING_GRAPH_NODE_BALANCE, arfast(ROBOT_SIDE_BACK, TABLE_SIDE_DOWN));
    // Stick to the balance, drop the goldenium, move back
    goto_xya(KX(140), 520, arfast(ROBOT_SIDE_BACK, TABLE_SIDE_DOWN));
    pump_off();
    idle_delay_ms(2000);
    goto_xya(KX(200), 700, arfast(ROBOT_SIDE_BACK, TABLE_SIDE_DOWN));
    //TODO check action success
    update_score(SCORE_GOLDENIUM_IN_BALANCE);
  }

  ROME_LOG(ROME_DST_PADDOCK, INFO, "That's all folks !");
  ROME_SENDWAIT_ASSERV_ACTIVATE(ROME_DST_ASSERV, 0);
}

void strat_test(void)
{
  ROME_LOG(ROME_DST_PADDOCK, INFO, "Strat test stuff");
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
