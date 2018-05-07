#define ROBOT_SIDE_CUBE_CLAW (ROBOT_SIDE_LEFT)

typedef enum{
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
  ROME_SENDWAIT_R3D2_SET_ROTATION(&rome_asserv,0,25);
  for(;;) {
    idle();
    if(!robot_state.gyro_calibration)
      break;
  }

  // set R3D2 parameters
  ROME_SENDWAIT_R3D2_SET_ROTATION(&rome_asserv,350,25);

}

void strat_prepare(void)
{
  set_speed(RS_NORMAL);
  //initalise kx factor
  robot_kx = robot_state.team == TEAM_GREEN ? -1 : 1;

  ROME_LOG(&rome_paddock,DEBUG,"Strat prepare");

  ROME_SENDWAIT_ASSERV_ACTIVATE(&rome_asserv, 1);

  set_xya_wait(KX(0), 0, arfast(ROBOT_SIDE_BACK,TABLE_SIDE_MAIN));
  autoset(ROBOT_SIDE_BACK,AUTOSET_MAIN, KX(1500-BUMPER_TO_CENTER_DIST), 0);
  goto_xya(KX(1200), 0, arfast(ROBOT_SIDE_BACK,TABLE_SIDE_MAIN));
  goto_xya(KX(1200), 0, arfast(ROBOT_SIDE_BACK,TABLE_SIDE_UP));
  autoset(ROBOT_SIDE_BACK,AUTOSET_UP, 0, 2000-BUMPER_TO_CENTER_DIST);
  goto_xya(KX(1200), 1800, arfast(ROBOT_SIDE_BACK,TABLE_SIDE_UP));
  goto_xya(KX(1350), 1800, arfast(ROBOT_SIDE_CUBE_CLAW,TABLE_SIDE_AUX));

  ROME_SENDWAIT_ASSERV_SET_SERVO(&rome_asserv, CUBE_CLAW_ELEVATOR, CUBE_CLAW_ELEVATOR_DOWN);
  cube_claw_open();

  ROME_SENDWAIT_ASSERV_ACTIVATE(&rome_asserv, 0);
  ROME_SENDWAIT_ASSERV_GYRO_INTEGRATION(&rome_asserv, 0);
}

void strat_run(void)
{
  ROME_SENDWAIT_ASSERV_GYRO_INTEGRATION(&rome_asserv, 1);
  ROME_SENDWAIT_ASSERV_ACTIVATE(&rome_asserv, 1);
  cube_claw_close();

  //wait for galipeur departure
  idle_delay_ms(1000);

  //go switching domotic panel first
  ROME_SENDWAIT_ASSERV_SET_SERVO(&rome_asserv, CUBE_CLAW_ELEVATOR, CUBE_CLAW_ELEVATOR_BUTTON_WITHCUBE);

  int16_t traj[] = {
      KX(1000), 1700,
      KX(370), 1700,
      KX(370), 1820,
      };
  goto_traj(traj, arfast(ROBOT_SIDE_CUBE_CLAW,TABLE_SIDE_UP));
  idle_delay_ms(2000);
  cube_claw_open();
  goto_xya(KX(370), 1460, arfast(ROBOT_SIDE_CUBE_CLAW,TABLE_SIDE_MAIN));
  ROME_SENDWAIT_ASSERV_SET_SERVO(&rome_asserv, CUBE_CLAW_ELEVATOR, CUBE_CLAW_ELEVATOR_UP);

}

void strat_test(void)
{
#if 1
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
#endif
}
