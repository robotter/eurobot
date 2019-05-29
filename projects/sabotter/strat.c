#include <avarix.h>
#include <math.h>
#include <stdlib.h>
#include <clock/clock.h>
#include <util/delay.h>
#include <avarix/portpin.h>
#include <rome/rome.h>
#include <timer/uptime.h>
#include <idle/idle.h>
#include <rome/rome.h>
#include <pathfinding/pathfinding.h>
#include "strat.h"
#include "config.h"
#include "common.h"
#include "leds.h"

#define ANGLE_TYPE__ float
#include "modulo.inc.h"
#undef ANGLE_TYPE__

// Default timeout for "goto" orders
#define GOTO_TIMEOUT_MS  10000
// Time to wait before returning a DETECTION_PATH_BLOCKED status
#define DETECTION_WAIT_MS 2000

extern pathfinding_t pathfinder;

void rome_send_pathfinding_path(const pathfinding_t *finder)
{
  ROME_SEND_PATHFINDING_PATH(ROME_DST_PADDOCK, finder->path, finder->path_size);
}

robot_state_t robot_state;
// we don't put 'kx' on robot_state to allow it to be static
// this avoids optimization errors
static float robot_kx;

typedef enum {
  RS_VERYSLOW = 0,
  RS_SLOW,
  RS_NORMAL,
  RS_FAST,
} robot_speed_t;

typedef enum {
  ROBOT_SIDE_LEFT = 0,
  ROBOT_SIDE_RIGHT,
  ROBOT_SIDE_BACK,
} robot_side_t;

typedef enum {
  TABLE_SIDE_LEFT = 0,
  TABLE_SIDE_RIGHT,
  TABLE_SIDE_UP,
  TABLE_SIDE_DOWN,
} table_side_t;

// Return first value if left team (x<0), second value if right team (x>0)
#define TEAM_SIDE_VALUE(left,right)  (robot_state.team == TEAM_YELLOW ? (left) : (right))

#define TABLE_SIDE_MAIN  TEAM_SIDE_VALUE(TABLE_SIDE_LEFT, TABLE_SIDE_RIGHT)
#define TABLE_SIDE_AUX  TEAM_SIDE_VALUE(TABLE_SIDE_RIGHT, TABLE_SIDE_LEFT)

#define ROBOT_SIDE_MAIN  TEAM_SIDE_VALUE(ROBOT_SIDE_RIGHT, ROBOT_SIDE_LEFT)
#define ROBOT_SIDE_AUX  TEAM_SIDE_VALUE(ROBOT_SIDE_LEFT, ROBOT_SIDE_RIGHT)
#define AUTOSET_MAIN  TEAM_SIDE_VALUE(AUTOSET_LEFT, AUTOSET_RIGHT)
#define AUTOSET_AUX  TEAM_SIDE_VALUE(AUTOSET_RIGHT, AUTOSET_LEFT)
#define KX(x) (robot_kx*(x))

// align robot face along table side
float arfast(robot_side_t face, table_side_t side) {
  switch(face){
    case ROBOT_SIDE_LEFT:
      switch(side){
        case TABLE_SIDE_LEFT:
          return M_PI/6;
        case TABLE_SIDE_RIGHT:
          return -5*M_PI/6;
        case TABLE_SIDE_UP:
          return -M_PI/3;
        case TABLE_SIDE_DOWN:
          return 2*M_PI/3;
      }
      break;
    case ROBOT_SIDE_RIGHT:
      switch(side){
        case TABLE_SIDE_LEFT:
          return 5*M_PI/6;
        case TABLE_SIDE_RIGHT:
          return -M_PI/6;
        case TABLE_SIDE_UP:
          return M_PI/3;
        case TABLE_SIDE_DOWN:
          return -2*M_PI/3;
      }
      break;
    case ROBOT_SIDE_BACK:
      switch(side){
        case TABLE_SIDE_LEFT:
          return -M_PI/2;
        case TABLE_SIDE_RIGHT:
          return M_PI/2;
        case TABLE_SIDE_UP:
          return M_PI;
        case TABLE_SIDE_DOWN:
          return 0;
      }
      break;
    default:
      ROME_LOG(ROME_DST_PADDOCK, ERROR, "arfast : invalid angle consign");
      break;
  }
  return 0;
}

void update_score(uint16_t points) {
  robot_state.points += points;
  ROME_LOGF(ROME_DST_PADDOCK, INFO, "score : %u", robot_state.points);
}

typedef enum {
  ORDER_SUCCESS = 0,
  ORDER_PATHFINDING_FAILED,
  ORDER_DETECTION,
  ORDER_ABORTED,
  ORDER_TIMEOUT,
  ORDER_FAILURE,
} order_result_t;

/// delay for some ms
void idle_delay_ms(uint32_t time_ms) {
  uint32_t start_time_us = uptime_us();
  while(uptime_us() - start_time_us < time_ms*1000)
    idle();
}


// Aliases using default waiting time
#define goto_xya(x,y,a)  goto_xya_wait((x),(y),(a),GOTO_TIMEOUT_MS)
#define goto_xya_synced(x,y,a)  goto_xya_synced_wait((x),(y),(a),GOTO_TIMEOUT_MS)
#define goto_xya_panning(x,y,a)  goto_xya_panning_wait((x),(y),(a),GOTO_TIMEOUT_MS)
#define goto_xya_rel(x,y,a)  goto_xya_rel_wait((x),(y),(a),GOTO_TIMEOUT_MS)
#define goto_traj(xy,a)  goto_traj_wait((xy),(a),GOTO_TIMEOUT_MS)
#define goto_traj_n(xy,n,a)  goto_traj_n_wait((xy),(n),(a),GOTO_TIMEOUT_MS)


#define IN_RANGE(x,min,max) ((x) >= (min) && (x) <= (max))

// Return true if an opponent is detected within an arc
bool opponent_detected_in_arc(float angle, float fov, float distance)
{
  float amin = float_modulo__(angle - fov/2, 0, 2*M_PI);
  float amax = float_modulo__(angle + fov/2, 0, 2*M_PI);
  if(amin > amax) {
    amax += 2*M_PI;
  }
  for(uint8_t i=0; i<R3D2_OBJECTS_MAX; i++) {
    const r3d2_object_t *object = &robot_state.r3d2.objects[i];
    if(!object->detected) {
      continue;
    }
    const float d_a = float_modulo__(object->a + robot_state.current_pos.a, 0, 2*M_PI);
    const bool in_arc = IN_RANGE(d_a, amin, amax) || IN_RANGE(d_a+2*M_PI, amin, amax);
    if(object->r < distance && in_arc) {
      return true;
    }
  }
  return false;
}

// Check if an opponent is in a corridor
bool opponent_detected_in_corridor(float angle, float width, float depth)
{
  static uint32_t detection_log_timer = 0;
  for(uint8_t i=0; i<R3D2_OBJECTS_MAX; i++) {
    const r3d2_object_t *object = &robot_state.r3d2.objects[i];
    if(!object->detected) {
      continue;
    }
    const float d_a = robot_state.current_pos.a + object->a - angle;
    const float dx = object->r * cos(d_a);
    const float dy = object->r * sin(d_a);
    if(uptime_us() - detection_log_timer > 300000) {
      ROME_LOGF(ROME_DST_PADDOCK, DEBUG, "corridor : d_a %f, r %f", d_a, object->r);
      ROME_LOGF(ROME_DST_PADDOCK, DEBUG, "corridor : dx %f, dy %f", dx, dy);
      detection_log_timer = uptime_us();
    }
    if(dx >= 0 && dx < depth && fabs(dy) <= width) {
      return true;
    }
  }
  return false;
}

float angle_from_carrot(int16_t cx, int16_t cy)
{
  const int16_t dx = cx - robot_state.current_pos.x;
  const int16_t dy = cy - robot_state.current_pos.y;
  return atan2(dy, dx);
}

static void set_xya_wait(double x, double y, double a) {
  ROME_SENDWAIT_ASSERV_SET_XYA(ROME_DST_ASSERV, x, y, 1000.0*a);
}

typedef enum {
  DETECTION_PATH_FREE = 0,
  DETECTION_PATH_BLOCKED,
  DETECTION_PATH_CLEARED,
} detection_path_t;


/** Check if an opponent is close, wait for it to move away
 *
 * If an opponent is closer than R3D2_STOP_DISTANCE:
 *  - disable asserv
 *  - wait until it moves away or time_end
 */
detection_path_t avoid_opponent_close(float angle, uint32_t time_end) {
  if(!opponent_detected_in_corridor(angle, R3D2_CORRIDOR_WIDTH, R3D2_STOP_DISTANCE)) {
    return DETECTION_PATH_FREE;
  }

  ROME_LOG(ROME_DST_PADDOCK, DEBUG, "opponent too close");
  ROME_SENDWAIT_ASSERV_GOTO_XY_REL(ROME_DST_ASSERV, 0, 0, 0);
  ROME_SENDWAIT_ASSERV_ACTIVATE(ROME_DST_ASSERV, 0);

  for(;;) {
    idle();

    if(!opponent_detected_in_corridor(angle, R3D2_CORRIDOR_WIDTH, R3D2_STOP_DISTANCE)) {
      // opponent moved away, resume asserv and exit
      ROME_LOG(ROME_DST_PADDOCK, DEBUG, "close opponent went away, resume initial trajectory");
      ROME_SENDWAIT_ASSERV_GOTO_XY_REL(ROME_DST_ASSERV, 0, 0, 0);
      ROME_SENDWAIT_ASSERV_ACTIVATE(ROME_DST_ASSERV, 1);
      return DETECTION_PATH_CLEARED;
    }

    const uint32_t time = uptime_us();
    if(time > time_end) {
      // timeout: path is blocked
      ROME_LOGF(ROME_DST_PADDOCK, DEBUG, "close opponent: timeout %lu > %lu", time, time_end);
      ROME_SENDWAIT_ASSERV_GOTO_XY_REL(ROME_DST_ASSERV, 0, 0, 0);
      ROME_SENDWAIT_ASSERV_ACTIVATE(ROME_DST_ASSERV, 1);
      return DETECTION_PATH_BLOCKED;
    }
  }
}

/** Check if an opponent is on the way to avoid it
 *
 * If an opponent is closer than R3D2_AVOID_DISTANCE:
 *  - move slightly backwards
 *  - wait at most DETECTION_WAIT_MS for it to move away
 *  - stop if it is to close (see avoid_opponent_close())
 */
detection_path_t avoid_opponent_on_the_way(int16_t x, int16_t y) {
  const float angle = angle_from_carrot(x, y);

  if(!opponent_detected_in_corridor(angle, R3D2_CORRIDOR_WIDTH, R3D2_AVOID_DISTANCE)) {
    return DETECTION_PATH_FREE;
  }

  uint32_t time_end = uptime_us() + DETECTION_WAIT_MS * 1000UL;
  ROME_LOG(ROME_DST_PADDOCK, INFO, "goto: opponent detected");
  ROME_SENDWAIT_ASSERV_GOTO_XY_REL(ROME_DST_ASSERV, -R3D2_AVOID_MOVEBACK*cos(angle), -R3D2_AVOID_MOVEBACK*sin(angle), 0);

  for(;;) {
    idle();

    if(avoid_opponent_close(angle, time_end) == DETECTION_PATH_BLOCKED) {
      // timeout on close opponent
      return DETECTION_PATH_BLOCKED;
    }

    if(!opponent_detected_in_corridor(angle, R3D2_CORRIDOR_WIDTH, R3D2_AVOID_DISTANCE + R3D2_AVOID_MOVEBACK)) {
      // opponent moved away
      ROME_LOG(ROME_DST_PADDOCK, DEBUG, "goto: opponent went away, resume initial trajectory");
      return DETECTION_PATH_CLEARED;
    }

    const uint32_t time = uptime_us();
    if(time > time_end) {
      // timeout: path is blocked
      ROME_LOGF(ROME_DST_PADDOCK, DEBUG, "goto: timeout %lu > %lu", time, time_end);
      return DETECTION_PATH_BLOCKED;
    }
  }
}

// Wait for asserv xy/a order, handle detection
order_result_t wait_asserv_xya_and_detect(int16_t x, int16_t y, uint32_t time_end) {
  robot_state.asserv.xy = false;
  robot_state.asserv.a = false;
  for(;;) {
    idle();
    if(robot_state.asserv.xy && robot_state.asserv.a) {
      return ORDER_SUCCESS;
    }
    const detection_path_t dp = avoid_opponent_on_the_way(x, y);
    if(dp == DETECTION_PATH_BLOCKED) {
      return ORDER_DETECTION;
    } else if(dp == DETECTION_PATH_CLEARED) {
      return ORDER_ABORTED;
    }

    const uint32_t time = uptime_us();
    if(time > time_end) {
      ROME_LOGF(ROME_DST_PADDOCK, INFO, "wait asserv: timeout %lu > %lu", time, time_end);
      return ORDER_TIMEOUT;
    }
  }
}

/// Go to given position, avoid opponents
order_result_t goto_xya_wait(int16_t x, int16_t y, float a, uint16_t timeout_ms)
{
  const uint32_t time_end = uptime_us() + timeout_ms * 1000UL;
  for(;;) {
    idle();
    ROME_SENDWAIT_ASSERV_GOTO_XY(ROME_DST_ASSERV, x, y, 1000*a);
    order_result_t or = wait_asserv_xya_and_detect(x, y, time_end);
    if(or != ORDER_ABORTED) {
      return or;
    }
    // resend order
  }
}

/// Go to given relative position, avoid opponents
order_result_t goto_xya_rel_wait(int16_t x, int16_t y, float a, uint16_t timeout_ms)
{
  uint32_t time_end = uptime_us() + timeout_ms * 1000UL;
  for(;;) {
    idle();
    ROME_SENDWAIT_ASSERV_GOTO_XY_REL(ROME_DST_ASSERV, x, y, 1000*a);
    order_result_t or = wait_asserv_xya_and_detect(x, y, time_end);
    if(or != ORDER_ABORTED) {
      return or;
    }
    // do not resend order, it would be obsolete since we already moved
    return ORDER_DETECTION;
  }
}

/// Go to given position, avoid opponents
order_result_t goto_xya_synced_wait(int16_t x, int16_t y, float a, uint16_t timeout_ms)
{
  const uint32_t time_end = uptime_us() + timeout_ms * 1000UL;
  for(;;) {
    idle();
    ROME_SENDWAIT_ASSERV_GOTO_XYA_SYNCED(ROME_DST_ASSERV, x, y, 1000*a);
    order_result_t or = wait_asserv_xya_and_detect(x, y, time_end);
    if(or != ORDER_ABORTED) {
      return or;
    }
    // resend order
  }
}

/// Go to given position, with robot panning and scanning, avoid opponents
order_result_t goto_xya_panning_wait(int16_t x, int16_t y, float pan_angle, uint16_t timeout_ms)
{
  uint32_t time_end = uptime_us() + timeout_ms * 1000UL;
  for(;;) {
    idle();
    ROME_SENDWAIT_ASSERV_GOTO_XYA_PANNING(ROME_DST_ASSERV, x, y, 1000*pan_angle);
    order_result_t or = wait_asserv_xya_and_detect(x, y, time_end);
    if(or != ORDER_ABORTED) {
      return or;
    }
    // resend order
  }
}

/// Execute trajectory, avoid opponents
#define goto_traj_wait(xy,a,w) goto_traj_n_wait((xy), ARRAY_LEN(xy), (a), (w))
order_result_t goto_traj_n_wait(int16_t* xy, uint8_t n, float a, uint16_t timeout_ms)
{
  const uint32_t time_end = uptime_us() + timeout_ms * 1000UL;
  uint8_t path_i = 0;
  for(;;) {
    idle();
    const uint8_t xy_len = n - path_i;
    ROME_SENDWAIT_ASSERV_RUN_TRAJ(ROME_DST_ASSERV, 1000*a, xy+path_i*2, xy_len);
    const int16_t x = xy[2*robot_state.asserv.path_i];
    const int16_t y = xy[2*robot_state.asserv.path_i+1];
    robot_state.asserv.path_i = path_i;
    robot_state.asserv.path_n = xy_len/2;

    order_result_t or = wait_asserv_xya_and_detect(x, y, time_end);
    if(or != ORDER_ABORTED) {
      return or;
    }
    // resend order, continue from where asserv stopped
    path_i = robot_state.asserv.path_i;
  }
}

/// Execute pathfinding type trajectory
order_result_t goto_pathfinding_node(uint8_t goal, float angle)
{
  for(;;) {
    idle();
    // update obstacles
    // first is our other robot, the others are the detected robots
    pathfinding_obstacle_t obstacles[R3D2_OBJECTS_MAX+1] = {{0,0,0}};

    obstacles[0].x = robot_state.partner_pos.x;
    obstacles[0].y = robot_state.partner_pos.y;
    obstacles[0].r = 400;

    // compute absolute position of detected robots
    for(uint8_t i=0; i<R3D2_OBJECTS_MAX; i++) {
      r3d2_object_t *object = &robot_state.r3d2.objects[i];
      if(object->detected) {
        float da = float_modulo__(robot_state.current_pos.a + object->a, 0, 2*M_PI);
        obstacles[i+1].x = (float) robot_state.current_pos.x + object->r*cos(da);
        obstacles[i+1].y = (float) robot_state.current_pos.y + object->r*sin(da);
        obstacles[i+1].r = 400;
        ROME_LOGF(ROME_DST_PADDOCK, DEBUG, "obstacle ra %f %f", object->r, da);
        ROME_LOGF(ROME_DST_PADDOCK, DEBUG, "obstacle xy %d %d", obstacles[i].x, obstacles[i].y);
      }
    }

    pathfinder.obstacles = obstacles;
    pathfinder.obstacles_size = ARRAY_LEN(obstacles);

    // search for a path
    const uint8_t start = pathfinding_nearest_node(
        &pathfinder, robot_state.current_pos.x, robot_state.current_pos.y);

    ROME_LOGF(ROME_DST_PADDOCK, DEBUG, "searching path from %u to %u", start, goal);
    // check if we are already arrived
    if(start == goal) {
      return ORDER_SUCCESS;
    }

    pathfinding_search(&pathfinder, start, goal);
    ROME_LOGF(ROME_DST_PADDOCK, DEBUG, "path found (%u)", pathfinder.path_size);
    if(pathfinder.path_size == 0) {
      return ORDER_PATHFINDING_FAILED;
    }

    rome_send_pathfinding_path(&pathfinder);

    // send the trajectory to asserv, but do not go to nearest node
    int16_t traj[pathfinder.path_size * 2];
    for(int i=0; i < pathfinder.path_size; i++) {
      traj[2*i] = pathfinder.nodes[pathfinder.path[i]].x;
      traj[2*i+1] = pathfinder.nodes[pathfinder.path[i]].y;
    }

    const uint32_t time_end = uptime_us() + GOTO_TIMEOUT_MS * 1000UL;
    ROME_SENDWAIT_ASSERV_RUN_TRAJ(ROME_DST_ASSERV, 1000*angle, traj, ARRAY_LEN(traj));
    robot_state.asserv.xy = false;
    robot_state.asserv.a = false;
    for(;;) {
      idle();
      if(robot_state.asserv.xy && robot_state.asserv.a) {
        return ORDER_SUCCESS;
      }

      const uint32_t time = uptime_us();
      if(time > time_end) {
        ROME_LOGF(ROME_DST_PADDOCK, INFO, "trajectory: timeout %lu > %lu", time, time_end);
        return ORDER_TIMEOUT;
      }

      // check opponent position
      const float angle = angle_from_carrot(traj[2*robot_state.asserv.path_i],
                                            traj[2*robot_state.asserv.path_i+1]);
      if(avoid_opponent_close(angle, time + DETECTION_WAIT_MS * 1000UL) == DETECTION_PATH_BLOCKED) {
        // timeout on close opponent
        return ORDER_DETECTION;
      }

      // if opponent is detected, compute a new trajectory
      if(opponent_detected_in_corridor(angle, R3D2_CORRIDOR_WIDTH, R3D2_AVOID_DISTANCE)) {
        ROME_LOG(ROME_DST_PADDOCK, DEBUG, "opponenent detected, computing a new trajectory");
        break;
      }
    }
  }
  return ORDER_FAILURE;
}


/// Move around a point in several steps
void go_around(int16_t cx, int16_t cy, float a)
{
  const int16_t rx = robot_state.current_pos.x;
  const int16_t ry = robot_state.current_pos.y;
  const float   ra = robot_state.current_pos.a;
  a += M_PI/6;
  const uint8_t nb_steps = 6;
  const int16_t x = rx - cx;
  const int16_t y = ry - cy;

  for(int i = 0; i < nb_steps; i++) {
    const float a_step = (i+1)*(a + M_PI/6) / nb_steps;
    const int16_t tmpx = cx + x*cos(a_step) - y*sin(a_step);
    const int16_t tmpy = cy + y*cos(a_step) + x*sin(a_step);
    const float tmpa = ra + i*(a/nb_steps);
    goto_xya_synced(tmpx, tmpy, tmpa);
  }
}

/// Do an autoset
void autoset(robot_side_t robot_side, autoset_side_t table_side, float x, float y)
{
  ROME_SENDWAIT_ASSERV_AUTOSET(ROME_DST_ASSERV, robot_side, table_side, x, y);
  robot_state.asserv.autoset = 0;
  while(!robot_state.asserv.autoset) {
    idle();
  }
}

bool starting_cord_plugged(void)
{
  for(;;) {
    // filter the value: wait for 1000 consecutive equal values
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
  ROME_LOG(ROME_DST_PADDOCK, INFO, "Unplug starting cord ...");
  while(starting_cord_plugged()) {
    idle();
    if((uptime_us() / 500000) % 2 == 0) {
      LED_RGB_SET(LED_COLOR_BLUE);
    } else {
      LED_RGB_SET(LED_COLOR_BLACK);
    }
  }

  // wait for color to be selected
  // color is selected when starting cord is plugged
  ROME_LOG(ROME_DST_PADDOCK, INFO, "Select color ...");
  team_t team = TEAM_NONE;
  for(;;) {
    idle();
    if(portpin_in(&COLOR_SELECTOR_PP)) {
      LED_RGB_SET(LED_COLOR_YELLOW);
      team = TEAM_YELLOW;
    } else {
      LED_RGB_SET(LED_COLOR_MAGENTA);
      team = TEAM_PURPLE;
    }
    if(starting_cord_plugged()) {
      LED_RGB_SET(LED_COLOR_BLACK);
      break;
    }
  }

  if(team == TEAM_YELLOW) {
    ROME_LOG(ROME_DST_PADDOCK, INFO, "Color : YELLOW !");
  } else {
    ROME_LOG(ROME_DST_PADDOCK, INFO, "Color : PURPLE !");
  }

  // wait 2s before next step
  const uint32_t tend = uptime_us() + 2e6;
  while(uptime_us() < tend) {
    idle();
  }

  return team;
}

void strat_wait_start(void)
{
  while(starting_cord_plugged()) {
    idle();
    if((uptime_us() / 500000) % 2 == 0) {
      if(robot_state.team == TEAM_YELLOW) {
        LED_RGB_SET(LED_COLOR_YELLOW);
      } else {
        LED_RGB_SET(LED_COLOR_MAGENTA);
      }
    } else {
      LED_RGB_SET(LED_COLOR_BLUE);
    }
  }

  LED_RGB_SET(LED_COLOR_BLACK);
  ROME_SENDWAIT_START_TIMER(ROME_DST_ASSERV);
#if (defined UART_MECA)
  ROME_SENDWAIT_START_TIMER(ROME_DST_MECA);
#endif
}

// Aliases for graph nodes, independent of team color
#define PATHFINDING_GRAPH_NODE_TEAM_SIDE(name,left,right)  TEAM_SIDE_VALUE(PATHFINDING_GRAPH_NODE_##left##_##name, PATHFINDING_GRAPH_NODE_##right##_##name)
#define PATHFINDING_GRAPH_NODE_RAMP_ENTRANCE  PATHFINDING_GRAPH_NODE_TEAM_SIDE(RAMP_ENTRANCE, YELLOW, PURPLE)
#define PATHFINDING_GRAPH_NODE_SMALL_DISPENSER  PATHFINDING_GRAPH_NODE_TEAM_SIDE(SMALL_DISPENSER, YELLOW, PURPLE)
#define PATHFINDING_GRAPH_NODE_LARGE_DISPENSER_NEAR  PATHFINDING_GRAPH_NODE_TEAM_SIDE(LARGE_DISPENSER_NEAR, YELLOW, PURPLE)
#define PATHFINDING_GRAPH_NODE_LARGE_DISPENSER_FAR  PATHFINDING_GRAPH_NODE_TEAM_SIDE(LARGE_DISPENSER_FAR, YELLOW, PURPLE)
#define PATHFINDING_GRAPH_NODE_GOLDENIUM  PATHFINDING_GRAPH_NODE_TEAM_SIDE(GOLDENIUM, PURPLE, YELLOW)
#define PATHFINDING_GRAPH_NODE_ACCELERATED_BLUE  PATHFINDING_GRAPH_NODE_TEAM_SIDE(ACCELERATED_BLUE, PURPLE, YELLOW)
#define PATHFINDING_GRAPH_NODE_BALANCE  PATHFINDING_GRAPH_NODE_TEAM_SIDE(BALANCE, YELLOW, PURPLE)
#define PATHFINDING_GRAPH_NODE_GALIPETTE_START  PATHFINDING_GRAPH_NODE_TEAM_SIDE(GALIPETTE_START, YELLOW, PURPLE)


// Score for each action

#define SCORE_ATOM_IN_START_AREA  1
#define SCORE_ATOM_IN_START_AREA_COLOR_BONUS  5
#define SCORE_GOLDENIUM_IN_START_AREA  6

#define SCORE_REDIUM_IN_BALANCE  4
#define SCORE_GREENIUM_IN_BALANCE  8
#define SCORE_BLUEIUM_IN_BALANCE  12
#define SCORE_GOLDENIUM_IN_BALANCE  24

#define SCORE_ATOM_IN_ACCELERATOR  10
#define SCORE_GOLDENIUM_FREED  10
#define SCORE_GOLDENIUM_EXTRACTED  20

#define SCORE_EXPERIENCE_INSTALLED  5
#define SCORE_EXPERIENCE_ACTIVATED  15
#define SCORE_EXPERIENCE_SUCCESS  20


//include robot's strat files
#if (defined GALIPEUR)
# include "strat_galipeur.inc.c"
#elif (defined GALIPETTE)
# include "strat_galipette.inc.c"
#else
# error Either GALIPEUR or GALIPETTE must be defined
#endif
