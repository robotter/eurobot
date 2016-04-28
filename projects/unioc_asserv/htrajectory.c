/*  
 *  Copyright RobOtter (2010)
 * 
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

/** \file htrajectory.c
 * \date 02/01/2010 
 * \author JD
 *
 * Trajectory management NG
 *
 */

#include <math.h>
#include "htrajectory.h"
#include "hrobot_manager.h"
#include "settings.h"

#include "telemetry.h"

#include "avoidance.h"

// float angles normalization
#define ANGLE_TYPE__ float
#include "modulo.inc.h"
#undef ANGLE_TYPE__

#include <stdio.h>

#define NORMALIZE_RADIANS_FLOAT_0_2PI(x) (float_modulo__(x, 0, 2*M_PI))
#define NORMALIZE_RADIANS_FLOAT_PI_PI(x) (float_modulo__(x, -M_PI, M_PI))

// avoidance system
extern avoidance_t avoidance;

typedef struct {
  float left,right,up,down;
  float base;

} autoset_configuration_t ;

const autoset_configuration_t AUTOSET_CONFIGURATIONS[] = {
  [ROBOT_SIDE_BACK] = {
    .left =  -M_PI_2,
    .right = +M_PI_2,
    .up =     M_PI,
    .down =   0.0,
    .base =  -0.5*M_PI,
  },
  [ROBOT_SIDE_RIGHT] = {
    .left =   5*M_PI/6.0,
    .right = -M_PI/6.0,
    .up =     M_PI/3.0,
    .down =  -2*M_PI/3.0,
    .base =   M_PI/6.0,
  },
  [ROBOT_SIDE_LEFT] = {
    .left = M_PI/6.0,
    .right = -5*M_PI/6.0,
    .up = -2*M_PI/6.0,
    .down = 4*M_PI/6.0,
    .base = 5*M_PI/6.0,
  },
};

/* -- private functions -- */

static inline double vDotProduct(vect_xy_t a, vect_xy_t b) {
  return a.x*b.x + a.y*b.y;
}

static inline vect_xy_t vSubtract(vect_xy_t a, vect_xy_t b) {
  return (vect_xy_t){.x = a.x - b.x, .y = a.y - b.y};
}

static inline vect_xy_t vAdd(vect_xy_t a, vect_xy_t b) {
  return (vect_xy_t){.x = a.x + b.x, .y = a.y + b.y};
}

static inline double vSqNorm(vect_xy_t a) {
  return a.x*a.x + a.y*a.y;
}

static inline double vNorm(vect_xy_t a) {
  return sqrtf(a.x*a.x + a.y*a.y);
}

static inline vect_xy_t vScalarProduct(vect_xy_t a, double s) {
  return (vect_xy_t){.x = s*a.x, .y = s*a.y};
}

/** \brief Set htj->carrot angle, transfer orders to low level CSs */
static inline void setCarrotAPosition( htrajectory_t *htj, double a )
{
  robot_cs_set_a_consign( htj->rcs, RCS_RAD_TO_CSUNIT*a );
}

/** \brief Set htj->carrot position, transfer orders to low level CSs */
static inline void setCarrotXYPosition( htrajectory_t *htj, vect_xy_t pos )
{
  robot_cs_set_xy_consigns( htj->rcs, RCS_MM_TO_CSUNIT*pos.x,
                                      RCS_MM_TO_CSUNIT*pos.y );
}

/** \brief Check if robot is in window of current point
  * \return 1- if target reached 0- otherwise
  */
uint8_t inWindowXY( htrajectory_t *htj )
{
  vect_xy_t robot,point;
  double dx,dy,dl;
  uint8_t inWindow;

  // get robot position
  hposition_get_xy( htj->hrp, &robot);

  // get next point position
  point = htj->path[htj->pathIndex];

  // XXX stop window is default window ?
  if( htj->state == STATE_PATH_MID )
    dl = htj->xySteeringWindow;
  else
    dl = htj->xyStopWindow;

  dx = point.x - robot.x;
  dy = point.y - robot.y;

  // coarse inegality to save computing time
  if( dx > dl && dy > dl )
    inWindow = 0;
  else {
    // (squared inegality)
    // check if robot is in window  
    if( dx*dx + dy*dy < dl*dl )
      inWindow = 1;
    else
      inWindow = 0;
  }

  return inWindow;
}

static inline void computeSpeeds(htrajectory_t *htj,
  double sqErrorLength, double nextSpeed, double currAcc) {

  double decDistance = 0.5*((htj->carrotSpeed)*(htj->carrotSpeed)
                        - nextSpeed*nextSpeed)/currAcc;
  // * deceleration phase
  if( sqErrorLength < decDistance*decDistance )
  {
    // set speed to minimum speed when acceleration will cause algorithm to overshoot
    if( htj->carrotSpeed - nextSpeed > currAcc )
      htj->carrotSpeed -= currAcc;
    else
      htj->carrotSpeed = nextSpeed;
  } 
  // * acceleration phase
  else if( (htj->carrotSpeed) < htj->cruiseSpeed )
  {
    // set speed to maximum speed when acceleration will cause algorithm to overshoot
    if( htj->cruiseSpeed - htj->carrotSpeed > htj->cruiseAcc )
      htj->carrotSpeed += htj->cruiseAcc;
    else
      htj->carrotSpeed = htj->cruiseSpeed;
  }
  // * stable phase
  else
  {
    /* nothing to do */
  }

}

/* -- public functions -- */

void htrajectory_init( htrajectory_t *htj,
                        hrobot_position_t *hrp,
                        robot_cs_t *rcs,
                        struct quadramp_filter *qramp_angle)
{
  htj->hrp = hrp;
  htj->rcs = rcs;
  htj->qramp_a = qramp_angle;


  htj->cruiseSpeed = 0.0;
  htj->cruiseAcc = 0.0;

  htj->steeringSpeed = 0.0;
  htj->steeringAcc = 0.0;

  htj->stopSpeed = 0.0;
  htj->stopAcc = 0.0;

  htj->xySteeringWindow = 1.0;
  htj->xyStopWindow = 1.0;
  htj->aStopWindow = 1.0;

  htj->carrot = (vect_xy_t){0.0,0.0};
  htj->carrotA = 0.0;
  htj->carrotSpeed = 0.0;

  htj->carrotA_reached = 0;

  htj->pathIndex = 0;
  htj->state = STATE_STOP;
  htj->blocked = 0;

  htj->lpos = (vect_xy_t){0.0,0.0};
  htj->la = 0.0;

  return;
}

/* -- orders -- */

void htrajectory_run( htrajectory_t *htj, step_t *path, uint8_t n )
{
  // copy points to internal structure
  memcpy(htj->path, path, n*sizeof(step_t));

  htj->pathIndex = 0;

  if(htj->pathSize == 1)
    htj->state = STATE_PATH_LAST;
  else
    htj->state = STATE_PATH_MID;

  // reset carrot speed
  htj->carrotSpeed = 0;

  return;
}

void htrajectory_gotoA( htrajectory_t *htj, double a )
{
  double robot_a;
  double da;

  // get robot angle
  hposition_get_a( htj->hrp, &robot_a );

  // compute distance between consign and position modulo 2pi
  da = NORMALIZE_RADIANS_FLOAT_PI_PI(a - robot_a);

  // update consign
  htj->carrotA = robot_a + da;
  htj->carrotA_reached = 0;

  // set robot carrot
  setCarrotAPosition(htj, htj->carrotA );
}

void htrajectory_gotoXY( htrajectory_t *htj, double x, double y)
{
  vect_xy_t path;

  // create a one point path
  path = (vect_xy_t){x,y};

  // load and run path
  htrajectory_run(htj, &path,  1);
}

void htrajectory_gotoA_R( htrajectory_t *htj, double da)
{
  double ra;
  hposition_get_a( htj->hrp, &ra);
  htrajectory_gotoA( htj, ra + da);
}

void htrajectory_gotoXY_R( htrajectory_t *htj, double dx, double dy)
{
  vect_xy_t rp;
  hposition_get_xy( htj->hrp, &rp);
  htrajectory_gotoXY( htj, rp.x + dx, rp.y + dy);
}

/* -- speed management -- */

void htrajectory_setASpeed( htrajectory_t *htj, double speed, double acc )
{
  quadramp_set_1st_order_vars(htj->qramp_a, speed, speed);
  quadramp_set_2nd_order_vars(htj->qramp_a, acc, acc);
}

void htrajectory_setXYCruiseSpeed( htrajectory_t *htj, double speed, double acc )
{
  htj->cruiseSpeed = speed;
  htj->cruiseAcc = acc;
}

void htrajectory_setXYSteeringSpeed( htrajectory_t *htj, double speed, double acc )
{
  htj->steeringSpeed = speed;
  htj->steeringAcc = acc;
}

void htrajectory_setXYStopSpeed( htrajectory_t *htj, double speed, double acc )
{
  htj->stopSpeed = speed;
  htj->stopAcc = acc;
}

/* -- target window management -- */

void htrajectory_setSteeringWindow( htrajectory_t *htj, double xywin )
{
  htj->xySteeringWindow = xywin;
}

void htrajectory_setStopWindows( htrajectory_t *htj, double xywin, double awin )
{
  htj->xyStopWindow = xywin;
  htj->aStopWindow = awin;
}

/* -- status -- */

uint8_t htrajectory_doneXY( htrajectory_t *htj )
{
  if( htj->state == STATE_STOP )
    return 1;
  else
    return 0;
}

uint8_t htrajectory_doneA( htrajectory_t *htj )
{
  double robot_a;
  double da;

  hposition_get_a(htj->hrp, &robot_a);

  da = fabs(htj->carrotA - robot_a);

  if( da < htj->aStopWindow )
    return 1;
  else
    return 0;
}

uint8_t htrajectory_blocked( htrajectory_t *htj )
{
  return htj->blocked;
}

uint8_t htrajectory_doneAutoset( htrajectory_t *htj )
{
  if( (htj->state == STATE_AUTOSET_HEADING)
      || (htj->state == STATE_AUTOSET_HEADING_WAIT)
      || (htj->state == STATE_AUTOSET_MOVE) )
    return 0;
  else
    return 1;
}

/* -- trajectory update -- */

static void _htrajectory_update( htrajectory_t *htj )
{
  double sqErrorLength;
  vect_xy_t *point;
  double dx,dy;
  direction_t dir;
  vect_xy_t robot;

  // manage angular orders / position
  if( !(htj->carrotA_reached) && htrajectory_doneA(htj) )
  {
    htj->carrotA_reached = 1;
  }

  // trajectory states
  if( htj->state == STATE_STOP )
    /* nothing to do */
    return;

  if( htj->state == STATE_AUTOSET_HEADING )
  {
    // wait for robot heading OK
    if( htrajectory_doneA(htj) )
    {
      // wait some time
      htj->state = STATE_AUTOSET_HEADING_WAIT;
      // reste heading wait count
      htj->autosetHeadingWaitCount = 0;
    }
    return;
  }

  if( htj->state == STATE_AUTOSET_HEADING_WAIT ) {

    htj->autosetHeadingWaitCount++;

    if(htj->autosetHeadingWaitCount > 50) {

      // set autoset to next step
      htj->state = STATE_AUTOSET_MOVE;

      // shutdown robot CSs
      robot_cs_activate(htj->rcs, 0);

      // store current robot position
      hposition_get(htj->hrp, &(htj->autosetInitPos));

      // reset autoset count
      htj->autosetCount = 0;
    }

    return;
  }

  if( htj->state == STATE_AUTOSET_MOVE )
  {
    // XXX maybe replaced with an advance detector XXX
    if(true)
    {
      htj->autosetCount++;

      const autoset_configuration_t autoconf =
        AUTOSET_CONFIGURATIONS[htj->autosetRobotSide];
      dx = SETTING_AUTOSET_SPEED*cos(autoconf.base);
      dy = SETTING_AUTOSET_SPEED*sin(autoconf.base);

      float alpha = MIN(1.0, 2.0*htj->autosetCount/SETTING_AUTOSET_ZEROCOUNT);

      // set course
      hrobot_set_motors(dx*alpha, dy*alpha, 0.0);

      if( htj->autosetCount > SETTING_AUTOSET_ZEROCOUNT )
      {
        // autoset done
        const autoset_configuration_t autoconf = 
          AUTOSET_CONFIGURATIONS[htj->autosetRobotSide];
        switch(htj->autosetTableSide)
        {
          case TS_LEFT:
            htj->autosetInitPos.alpha = autoconf.left;
            htj->autosetInitPos.x = htj->autosetTargetX;
            break;

          case TS_RIGHT:
            htj->autosetInitPos.alpha = autoconf.right;
            htj->autosetInitPos.x = htj->autosetTargetX;
            break;

          case TS_UP:
            htj->autosetInitPos.alpha = autoconf.up;
            htj->autosetInitPos.y = htj->autosetTargetY;
            break;

          case TS_DOWN:
            htj->autosetInitPos.alpha = autoconf.down;
            htj->autosetInitPos.y = htj->autosetTargetY;
            break;

          default:
            return;
        }

        hposition_set(htj->hrp, htj->autosetInitPos.x,
                                htj->autosetInitPos.y,
                                htj->autosetInitPos.alpha);
        // reset htrajectory carrot
        htrajectory_reset_carrot(htj);
        setCarrotXYPosition( htj, htj->carrot );
        // reactivate robot CSs damn it !
        robot_cs_activate(htj->rcs, 1);
        // set trajectory status to stop
        htj->state = STATE_STOP;
      }
    }
    else
      htj->autosetCount = 0;

    return;
  }

  // is robot blocked by something ?
  dir = avoidance_check(&avoidance);

  // if blocked ...
  if( dir != DIR_NONE )
  {
    // ... and previously not blocked
    if( !(htj->blocked) )
    {

      // get robot position
      hposition_get_xy( htj->hrp, &robot);

      // set carrot to current position
      setCarrotXYPosition(htj, robot);

      // reset carrot speed
      htj->carrotSpeed = 0;
      htj->blocked = 1;
    }

    // if robot blocked do not update anything more
    return;
  }

  // if NOT blocked and previously blocked
  if( (dir == DIR_NONE) && (htj->blocked) )
  {
    hposition_get_xy( htj->hrp, &robot);

    htj->carrot = robot;

    // resume target to carrot
    setCarrotXYPosition(htj, htj->carrot);

    htj->blocked = 0;
  }

  if( htj->blocked )
    return;

  // is robot in position ?
  if( inWindowXY(htj) )
  {
    // last point reached
    if( htj->state == STATE_PATH_LAST )
    {
      // put trajectory management to full stop
      htj->pathIndex = 0;
      htj->state = STATE_STOP;

      // set htj->carrot to last position
      setCarrotXYPosition( htj, htj->path[htj->pathSize - 1] );


      return;
    }

    // switch to next point 
    htj->pathIndex++;

    // if next point is last point
    if( htj->pathIndex >= htj->pathSize - 1 )
      htj->state = STATE_PATH_LAST;

  }
  
  vect_xy_t *ppoint;
  // set pointer on previous point in trajectory
  if(htj->pathIndex <= 0) {
    ppoint = NULL;
  }
  else {
    ppoint = htj->path + htj->pathIndex - 1;
  }
  // set pointer on current point in trajectory
  point = htj->path + htj->pathIndex;

  // --- compute speed consign & ramp ---

  // compute distance at which constant deceleration will bring robot
  // to desired speed
  double nextSpeed = 0.0;
  double currAcc = 0.0;
  double decDistance;

  if( htj->state == STATE_PATH_MID )
  {
    nextSpeed = htj->steeringSpeed;
    currAcc = htj->steeringAcc;
  }
  else {
    if( htj->state == STATE_PATH_LAST )
    {
      nextSpeed = htj->stopSpeed;
      currAcc = htj->stopAcc;
    }
    else
    {
      return;
    }
  }

  // get robot position
  hposition_get_xy( htj->hrp, &robot);

  if(ppoint != NULL) {
    // project vector from current position to target onto desired path
    vect_xy_t ab = vSubtract(*point, *ppoint);
    vect_xy_t rb = vSubtract(*point, robot);
    double dotp = vDotProduct(ab, rb);
    double sqAlongTrackError = dotp*dotp/vSqNorm(ab);

    // update speeds
    computeSpeeds(htj,sqAlongTrackError,nextSpeed,currAcc);

    // along track carrot, carrot which is located ON the desired track
    double alongTrackError = dotp/vNorm(ab);
    vect_xy_t atcarrot = vAdd(*ppoint, vScalarProduct(ab, MIN(1.0-(alongTrackError/vNorm(ab))+0.1,1.0) ));

    // target along track carrot using current speed
    vect_xy_t rc = vSubtract(atcarrot, robot);
    // limit 
    vect_xy_t dv = vScalarProduct(rc, htj->carrotSpeed/vNorm(rc));
    htj->carrot = vAdd(robot, dv);
  }
  else {
    // compute squared distance between current position and target
    vect_xy_t rb = vSubtract(*point,robot);
    sqErrorLength = vSqNorm(rb);

    // update speeds
    computeSpeeds(htj, sqErrorLength, nextSpeed, currAcc);

    vect_xy_t dv = vScalarProduct(rb, htj->carrotSpeed/vNorm(rb));
    htj->carrot = vAdd(robot, dv);
  }

  // --- update carrot position ---
  setCarrotXYPosition( htj, htj->carrot );
}

void htrajectory_update( htrajectory_t *htj ) {
  // update trajectory
  _htrajectory_update(htj);
  // update telemetries
  TM_DL_HTRAJ_STATE(htj->state);
  TM_DL_HTRAJ_PATH_INDEX(htj->pathIndex, htj->pathSize);
  TM_DL_HTRAJ_CARROT_XY(htj->carrot.x, htj->carrot.y);
  TM_DL_HTRAJ_SPEED(htj->carrotSpeed);
  TM_DL_HTRAJ_DONE(htrajectory_doneXY(htj), htrajectory_doneA(htj));
  TM_DL_HTRAJ_AUTOSET_DONE(htrajectory_doneAutoset(htj));
}

// --- AUTOSET ---

void htrajectory_autoset( htrajectory_t *htj, 
                          robot_side_t robot_side, tableSide_t table_side,
                          double x, double y)
{
  vect_xy_t robot;

  // reset trajectory and start autoset
  htj->pathIndex = 0;
  htj->state = STATE_AUTOSET_HEADING;

  // get robot position
  hposition_get_xy( htj->hrp, &robot);

  // set carrot position to current position
  setCarrotXYPosition( htj, robot);

  const autoset_configuration_t autoconf = AUTOSET_CONFIGURATIONS[robot_side];
  switch(table_side)
  {
    case TS_LEFT:
      htj->carrotA = autoconf.left;
      break;

    case TS_RIGHT:
      htj->carrotA = autoconf.right;
      break;

    case TS_UP:
      htj->carrotA = autoconf.up;
      break;

    case TS_DOWN:
      htj->carrotA = autoconf.down;
      break;

    default:
      return;
  }

  htrajectory_gotoA(htj, htj->carrotA);

  htj->autosetTargetX = x;
  htj->autosetTargetY = y;

  htj->autosetTableSide = table_side;
  htj->autosetRobotSide = robot_side;
}

void htrajectory_reset_carrot( htrajectory_t *htj )
{
  vect_xy_t robot;
  hposition_get_xy(htj->hrp, &robot);
  htj->carrot = robot;
}

