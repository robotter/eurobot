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

typedef enum
{
  TRAJ_WITHOUT_MANAGER = 0,
  TRAJ_PANNING
}htrajectory_traj_type_t;

typedef enum
{
  PANNING_FSM_INIT = 0,
  PANNING_FSM_SET_ANGLE_SP,
  PANNING_FSM_ACQ,
  PANNING_FSM_CHECK_FOR_NEW_ANGLE_SP,
  PANNING_FSM_FINAL_ROTATION,
  PANNING_FSM_WAIT_FINAL_ROTATION_END,
  PANNING_FSM_FINISHED
}htrajectory_PANNING_t;

typedef struct
{
  struct
  {
    double x;
    double y;
    double a;
  }origin;

  struct
  {
    double x;
    double y;
    double a;
  }destination;
  htrajectory_traj_type_t traj_type;

  bool trajectory_finished;
  htrajectory_t *htj ;

  union
  {
    struct
    {
      void (*acq_fn)(double);
      int8_t rotation_direction;
      htrajectory_PANNING_t fsm_step;
      double panning_angle;
    }panning_data;
  }fsm_data;
}htrajectory_manager_t;

static htrajectory_manager_t htraj_man_data;
// avoidance system
extern avoidance_t avoidance;

/* -- private functions -- */

void htrajectory_panning_fsm( htrajectory_manager_t *man_data);
void htrajectory_manager(void);


/** \brief Compute normalized error vector to current point */
static inline vect_xy_t computeNormalizedError( vect_xy_t point, vect_xy_t carrot )
{ 
  vect_xy_t error;
  double errorLength;
  double dx,dy;

  dx = point.x - carrot.x;
  dy = point.y - carrot.y;

  // compute error vector length
  errorLength = sqrt( dx*dx + dy*dy );

  if( errorLength == 0.0 )
  {
    error.x = 0.0;
    error.y = 0.0;
  }
  else
  {
    // normalize error vector
    error.x = dx/errorLength;
    error.y = dy/errorLength;
  }

  return error;
}

/** \brief Prepare point */
static inline void preparePoint( htrajectory_t *htj )
{
  // compute and store normalized error vector
  htj->normalizedError = 
    computeNormalizedError(htj->path[htj->pathIndex],htj->carrot);
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

/** \brief Copy given path into internal structure */
static inline void copyPath( htrajectory_t *htj, vect_xy_t *path, uint8_t n)
{
  uint8_t it;

  if( n == 0 )
    return;

  if( n > HTRAJECTORY_MAX_POINTS )
    return;

  htj->pathSize = n;

  // copy points in internal structure
  for(it=0;it<n;it++)
    htj->path[it] = path[it];
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
  else
    // (squared inegality)
    // check if robot is in window  
    if( dx*dx + dy*dy < dl*dl )
      inWindow = 1;
    else
      inWindow = 0;

  return inWindow;
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

void htrajectory_run( htrajectory_t *htj, vect_xy_t *path, uint8_t n )
{
  // copy points to internal structure
  copyPath( htj, path, n );

  htj->pathIndex = 0;

  if(htj->pathSize == 1)
    htj->state = STATE_PATH_LAST;
  else
    htj->state = STATE_PATH_MID;

  preparePoint(htj);

  // reset carrot speed
  htj->carrotSpeed = 0;
  // set htj->carrot to first position
  setCarrotXYPosition( htj, htj->path[0] );

  return;
}

void htrajectory_gotoA( htrajectory_t *htj, double a )
{
  double robot_a;
  double da;

  // get robot angle
  hposition_get_a( htj->hrp, &robot_a );

  // compute distance between consign and position modulo 2pi
  da = fmod( a - robot_a, 2*M_PI );

  // update consign
  htj->carrotA = robot_a + da;
  htj->carrotA_reached = 0;

  // set robot carrot
  setCarrotAPosition(htj, htj->carrotA );
  htraj_man_data.htj = htj;
}

void htrajectory_gotoXY( htrajectory_t *htj, double x, double y)
{
  vect_xy_t path;

  // create a one point path
  path = (vect_xy_t){x,y};

  // load and run path
  htrajectory_run(htj, &path,  1);
  htraj_man_data.htj = htj;
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

void htrajectory_gotoXY_panning( htrajectory_t *htj, double dx, double dy, double panning_angle, void(*acq_callback)(double a))
{
  // get origin position
  vect_xy_t rp;
  hposition_get_xy( htj->hrp, &rp);
  htraj_man_data.htj = htj;
  htraj_man_data.origin.x = rp.x;
  htraj_man_data.origin.y = rp.y;
  hposition_get_a( htj->hrp, &htraj_man_data.origin.a);

  // compute destination position
  htraj_man_data.destination.x = htraj_man_data.origin.x + dx;
  htraj_man_data.destination.y = htraj_man_data.origin.y + dy;
  htraj_man_data.destination.x = dx;
  htraj_man_data.destination.y = dy;
  htraj_man_data.destination.a = htraj_man_data.origin.a;
  //htraj_man_data.destination.a = htraj_man_data.origin.a + da;
  // TODO : do modulo or not => RTFM or ask JD !!!
  htraj_man_data.trajectory_finished = false;
  // set 
  htraj_man_data.fsm_data.panning_data.acq_fn = acq_callback;
  htraj_man_data.fsm_data.panning_data.panning_angle = panning_angle;
  htraj_man_data.fsm_data.panning_data.fsm_step = PANNING_FSM_INIT;
  htraj_man_data.traj_type = TRAJ_PANNING;
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
      // set autoset to next step
      htj->state = STATE_AUTOSET_MOVE;

      // shutdown robot CSs
      robot_cs_activate(htj->rcs, 0);

      dx = SETTING_AUTOSET_SPEED*cos(-.5*M_PI);
      dy = SETTING_AUTOSET_SPEED*sin(-.5*M_PI);

      // store current robot position
      hposition_get(htj->hrp, &(htj->autosetInitPos));

      // set course
      hrobot_set_motors(dx, dy, 0.0);

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

      if( htj->autosetCount > SETTING_AUTOSET_ZEROCOUNT )
      {
        // autoset done
        switch(htj->autosetSide)
        {
          case TS_LEFT:
            htj->autosetInitPos.alpha = -M_PI_2;
            htj->autosetInitPos.x = htj->autosetTargetX;
            break;

          case TS_RIGHT:
            htj->autosetInitPos.alpha = +M_PI_2;
            htj->autosetInitPos.x = htj->autosetTargetX;
            break;

          case TS_UP:
            htj->autosetInitPos.alpha = +M_PI;
            htj->autosetInitPos.y = htj->autosetTargetY;
            break;

          case TS_DOWN:
            htj->autosetInitPos.alpha = 0;
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

    preparePoint(htj);    
  }

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
  else
    if( htj->state == STATE_PATH_LAST )
    {
      nextSpeed = htj->stopSpeed;
      currAcc = htj->stopAcc;
    }
    else
    {
      return;
    }

  // compute squared distance between carrot and target
  dx = point->x - htj->carrot.x;
  dy = point->y - htj->carrot.y;
  sqErrorLength = dx*dx + dy*dy;

  decDistance = 0.5*((htj->carrotSpeed)*(htj->carrotSpeed)
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

  // --- compute carrot position ---

  if( (htj->carrotSpeed)*(htj->carrotSpeed) > sqErrorLength )
  {
    htj->carrot.x = point->x;
    htj->carrot.y = point->y;
  }
  else
  {
    htj->carrot.x += (htj->carrotSpeed)*(htj->normalizedError.x);
    htj->carrot.y += (htj->carrotSpeed)*(htj->normalizedError.y);
  }

  // --- update carrot position ---
  setCarrotXYPosition( htj, htj->carrot );

}

void htrajectory_update( htrajectory_t *htj ) {
  //update trajectory manager
  htrajectory_manager();
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

void htrajectory_autoset( htrajectory_t *htj, tableSide_t side,
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

  switch(side)
  {
    case TS_LEFT:
      htj->carrotA = -M_PI_2;
      break;

    case TS_RIGHT:
      htj->carrotA = M_PI_2;
      break;

    case TS_UP:
      htj->carrotA = M_PI;
      break;

    case TS_DOWN:
      htj->carrotA = 0;
      break;

    default:
      return;
  }

  htrajectory_gotoA(htj, htj->carrotA);

  htj->autosetTargetX = x;
  htj->autosetTargetY = y;

  htj->autosetSide = side;
}

void htrajectory_reset_carrot( htrajectory_t *htj )
{
  vect_xy_t robot;
  hposition_get_xy(htj->hrp, &robot);
  htj->carrot = robot;
}

void htrajectory_manager(void)
{
  switch(htraj_man_data.traj_type)
  {
    case TRAJ_WITHOUT_MANAGER:
      if ((htrajectory_doneA(htraj_man_data.htj)) && 
          (htrajectory_doneXY(htraj_man_data.htj)) )
      {
        htraj_man_data.trajectory_finished = true;
      }
      else
      {
        htraj_man_data.trajectory_finished = false;
      }

      break;

    case TRAJ_PANNING :
      htrajectory_panning_fsm(&htraj_man_data);
      break;
    default : 
      break; 
  }
}

void htrajectory_panning_fsm( htrajectory_manager_t *man_data)
{
  /* if robot arrived to destination after panning, engage final rotation */
  if (htrajectory_doneXY(man_data->htj) && 
      (man_data->fsm_data.panning_data.fsm_step != PANNING_FSM_INIT) &&
      (man_data->fsm_data.panning_data.fsm_step != PANNING_FSM_WAIT_FINAL_ROTATION_END) &&
      (man_data->fsm_data.panning_data.fsm_step != PANNING_FSM_FINISHED))
  {
    man_data->fsm_data.panning_data.fsm_step = PANNING_FSM_FINAL_ROTATION;
  }
  
  switch( man_data->fsm_data.panning_data.fsm_step)
  {
    case PANNING_FSM_INIT : 
      // lauch robot xy move
      htrajectory_gotoXY(man_data->htj, man_data->destination.x, man_data->destination.y); 
      // first angle pan
      htrajectory_gotoA_R(man_data->htj, man_data->fsm_data.panning_data.panning_angle);
      man_data->fsm_data.panning_data.rotation_direction = 1;

      man_data->fsm_data.panning_data.fsm_step = PANNING_FSM_ACQ;
      break;

    case PANNING_FSM_SET_ANGLE_SP : 
      // inverse rotation direction
      man_data->fsm_data.panning_data.rotation_direction *= -1;
      // add or remove panning angle to robot angle (depending on the rotation direction)
      if ( man_data->fsm_data.panning_data.rotation_direction < 0)
      {
        htrajectory_gotoA_R(man_data->htj, -2.0 * man_data->fsm_data.panning_data.panning_angle);
      }
      else
      {
        htrajectory_gotoA_R(man_data->htj, 2.0 * man_data->fsm_data.panning_data.panning_angle);
      }
      man_data->fsm_data.panning_data.fsm_step = PANNING_FSM_ACQ;
      break;

    case PANNING_FSM_ACQ : 
      // execute acquisition callback 
      if (man_data->fsm_data.panning_data.acq_fn != NULL)
      {
        double a;
        hposition_get_a( man_data->htj->hrp, &a);
        (*(man_data->fsm_data.panning_data.acq_fn))(a);
      }
      man_data->fsm_data.panning_data.fsm_step = PANNING_FSM_CHECK_FOR_NEW_ANGLE_SP;
      // no break to continue with PANNING_FSM_CHECK_FOR_NEW_ANGLE_SP

    case PANNING_FSM_CHECK_FOR_NEW_ANGLE_SP :
      if (htrajectory_doneA(man_data->htj))
      {
        man_data->fsm_data.panning_data.fsm_step = PANNING_FSM_SET_ANGLE_SP;
      }
      else
      {
        man_data->fsm_data.panning_data.fsm_step = PANNING_FSM_ACQ;
      }
      break;

    case PANNING_FSM_FINAL_ROTATION:
      //initiate last rotation to reach final angle position
      htrajectory_gotoA(man_data->htj, man_data->destination.a);
      man_data->fsm_data.panning_data.fsm_step = PANNING_FSM_FINAL_ROTATION;
      break;
  
    case PANNING_FSM_WAIT_FINAL_ROTATION_END:
      if (htrajectory_doneA(man_data->htj))
      {
        man_data->fsm_data.panning_data.fsm_step = PANNING_FSM_FINISHED;
      }
      break;

    case PANNING_FSM_FINISHED :
      // indicate that the move is finished
      man_data->trajectory_finished = true;
      /* last call of this fsm function */
      man_data->traj_type = TRAJ_WITHOUT_MANAGER;
      break;

    default : break;
  }
}

