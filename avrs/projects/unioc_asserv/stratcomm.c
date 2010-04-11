/*  
 *  Copyright RobOtter (2009) 
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

/** \file stratcomm.c
  * \author JD
  */

#include <aversive/error.h>
#include <i2cs.h>
#include <string.h>

#include "stratcomm.h"
#include "stratcomm_payloads.h"
#include "stratcomm_orders.h"

#include "htrajectory.h"
#include "hposition_manager.h"

extern htrajectory_t trajectory;
extern hrobot_position_t position;

void stratcomm_init(stratcomm_t* sc)
{
  sc->payloadIt = 0;
  sc->returnPayloadIt = 0;
  return;
}

void stratcomm_update(stratcomm_t* sc)
{
  uint8_t payloadCRC,recvCRC;
  stratcommOrder_t order;

  // check if mailbox is full
  if(i2cs_state == I2C_RECEIVED)
  {
    // ------------------------------------------------------------
    // RECEIVE

    // -- ORDER --
    // first byte should be ORDER
    order = (stratcommOrder_t)i2cs_data[0];
    
    // -- PAYLOAD SIZE  --
    // second byte should be payload size
    sc->payloadSize = i2cs_data[1];
  
    // if payloadSize will overflow i2c buffer size
    if(sc->payloadSize > STRATCOMM_MAX_PAYLOAD_SIZE)
    {
      WARNING(STRATCOMM_ERROR,
        "i2c message payload size too big (payloadSize=%d, I2C_BUF_SIZE=%d\n",
        sc->payloadSize,I2C_BUF_SIZE); 
     
      i2cs_state = I2C_NONE;
      return;
    }
    
    // -- PAYLOAD CRC --
    // last byte should be CRC
    payloadCRC = i2cs_data[sc->payloadSize+2];
    
    // compute CRC
    recvCRC = stratcomm_computeChecksum(i2cs_data+2, sc->payloadSize);
    
    // in case of CRCs mismatch
    if( payloadCRC != recvCRC )
    {
      WARNING(STRATCOMM_ERROR,
        "i2c message CRC mismatch (payloadCRC=0x%2.2x recvCRC=0x%2.2x)",
        payloadCRC, recvCRC);
      
      i2cs_state = I2C_NONE;
      return;
    }
    
    DEBUG(0,"ORDER RCV : order=0x%2.2x psize=0x%2.2x crc=0x%2.2x",order,sc->payloadSize,recvCRC);

    // --
    stratcomm_resetPayload(sc);
    stratcomm_resetReturnPayload(sc);

    // perform ORDER
    stratcomm_doOrder(sc, order, i2cs_data+2);

    // ------------------------------------------------------------
    // SEND
    
    // 
    memset(i2cs_data, 0, I2C_BUF_SIZE);

    // size of returned payload
    i2cs_data[0] = sc->returnPayloadIt;

    // prepare returned payload
    memcpy(i2cs_data + 1, sc->returnPayload, sc->returnPayloadIt);

    // compute CRC and add it to payload
    payloadCRC =
      stratcomm_computeChecksum(sc->returnPayload, sc->returnPayloadIt);
    i2cs_data[ sc->returnPayloadIt + 2 ] = payloadCRC;

    // set i2c state machine to READY (data OK to be sent)
    i2cs_state = I2C_READY;
  }

  return;
}

void stratcomm_doOrder(stratcomm_t* sc,
                        stratcommOrder_t order,
                        uint8_t* payload)
{
  double x,y,a;
  vect_xy_t pxy;
  uint8_t b,n;

  switch(order)
  {
    //---------------------------------------------------------
    // NONE order
    case SO_NONE:
      DEBUG(0,"new order NONE received");
      break;

    //---------------------------------------------------------
    // order GOTO absolute XYA
    case SO_GOTO_A_XYA:

      // unpack payload
      x = UNPACK_DOUBLE(sc, payload);
      y = UNPACK_DOUBLE(sc, payload);
      a = UNPACK_DOUBLE(sc, payload);
      
      DEBUG(0,"new order GOTO_A_XYA(%3.3f,%3.3f,%3.3f) received", x,y,a);

      // send order to lower CSs
      htrajectory_gotoXY(&trajectory, x, y);
      htrajectory_gotoA(&trajectory, a);

      break;
    
    //---------------------------------------------------------
    // order GOTO relative XYA
    case SO_GOTO_R_XYA:

      // unpack payload
      x = UNPACK_DOUBLE(sc, payload);
      y = UNPACK_DOUBLE(sc, payload);
      a = UNPACK_DOUBLE(sc, payload);
      
      DEBUG(0,"new order GOTO_R_XYA(%3.3f,%3.3f,%3.3f) received", x,y,a);

      // send order to lower CSs
      htrajectory_gotoXY_R(&trajectory, x, y);
      htrajectory_gotoA_R(&trajectory, a);

      break;
  
    //---------------------------------------------------------
    // set checkpoint # in trajectory
    case SO_TRAJECTORY_SET_CHECKPOINT:

      // unpack payload
      n = UNPACK_UINT8(sc, payload);
      x = UNPACK_DOUBLE(sc, payload);
      y = UNPACK_DOUBLE(sc, payload);

      DEBUG(0,"new order TRAJECTORY_SET_CHECKPOINT (%u, %3.3f, %3.3f) received",
              n,x,y);
      
      // update checkpoints
      if(n >= HTRAJECTORY_MAX_POINTS)
        ERROR(STRATCOMM_ERROR, "Can't set point in stratcomm checkpoint list, point %d is out of bounds",
                n);

      sc->htrajectoryCheckpoints[n] = (vect_xy_t){x,y};

      break;

    //---------------------------------------------------------
    // run current trajectory
    case SO_TRAJECTORY_RUN:

      // unpack payload
      n = UNPACK_UINT8(sc, payload);

      DEBUG(0,"new order TRAJECTORY_RUN (%u) received", n);

      if(n >= HTRAJECTORY_MAX_POINTS)
        ERROR(STRATCOMM_ERROR, "Can't run trajectory on %d points, out of bounds", n);

      // perform order
      htrajectory_run(&trajectory, sc->htrajectoryCheckpoints, n);

      break;

    //---------------------------------------------------------
    // get trajectory manager status
    case SO_TRAJECTORY_STATUS:

      DEBUG(0,"new order TRAJECTORY_STATUS received");

      // no payload to unpack
      
      // get values from trajectory manager
      b = htrajectory_doneXY(&trajectory);
      // robot in XY position, boolean
      stratcomm_pushReturnPayload(sc, PACK_UINT8(b), sizeof(uint8_t));

      b = htrajectory_doneA(&trajectory);
      // robot in A position, boolean
      stratcomm_pushReturnPayload(sc, PACK_UINT8(b), sizeof(uint8_t));

      break;

    //---------------------------------------------------------
    case SO_SET_A_SPEED:
      ERROR(STRATCOMM_ERROR,"order SO_SET_A_SPEED not implemented");
      break;


    //---------------------------------------------------------
    case SO_SET_XY_CRUISE_SPEED:
      ERROR(STRATCOMM_ERROR,"order SO_SET_XY_CRUISE_SPEED not implemented");
      break;


    //---------------------------------------------------------
    case SO_SET_XY_STEERING_SPEED:
      ERROR(STRATCOMM_ERROR,"order SO_SET_XY_STEERING_SPEED not implemented");
      break;


    //---------------------------------------------------------
    case SO_SET_XY_STOP_SPEED:
      ERROR(STRATCOMM_ERROR,"order SO_SET_XY_STOP_SPEED not implemented");
      break;


    //---------------------------------------------------------
    case SO_SET_STEERING_WIN:
      ERROR(STRATCOMM_ERROR,"order SO_SET_STEERING_WIN not implemented");
      break;


    //---------------------------------------------------------
    case SO_SET_STOP_WIN:
      ERROR(STRATCOMM_ERROR,"order SO_SET_STOP_WIN not implemented");
      break;

    //---------------------------------------------------------
    // get robot XYA position
    case SO_GET_XYA:

      DEBUG(0,"new order GET_XYA received");

      // no payload to unpack

      // get values from CSs
      hposition_get_xy(&position, &pxy);
      hposition_get_a(&position, &a);
   
      // x position, double, in mm
      stratcomm_pushReturnPayload(sc, PACK_UINT8(pxy.x), sizeof(double));
      // y position, double, in mm
      stratcomm_pushReturnPayload(sc, PACK_UINT8(pxy.y), sizeof(double));
      // a position, double, in rads
      stratcomm_pushReturnPayload(sc, PACK_UINT8(a), sizeof(double));

      break;
    
    //---------------------------------------------------------
    // set robot XYA position
    case SO_SET_XYA:
      ERROR(STRATCOMM_ERROR, "order SO_SET_XYA not implemented");
      break;

    //---------------------------------------------------------
    default:
      WARNING(STRATCOMM_ERROR,"unrecognized order 0x%2.2x",order);
      break;
  }

  return;
}

uint8_t stratcomm_computeChecksum(uint8_t* payload, uint8_t payloadSize)
{
  uint8_t it;
  uint8_t crc = 0x00;

  for(it=0;it<payloadSize;it++)
    crc ^= payload[it]; 
  
  return crc;
}

void stratcomm_resetPayload(stratcomm_t *sc)
{
  // reset payload read pointer
  sc->payloadIt = 0;
}

uint8_t* stratcomm_popPayload(stratcomm_t* sc, uint8_t *p, uint8_t psize)
{
  uint8_t *pdata;

  if(sc->payloadIt + psize > sc->payloadSize)
    ERROR(STRATCOMM_ERROR,
      "can't pop value on payload, will overflow. (pit=%d ps=%d)",
      sc->payloadIt, psize);

  pdata = (p + sc->payloadIt);
  sc->payloadIt += psize;

  return pdata;
}

void stratcomm_resetReturnPayload(stratcomm_t *sc)
{
  // reset buffer write pointer
  sc->returnPayloadIt = 0;
}

void stratcomm_pushReturnPayload(stratcomm_t* sc, uint8_t* p, uint8_t psize )
{
  // if push will overflow return payload buffer
  if(sc->returnPayloadIt + psize > STRATCOMM_MAX_RPAYLOAD_SIZE)
    ERROR(STRATCOMM_ERROR,
      "can't push new value on return payload, will overflow. (rpit=%d psz=%d",sc->returnPayloadIt, psize);
 
  // add value to payload
  memcpy( sc->returnPayload + sc->returnPayloadIt, p, psize);

  // update payload iterator
  sc->returnPayloadIt += psize;

  return;
}
