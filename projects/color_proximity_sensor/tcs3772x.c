#include <i2c/i2c.h>
#include "tcs3772x.h"
#include "tcs3772x_defs.h"


#define TCS3772x_FRAME_LENGTH 32u;

/************* Local functions declaration ************/
static void tcs3772x_EncodeFrameHeader(uint8_t *Packet, uint8_t IsSpecialFunction, tcs3772x_CommandAddrSF_t AddrSf);

static int8_t tcs3772x_UpdateRegister8Bits(tcs3772x_t *Sensor, tcs3772x_Reg_t Addr, uint8_t Value); 

static int8_t tcs3772x_UpdateRegister16Bits(tcs3772x_t *Sensor, tcs3772x_Reg_t Addr, uint16_t Value);

static int8_t tcs3772x_ReadRegister8Bits(tcs3772x_t *Sensor, tcs3772x_Reg_t Addr, uint8_t *Value); 

static int8_t tcs3772x_ReadRegister16Bits(tcs3772x_t *Sensor, tcs3772x_Reg_t Addr, uint16_t *Value);

/* CyclePerformed set to 1 if RGBC acquisition cycle performed else set to 0*/
static int8_t tcs3772x_IsRGBCCyclePerformed(tcs3772x_t *Sensor, uint8_t *CyclePerformed);
/************* Local functions ************/

static void tcs3772x_EncodeFrameHeader(uint8_t *Packet, uint8_t IsSpecialFunction, tcs3772x_CommandAddrSF_t AddrSf)
{
  if (0x00u == IsSpecialFunction)
  {
    Packet[0u] = TYPE_SPECIAL_FUNCTION | AddrSf.Value;
  }
  else
  {
    Packet[0u] = TYPE_AUTO_INCREMENT_BYTE_ACCESS | AddrSf.Value;
  }
}

static int8_t tcs3772x_UpdateRegister8Bits(tcs3772x_t *Sensor, tcs3772x_Reg_t Addr, uint8_t Value)
{
  uint8_t Payload[2u];

  uint8_t PayloadLength = sizeof(Payload) * sizeof(uint8_t);

  tcs3772x_CommandAddrSF_t AddrSf = {.Addr = Addr};

  /* format payload header */
  tcs3772x_EncodeFrameHeader(Payload, 0x00u,AddrSf );

  /* set register value */
  Payload[1u] = Value;

  /* send it to the i2c bus */
  return (PayloadLength == i2cm_send(Sensor->I2c, Sensor->I2cAddress, Payload, PayloadLength)); 
}

static int8_t tcs3772x_UpdateRegister16Bits(tcs3772x_t *Sensor, tcs3772x_Reg_t Addr, uint16_t Value)
{
  uint8_t Payload[3u];

  uint8_t PayloadLength = sizeof(Payload) * sizeof(uint8_t);

  tcs3772x_CommandAddrSF_t AddrSf = {.Addr = Addr};

  /* format payload header */
  tcs3772x_EncodeFrameHeader(Payload, 0x00u, AddrSf);

  /* set register value */
  Payload[1u] = (uint8_t)(Value & 0x00ffu);
  Payload[2u] = (uint8_t)((Value >>8) & 0x00ffu);

  /* send it to the i2c bus */
  return (PayloadLength == i2cm_send(Sensor->I2c, Sensor->I2cAddress, Payload, PayloadLength)); 
}

static int8_t tcs3772x_ReadRegister8Bits(tcs3772x_t *Sensor, tcs3772x_Reg_t Addr, uint8_t *Value)
{
  uint8_t Payload[1u];
  uint8_t PayloadLength = sizeof(Payload) * sizeof(uint8_t);
  
  tcs3772x_CommandAddrSF_t AddrSf = {.Addr = Addr};
  
  /* format payload header */
  tcs3772x_EncodeFrameHeader(Payload, 0x00u, AddrSf);
  
  /* send next address to be accessed */
  if (PayloadLength == i2cm_send(Sensor->I2c, Sensor->I2cAddress, Payload, PayloadLength)) 
  {
    /* echange of data complete => continue with read */

    if (PayloadLength == i2cm_recv(Sensor->I2c, Sensor->I2cAddress, Payload, PayloadLength)) 
    {
      if (Value)
      {
        *Value = Payload[0];
        return 1;
      }
    }
  }
    return 0; 
}


static int8_t tcs3772x_ReadRegister16Bits(tcs3772x_t *Sensor, tcs3772x_Reg_t Addr, uint16_t *Value)
{
  uint8_t Payload[2u];
  uint8_t PayloadLength = 1u;
  
  tcs3772x_CommandAddrSF_t AddrSf = {.Addr = Addr};
  
  /* format payload header */
  tcs3772x_EncodeFrameHeader(Payload, 0x00u, AddrSf);
  
  /* send next address to be accessed */
  if (PayloadLength == i2cm_send(Sensor->I2c, Sensor->I2cAddress, Payload, PayloadLength)) 
  {
    /* echange of data complete => continue with read */
    PayloadLength = sizeof(Payload) * sizeof(uint8_t);

    if (PayloadLength == i2cm_recv(Sensor->I2c, Sensor->I2cAddress, Payload, PayloadLength)) 
    {
      if (Value)
      {
        *Value = 0;
        *Value = (((uint16_t)Payload[1]<<8)&0xFF00u);
        *Value |= (((uint16_t)Payload[0])&0x00FFu);
        return 1;
      }
    }
  }
  return 0; 
}

static int8_t tcs3772x_IsRGBCCyclePerformed(tcs3772x_t *Sensor, uint8_t *CyclePerformed)
{
  uint8_t Status = 0;
  *CyclePerformed = 0;
  if (tcs3772x_ReadRegister8Bits(Sensor, STATUS, &Status))
  {
    if (Status & STATUS_RGBC_VALID_BM)
    {
      *CyclePerformed = 1;
    }
    return 1;
  }
  return 0; // error during communication
}
/************* Global functions ************/

int8_t tcs3772x_init(tcs3772x_t *Sensor, i2cm_t *i2c, uint16_t tcs3772x_model )
{
  /* configure i2c address and check model of component */
  switch(tcs3772x_model)
  {
    case 37721 : 
    case 37723 : 
      Sensor->I2cAddress = 0x39;
      break;
    case 37725 : 
    case 37727 : 
      Sensor->I2cAddress = 0x29;
      break;
    default : return -2;
  }

  Sensor->I2c = i2c;

  /* initialise i2c  */
  i2c_init();

  return tcs3772x_Disable(Sensor);
}

int8_t tcs3772x_RGBCSetIntegrationTime_ms(tcs3772x_t *Sensor, uint16_t IntegrationTime_ms)
{
  /* check IntegrationTime_ms is in correct range */
  if ((IntegrationTime_ms < 3) ||
      (IntegrationTime_ms > 614))
  {
    return -2;
  }
  else
  {
    /* every parameter must be multiplied by 10 to compute division by 2.4 ms */
    uint16_t RegMult10 = 0;

    /* division by 24 instead of 2.4ms because every parameter is pultiplied by 10*/
    RegMult10 = 2560u - (IntegrationTime_ms*10u)/24u;

    uint8_t Reg = (uint8_t)(RegMult10 /10);

    if (tcs3772x_UpdateRegister8Bits(Sensor, ATIME, Reg))
    {
      return 0;
    }
    else
    {
      return -1;
    }
  }
}

int8_t tcs3772x_ProximitySetPulseNb(tcs3772x_t *Sensor, uint8_t ProximityPulseNb)
{
  /* check IntegrationTime_ms is in correct range */
  if (ProximityPulseNb == 0)
  {
    return -2;
  }
  else
  {
    if (tcs3772x_UpdateRegister8Bits(Sensor, PPULSE, ProximityPulseNb))
    {
      return 0;
    }
    else
    {
      return -1;
    }
  }
}

int8_t tcs3772x_Enable(tcs3772x_t *Sensor)
{
  if (tcs3772x_UpdateRegister8Bits(Sensor, ENABLE, (uint8_t)ENABLE_PON))
  {
    return 0;
  }
  else
  {
    return -1;
  }
}

int8_t tcs3772x_Disable(tcs3772x_t *Sensor)
{
  if (tcs3772x_UpdateRegister8Bits(Sensor, ENABLE, 0x00u))
  {
    return 0;
  }
  else
  {
    return -1;
  }
}

int8_t tcs3772x_ProximityEnableDetection(tcs3772x_t *Sensor)
{
  if (tcs3772x_UpdateRegister8Bits(Sensor, ENABLE, (uint8_t)ENABLE_PON|ENABLE_PEN))
  {
    return 0;
  }
  else
  {
    return -1;
  }
}

int8_t tcs3772x_ProximityDisableDetection(tcs3772x_t *Sensor)
{
  return tcs3772x_Disable(Sensor);
}

int8_t tcs3772x_ProximityEnableInterrupt(tcs3772x_t *Sensor)
{
  if (tcs3772x_UpdateRegister8Bits(Sensor, ENABLE, (uint8_t)ENABLE_PON|ENABLE_PEN|ENABLE_PIEN))
  {
    return 0;
  }
  else
  {
    return -1;
  }
}

int8_t tcs3772x_ProximityDisableInterrupt(tcs3772x_t *Sensor)
{
  return tcs3772x_Disable(Sensor);
}


int8_t tcs3772x_ProximityClearInterrupt(tcs3772x_t *Sensor)
{
  uint8_t Payload = 0x00u;
  tcs3772x_CommandAddrSF_t AddrSf = {
    .sf = SF_PROXIMITY_INTERRUPT_CLEAR
  };

  tcs3772x_EncodeFrameHeader(&Payload, 0x01,AddrSf);
  
  return (1u == i2cm_send(Sensor->I2c, Sensor->I2cAddress, &Payload, 1u)); 
}

int8_t tcs3772x_RGBCGetValue(tcs3772x_t *Sensor, tcs3772x_ColorResult_t *result)
{
  /* disable poximity sensor and enable color sensor*/
  if (tcs3772x_UpdateRegister8Bits(Sensor, ENABLE, (uint8_t)ENABLE_PON|ENABLE_AEN))
  {
  
  /* wait for color acquisition cycle to be performed */
    uint8_t RGBCConversionPerformed = 0;
    do{
      tcs3772x_IsRGBCCyclePerformed(Sensor, &RGBCConversionPerformed);
    } while (RGBCConversionPerformed == 0);

    int8_t CdataResult = tcs3772x_ReadRegister16Bits(Sensor,CDATA, &(result->clear_value));
    int8_t RdataResult = tcs3772x_ReadRegister16Bits(Sensor,RDATA, &(result->red_value));
    int8_t GdataResult = tcs3772x_ReadRegister16Bits(Sensor,GDATA, &(result->green_value));
    int8_t BdataResult = tcs3772x_ReadRegister16Bits(Sensor,BDATA, &(result->blue_value));

    if ((CdataResult != 0) &&
        (RdataResult != 0) &&
        (GdataResult != 0) &&
        (BdataResult != 0))
    {
      return tcs3772x_ProximityEnableDetection(Sensor);
    }
  }
  return -1;
}


int8_t tcs3772x_SetProximityInterruptThreshold(tcs3772x_t *Sensor, uint16_t LowThreshold, uint16_t HighThreshold, uint8_t ConsecutiveMeasOutRangeThreshold)
{
  if ((ConsecutiveMeasOutRangeThreshold <1) || (ConsecutiveMeasOutRangeThreshold > 15u))
  {
    return -2;
  }
  else
  {

    int8_t PiltlReturn = tcs3772x_UpdateRegister16Bits(Sensor, PILTL, LowThreshold);

    int8_t PihtlReturn =  tcs3772x_UpdateRegister16Bits(Sensor, PIHTL, HighThreshold);

    uint8_t PersValue = (ConsecutiveMeasOutRangeThreshold << TCS3772x_PPERS_bp) & TCS3772x_PPERS_MASK;
    int8_t PersReturn = tcs3772x_UpdateRegister8Bits(Sensor, PERS, PersValue);

    if ((PiltlReturn!=0) &&
        (PihtlReturn != 0) &&
        (PersReturn != 0))
    {
      return 0;
    }
    else 
    {
      return -1;
    }
  }
}
