#ifndef TCS3772X_DEFS_H
#define TCS3772X_DEFS_H


typedef enum
{
  ENABLE    = 0x00u,
  ATIME     = 0x01u,
  PTIME     = 0x02u,
  WTIME     = 0x03u,
  AILTL     = 0x04u,
  AILTH     = 0x05u,
  AIHTL     = 0x06u,
  AIHTH     = 0x07u,
  PILTL     = 0x08u,
  PILTH     = 0x09u,
  PIHTL     = 0x0Au,
  PIHTH     = 0x0Bu,
  PERS      = 0x0Cu,
  CONFIG    = 0x0Du,
  PPULSE    = 0x0Eu,
  CONTROL   = 0xF4u,
  ID        = 0x12u,
  STATUS    = 0x13u,
  CDATA     = 0x14u,
  CDATAH    = 0x15u,
  RDATA     = 0x16u,
  RDATAH    = 0x17u,
  GDATA     = 0x18u,
  GDATAH    = 0x19u,
  BDATA     = 0x1Au,
  BDATAH    = 0x1Bu,
  PDATA     = 0x1Cu,
  PDATAH    = 0x1Du,
}tcs3772x_Reg_t;

/// enum used in COMMAND register
// those bits integrate the CMD bit (bit 7) that must be set to enable COMMAND register 
typedef enum
{
  TYPE_REPEATED_BYTE_ACCESS         = 0x80u, //repeatedly read the same registe with each data access
  TYPE_AUTO_INCREMENT_BYTE_ACCESS   = 0xA0u, //auto increment pointer to read successives bytes
  TYPE_SPECIAL_FUNCTION             = 0xE0u, //enable use of special functions
}tcs3772x_CommandType_t;

/// enum used in COMMAND register
typedef enum
{
  SF_PROXIMITY_INTERRUPT_CLEAR      = 0x05u,
  SF_CLEAR_CHANNEL_INTERRUPT_CLEAR  = 0x06u,
  SF_PROX_AND_CLEAR_INTERRUPT_CLEAR = 0x07u,
}tcs3772x_CommandSpecialFunction_t;

typedef union
{
  uint8_t Value;
  tcs3772x_Reg_t Addr;
  tcs3772x_CommandSpecialFunction_t sf;
}tcs3772x_CommandAddrSF_t;

typedef enum
{
  ENABLE_PON  = 0x01u,
  ENABLE_AEN  = 0x02u,
  ENABLE_PEN  = 0x04u,
  ENABLE_WEN  = 0x08u,
  ENABLE_AIEN = 0x10u,
  ENABLE_PIEN = 0x20u,
}tcs3772x_EnableBitMask_t;

typedef enum
{
  ATIME_2_4_MS  = 0xFFu,
  ATIME_24_MS   = 0xF6u,
  ATIME_101_MS  = 0xFFu,
  ATIME_154_MS  = 0xADu,
  ATIME_614_MS  = 0x00u,

}tcs3772x_ATime_t;

typedef enum
{
  PTIME_2_4_MS  = 0xFFu,
  PTIME_204_MS  = 0xABu,
  PTIME_604_MS  = 0x00u,
}tcs3772x_WTime_t;

#define TCS3772x_PPERS_MASK 0xF0
#define TCS3772x_PPERS_bp   4u

typedef enum
{
  APERS_INTERRUPT_EVERY_CYCLE         = 0x00u,
  APERS_INTERRUPT_1_SAMPLE_OUT_RANGE  = 0x01u,
  APERS_INTERRUPT_2_SAMPLE_OUT_RANGE  = 0x02u,
  APERS_INTERRUPT_3_SAMPLE_OUT_RANGE  = 0x03u,
  APERS_INTERRUPT_5_SAMPLE_OUT_RANGE  = 0x04u,
  APERS_INTERRUPT_10_SAMPLE_OUT_RANGE = 0x05u,
  APERS_INTERRUPT_15_SAMPLE_OUT_RANGE = 0x06u,
  APERS_INTERRUPT_20_SAMPLE_OUT_RANGE = 0x07u,
  APERS_INTERRUPT_25_SAMPLE_OUT_RANGE = 0x08u,
  APERS_INTERRUPT_30_SAMPLE_OUT_RANGE = 0x09u,
  APERS_INTERRUPT_35_SAMPLE_OUT_RANGE = 0x0Au,
  APERS_INTERRUPT_40_SAMPLE_OUT_RANGE = 0x0Bu,
  APERS_INTERRUPT_45_SAMPLE_OUT_RANGE = 0x0Cu,
  APERS_INTERRUPT_50_SAMPLE_OUT_RANGE = 0x0Du,
  APERS_INTERRUPT_55_SAMPLE_OUT_RANGE = 0x0Eu,
  APERS_INTERRUPT_60_SAMPLE_OUT_RANGE = 0x0Fu,
}tcs3772x_APers_t;


typedef enum
{
  CONFIG_WLONG = 0x02u,
}tcs3772x_Config_t;


typedef enum
{
  CONTROL_IR_LED_100mA  = 0x00u,
  CONTROL_IR_LED_50mA   = 0x40u,
  CONTROL_IR_LED_25mA   = 0x80u,
  CONTROL_IR_LED_12_5mA = 0xC0u,
}tcs3772x_ControlIRLedCurrent_t;


typedef enum
{
  CONTROL_RGBC_GAIN_1   = 0x00u,
  CONTROL_RGBC_GAIN_4   = 0x01u,
  CONTROL_RGBC_GAIN_16  = 0x02u,
  CONTROL_RGBC_GAIN_60  = 0x03u,
}tcs3772x_ControlRGBCGain_t;


typedef enum
{
  ID_TCS37721_37725 = 0x40u,
  ID_TXS37723_37727 = 0x49u,
}tcs3772x_Id;


typedef enum
{
  STATUS_PROXIMITY_INTERRUPT_BM     = _BV(5),
  STATUS_CLEAR_CHANNEL_INTERRUPT_BM = _BV(4),
  STATUS_PROXIMITY_VALID_BM         = _BV(1),
  STATUS_RGBC_VALID_BM              = _BV(0),

}tcs3772x_Status_t;

#endif //TCS3772X_DEFS_H
