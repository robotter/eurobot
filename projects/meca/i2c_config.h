/** @addtogroup i2c */
//@{
/** @file
 * @brief I2C configuration
 */
/** @name Configuration
 *
 * Unless otherwise noted, configuration values can be defined globally
 * (\c I2C_* prefix) or specifically for each TWIx (\c I2Cx_* prefix).
 *
 * I2Cs must be explicitly enabled by defining either \ref I2Cx_MASTER or
 * \ref I2Cx_SLAVE (not both).
 *
 * Some values are only used in master or slave mode.
 *
 * @warning Slave mode is not supported yet.
 */
//@{

/// Enable I2Cx in master mode
#define I2CC_MASTER

// Enable I2CE in master to control the servo hat
#define I2CE_MASTER
#define I2CE_INTLVL INTLVL_HI
#define I2CE_BAUDRATE	400000

/// Bus baudrate in Hz (master only)
#define I2C_BAUDRATE  100000

/// Interrupt level for I2C slave interrupts (an \ref intlvl_t value)
#define I2C_INTLVL  INTLVL_HI

//@}
//@}
