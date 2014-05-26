/** @addtogroup adxrs */
//@{
/** @file
 * @brief ADXRS gyro global configuration
 */
/** @name Global configuration
 */
//@{

/** @brief Use SPIx for ADXRS gyro
 * @note Only one SPI can be enabled.
 */
#define ADXRS_SPID_ENABLE

/// SPI prescaler factor (2, 4, 8, 16, 32, 64 or 128)
#define ADXRS_SPI_PRESCALER  128

/// Interrupt level for SPI capture (an \ref intlvl_t value)
#define ADXRS_CAPTURE_INTLVL  INTLVL_HI

//@}
//@}
