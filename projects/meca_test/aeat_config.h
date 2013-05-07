/** @addtogroup aeat */
//@{
/** @file
 * @brief AEAT encoders global configuration
 */
/** @name Global configuration
 */
//@{


/// SPI to use for AEAT encoders
#define AEAT_SPI  SPIE


#define AEAT_SPI_MOSI_PP PORTPIN(E,5)
#define AEAT_SPI_MISO_PP PORTPIN(E,6)
#define AEAT_SPI_SCK_PP  PORTPIN(E,7)
#define AEAT_SPI_SS_PP   PORTPIN(E,4)


/// SPI prescaler factor (2, 4, 8, 16, 32, 64 or 128)
#define AEAT_SPI_PRESCALER  16


//@}
//@}
