/** @addtogroup xbee */
//@{
/** @file
 * @brief XBee API configuration
 */
/** @name Configuration
 */
//@{


/** @brief Disabled interrupt level when sending XBee frames
 *
 * If XBee frames are sent from the main and an interrupt routine, frames will
 * be mixed up.
 * If this value is set, XBee frames can be safely sent from any interrupt of
 * configured (or lower) level.
 *
 * @sa XBEE_SEND_INTLVL_DISABLE()
 */
#define XBEE_SEND_INTLVL  INTLVL_LO


//@}
//@}
