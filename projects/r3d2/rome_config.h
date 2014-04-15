/** @addtogroup rome */
//@{
/** @file
 * @brief ROME configuration
 */
/** @name Configuration
 */
//@{


/** @brief Disabled interrupt level when sending ROME frames
 *
 * If ROME frames are sent from the main and and an interrupt routine, frames
 * will be mixed up.
 * If this value is set, ROME frames can be safely sent from any interrupt of
 * configured (or lower) level.
 *
 * @sa ROME_SEND_INTLVL_DISABLE()
 */
#define ROME_SEND_INTLVL  INTLVL_LO

/// If defined, disable sending of messages X
#define ROME_DISABLE_X


//@}
//@}
