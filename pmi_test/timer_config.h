/** @addtogroup timer */
//@{
/** @file
 * @brief Timer configuration
 */
/** @name Configuration
 *
 * Unless otherwise noted, configuration values can be defined globally
 * (\c TIMER_* prefix) or specifically for each TIMERxn (\c TIMERxn_* prefix).
 *
 * Timers must be explicitly enabled by defining \ref TIMERxn_ENABLED.
 */
//@{

/** @brief Timer prescaler factor
 *
 * This value directly affects the tick period.
 *
 * Valid values are 1, 2, 4, 8, 64, 256 and 1024.
 */
#define TIMER_PRESCALER_DIV  64

/// Enable TIMERxn
#define TIMERxn_ENABLED

//@}
//@}
