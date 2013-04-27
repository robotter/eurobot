/** @addtogroup clock
 *
 * @par Configuration
 *
 * Clock configuration shares definitions exported by \ref defs.h.
 *
 * Values are redundant, for instance \ref CLOCK_PRESCALER_A_DIV can
 * be deduced from \ref CLOCK_SYS_FREQ and \ref CLOCK_PER4_FREQ.
 * As a result, values are often optional but enough information must be
 * provided to have an unambiguous configuration.
 *
 * Variables defined manually are not overwritten. This is not an issue to
 * define more values than strictly needed, as long as they do not mismatch.
 *
 * Thorough checks are made to detect invalid configurations such as invalid
 * prescaler ratios, frequency roundings, incompatible values, etc.
 *
 * See documentation of each value to know when it must be defined.
 */
//@{
/** @file
 * @brief Clock configuration
 *
 * See \ref clock module description for configuration details.
 */
///@cond internal

#define CLOCK_SOURCE  CLOCK_SOURCE_XOSC
#define CLOCK_SOURCE_FREQ  16000000
#define CLOCK_CPU_FREQ  CLOCK_SOURCE_FREQ
#define CLOCK_PER2_FREQ  CLOCK_CPU_FREQ
#define CLOCK_PER4_FREQ  CLOCK_CPU_FREQ

///@endcond
//@}
