/** @addtogroup perlimpinpin */
//@{
/** @file
 * @brief Perlimpinpin configuration
 */
/** @name Configuration
 */
//@{


/** @brief Size of ppp_intf_rstate_t::payload
 *
 * If unset or null the feature is disabled, saving some place on the
 * ppp_intf_t structure.
 */
#define PPP_PAYLOAD_BUF_SIZE  32


/// Perlimpinpin node name
#define PPP_NODE_NAME  "R3D2"


/** @brief Support payload named \c X
 *
 * Valid payload names are defined in \ref payloads.h.
 *
 * Macro value must be empty.
 *
 * @note The \c SYSTEM payload is always defined.
 */
#define PPP_SUPPORT_ROOM


//@}
//@}
