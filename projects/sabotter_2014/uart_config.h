/** @addtogroup uart */
//@{
/** @file
 * @brief UART configuration
 */
/** @name Configuration
 *
 * Unless otherwise noted, configuration values can be defined globally
 * (\c UART_* prefix) or specifically for each UARTxn (\c UARTxn_* prefix).
 *
 * UARTs must be explicitly enabled by defining \ref UARTxn_ENABLED.
 */
//@{

/// Buffer size for received data
#define UART_RX_BUF_SIZE  64
/// Buffer size for sent data
#define UART_TX_BUF_SIZE  64

/// Baudrate
#define UART_BAUDRATE  38400
/// Scale factor used to compute baudrate (from -7 to 7)
#define UART_BSCALE  0

/// Enable UARTxn
#define UARTC0_ENABLED
#define UARTD0_ENABLED
#define UARTE1_ENABLED
#define UARTF0_ENABLED

/** @brief Interrupt level (an \ref intlvl_t value)
 * @note Global configuration only.
 */
#define UART_INTLVL  INTLVL_HI

//@}
//@}
