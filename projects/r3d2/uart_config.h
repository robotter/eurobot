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
#define UART_RX_BUF_SIZE  128
/// Buffer size for sent data
#define UART_TX_BUF_SIZE  128

/// Baudrate
#define UART_BAUDRATE  38400
/// Scale factor used to compute baudrate (from -7 to 7)
#define UART_BSCALE  0

/// Enable UARTxn
#define UARTF1_ENABLED


// We need motor and sensor interrupts to have a higher level
#define UART_INTLVL  INTLVL_MED

//@}
//@}
