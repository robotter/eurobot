/** @addtogroup bootloader */
//@{
/** @file
 * @brief Bootloader configuration
 */
/** @name Configuration
 *
 * For UART configuration, it is possible to use values from uart_config.h.
 */
//@{
#include "uart_config.h"

/// UART to use for bootloader
#define BOOTLOADER_UART  USARTxn

/// Baudrate
#define BOOTLOADER_UART_BAUDRATE  UART_BAUDRATE
/// Scale factor used to compute baudrate (from -6 to 7)
#define BOOTLOADER_UART_BSCALE  UART_BSCALE

/// Wait delay before running the application in milliseconds
#define BOOTLOADER_TIMEOUT  1000

/// Code to execute when the bootloader starts
#define BOOTLOADER_INIT_CODE
/// Code to execute before running the application
#define BOOTLOADER_BOOT_CODE

//@}
//@}
