/*
 * Automatically generated by make menuconfig: don't edit
 */
#define AUTOCONF_INCLUDED

/*
 * Hardware
 */
#undef  CONFIG_MCU_AT90S2313
#undef  CONFIG_MCU_AT90S2323
#undef  CONFIG_MCU_AT90S3333
#undef  CONFIG_MCU_AT90S2343
#undef  CONFIG_MCU_ATTINY22
#undef  CONFIG_MCU_ATTINY26
#undef  CONFIG_MCU_AT90S4414
#undef  CONFIG_MCU_AT90S4433
#undef  CONFIG_MCU_AT90S4434
#undef  CONFIG_MCU_AT90S8515
#undef  CONFIG_MCU_AT90S8534
#undef  CONFIG_MCU_AT90S8535
#undef  CONFIG_MCU_AT86RF401
#undef  CONFIG_MCU_ATMEGA103
#undef  CONFIG_MCU_ATMEGA603
#undef  CONFIG_MCU_AT43USB320
#undef  CONFIG_MCU_AT43USB355
#undef  CONFIG_MCU_AT76C711
#undef  CONFIG_MCU_ATMEGA8
#undef  CONFIG_MCU_ATMEGA48
#undef  CONFIG_MCU_ATMEGA88
#undef  CONFIG_MCU_ATMEGA8515
#undef  CONFIG_MCU_ATMEGA8535
#undef  CONFIG_MCU_ATTINY13
#undef  CONFIG_MCU_ATTINY2313
#undef  CONFIG_MCU_ATMEGA16
#undef  CONFIG_MCU_ATMEGA161
#undef  CONFIG_MCU_ATMEGA162
#undef  CONFIG_MCU_ATMEGA163
#undef  CONFIG_MCU_ATMEGA165
#undef  CONFIG_MCU_ATMEGA168
#undef  CONFIG_MCU_ATMEGA169
#define CONFIG_MCU_ATMEGA32 1
#undef  CONFIG_MCU_ATMEGA323
#undef  CONFIG_MCU_ATMEGA325
#undef  CONFIG_MCU_ATMEGA3250
#undef  CONFIG_MCU_ATMEGA64
#undef  CONFIG_MCU_ATMEGA645
#undef  CONFIG_MCU_ATMEGA6450
#undef  CONFIG_MCU_ATMEGA128
#undef  CONFIG_MCU_ATMEGA1281
#undef  CONFIG_MCU_AT90CAN128
#undef  CONFIG_MCU_AT94K
#undef  CONFIG_MCU_AT90S1200
#undef  CONFIG_MCU_ATMEGA2560
#undef  CONFIG_MCU_ATMEGA256
#define CONFIG_QUARTZ (16000000)

/*
 * Generation options
 */
#undef  CONFIG_OPTM_0
#undef  CONFIG_OPTM_1
#undef  CONFIG_OPTM_2
#undef  CONFIG_OPTM_3
#define CONFIG_OPTM_S 1
#define CONFIG_MATH_LIB 1
#undef  CONFIG_FDEVOPEN_COMPAT
#undef  CONFIG_MINIMAL_PRINTF
#undef  CONFIG_STANDARD_PRINTF
#define CONFIG_ADVANCED_PRINTF 1
#define CONFIG_FORMAT_IHEX 1
#undef  CONFIG_FORMAT_SREC
#undef  CONFIG_FORMAT_BINARY

/*
 * Base modules
 */
#define CONFIG_MODULE_CIRBUF 1
#undef  CONFIG_MODULE_CIRBUF_LARGE
#undef  CONFIG_MODULE_FIXED_POINT
#undef  CONFIG_MODULE_VECT2
#undef  CONFIG_MODULE_SCHEDULER
#undef  CONFIG_MODULE_SCHEDULER_CREATE_CONFIG
#undef  CONFIG_MODULE_SCHEDULER_USE_TIMERS
#define CONFIG_MODULE_SCHEDULER_TIMER0 1
#undef  CONFIG_MODULE_SCHEDULER_MANUAL
#undef  CONFIG_MODULE_TIME
#undef  CONFIG_MODULE_TIME_CREATE_CONFIG
#undef  CONFIG_MODULE_TIME_EXT
#undef  CONFIG_MODULE_TIME_EXT_CREATE_CONFIG

/*
 * Communication modules
 */
#define CONFIG_MODULE_UART 1
#undef  CONFIG_MODULE_UART_9BITS
#define CONFIG_MODULE_UART_CREATE_CONFIG 1
#undef  CONFIG_MODULE_SPI
#undef  CONFIG_MODULE_SPI_CREATE_CONFIG
#undef  CONFIG_MODULE_I2C
#undef  CONFIG_MODULE_I2C_MASTER
#undef  CONFIG_MODULE_I2C_MULTIMASTER
#undef  CONFIG_MODULE_I2C_CREATE_CONFIG
#undef  CONFIG_MODULE_MF2_CLIENT
#undef  CONFIG_MODULE_MF2_CLIENT_USE_SCHEDULER
#undef  CONFIG_MODULE_MF2_CLIENT_CREATE_CONFIG
#undef  CONFIG_MODULE_MF2_SERVER
#undef  CONFIG_MODULE_MF2_SERVER_CREATE_CONFIG

/*
 * Hardware modules
 */
#undef  CONFIG_MODULE_TIMER
#undef  CONFIG_MODULE_TIMER_CREATE_CONFIG
#undef  CONFIG_MODULE_TIMER_DYNAMIC
#undef  CONFIG_MODULE_PWM
#undef  CONFIG_MODULE_PWM_CREATE_CONFIG
#undef  CONFIG_MODULE_PWM_NG
#define CONFIG_MODULE_ADC 1
#define CONFIG_MODULE_ADC_CREATE_CONFIG 1

/*
 * IHM modules
 */
#undef  CONFIG_MODULE_MENU
#undef  CONFIG_MODULE_VT100
#undef  CONFIG_MODULE_RDLINE
#undef  CONFIG_MODULE_RDLINE_CREATE_CONFIG
#undef  CONFIG_MODULE_RDLINE_KILL_BUF
#undef  CONFIG_MODULE_RDLINE_HISTORY
#undef  CONFIG_MODULE_PARSE
#undef  CONFIG_MODULE_PARSE_NO_FLOAT

/*
 * External devices modules
 */
#undef  CONFIG_MODULE_LCD
#undef  CONFIG_MODULE_LCD_CREATE_CONFIG
#undef  CONFIG_MODULE_MULTISERVO
#undef  CONFIG_MODULE_MULTISERVO_CREATE_CONFIG
#undef  CONFIG_MODULE_AX12
#undef  CONFIG_MODULE_AX12_CREATE_CONFIG

/*
 * Brushless motor drivers (you should enable pwm modules to see all)
 */
#undef  CONFIG_MODULE_BRUSHLESS_3PHASE_DIGITAL_HALL
#undef  CONFIG_MODULE_BRUSHLESS_3PHASE_DIGITAL_HALL_CREATE_CONFIG
#undef  CONFIG_MODULE_BRUSHLESS_3PHASE_DIGITAL_HALL_DOUBLE
#undef  CONFIG_MODULE_BRUSHLESS_3PHASE_DIGITAL_HALL_DOUBLE_CREATE_CONFIG

/*
 * Encoders (you need comm/spi for encoders_spi)
 */
#undef  CONFIG_MODULE_ENCODERS_MICROB
#undef  CONFIG_MODULE_ENCODERS_MICROB_CREATE_CONFIG
#undef  CONFIG_MODULE_ENCODERS_EIRBOT
#undef  CONFIG_MODULE_ENCODERS_EIRBOT_CREATE_CONFIG
#undef  CONFIG_MODULE_ENCODERS_SPI
#undef  CONFIG_MODULE_ENCODERS_SPI_CREATE_CONFIG

/*
 * Robot specific modules
 */
#undef  CONFIG_MODULE_ROBOT_SYSTEM
#undef  CONFIG_MODULE_ROBOT_SYSTEM_MOT_AND_EXT
#undef  CONFIG_MODULE_POSITION_MANAGER
#undef  CONFIG_MODULE_TRAJECTORY_MANAGER
#undef  CONFIG_MODULE_BLOCKING_DETECTION_MANAGER
#undef  CONFIG_MODULE_OBSTACLE_AVOIDANCE

/*
 * Control system modules
 */
#undef  CONFIG_MODULE_CONTROL_SYSTEM_MANAGER
#undef  CONFIG_MODULE_PID
#undef  CONFIG_MODULE_PID_CREATE_CONFIG
#undef  CONFIG_MODULE_RAMP
#undef  CONFIG_MODULE_QUADRAMP
#undef  CONFIG_MODULE_QUADRAMP_DERIVATE
#undef  CONFIG_MODULE_BIQUAD
#undef  CONFIG_MODULE_QUADRAMP_DERIVATE
#undef  CONFIG_MODULE_BIQUAD

/*
 * Radio devices
 */
#undef  CONFIG_MODULE_CC2420
#undef  CONFIG_MODULE_CC2420_CREATE_CONFIG

/*
 * Crypto modules
 */
#undef  CONFIG_MODULE_AES
#undef  CONFIG_MODULE_AES_CTR
#undef  CONFIG_MODULE_MD5
#undef  CONFIG_MODULE_MD5_HMAC
#undef  CONFIG_MODULE_RC4

/*
 * Encodings modules
 */
#undef  CONFIG_MODULE_BASE64
#undef  CONFIG_MODULE_HAMMING

/*
 * Debug modules
 */
#undef  CONFIG_MODULE_DIAGNOSTIC
#undef  CONFIG_MODULE_DIAGNOSTIC_CREATE_CONFIG
#define CONFIG_MODULE_ERROR 1
#define CONFIG_MODULE_ERROR_CREATE_CONFIG 1

/*
 * Programmer options
 */
#define CONFIG_AVRDUDE 1
#undef  CONFIG_AVARICE

/*
 * Avrdude
 */
#undef  CONFIG_AVRDUDE_PROG_FUTURELEC
#undef  CONFIG_AVRDUDE_PROG_ABCMINI
#undef  CONFIG_AVRDUDE_PROG_PICOWEB
#undef  CONFIG_AVRDUDE_PROG_SP12
#undef  CONFIG_AVRDUDE_PROG_ALF
#undef  CONFIG_AVRDUDE_PROG_BASCOM
#undef  CONFIG_AVRDUDE_PROG_DT006
#undef  CONFIG_AVRDUDE_PROG_PONY_STK200
#define CONFIG_AVRDUDE_PROG_STK200 1
#undef  CONFIG_AVRDUDE_PROG_PAVR
#undef  CONFIG_AVRDUDE_PROG_BUTTERFLY
#undef  CONFIG_AVRDUDE_PROG_AVR910
#undef  CONFIG_AVRDUDE_PROG_STK500
#undef  CONFIG_AVRDUDE_PROG_AVRISP
#undef  CONFIG_AVRDUDE_PROG_BSD
#undef  CONFIG_AVRDUDE_PROG_DAPA
#undef  CONFIG_AVRDUDE_PROG_JTAG1
#undef  CONFIG_AVRDUDE_PROG_AVR109
#define CONFIG_AVRDUDE_PORT "/dev/parport0"

/*
 * Avarice
 */
#define CONFIG_AVARICE_PORT "/dev/ttyS0"
#define CONFIG_AVARICE_DEBUG_PORT (1234)
#define CONFIG_AVARICE_PROG_MKI 1
#undef  CONFIG_AVARICE_PROG_MKII

/*
 * RobOtter modules
 */
#undef  CONFIG_MODULE_ERROR_ROBOTTER
#undef  CONFIG_MODULE_ERROR_ROBOTTER_CREATE_CONFIG
#undef  CONFIG_MODULE_ADNS6010_ROBOTTER
#undef  CONFIG_MODULE_ADNS6010_ROBOTTER_CREATE_CONFIG
#undef  CONFIG_MODULE_ADNS9500_ROBOTTER
#undef  CONFIG_MODULE_GP2PACK_ROBOTTER
#undef  CONFIG_MODULE_GP2PACK_ROBOTTER_CREATE_CONFIG
#undef  CONFIG_MODULE_I2C_ROBOTTER
#undef  CONFIG_MODULE_I2C_ROBOTTER_CREATE_CONFIG
#undef  CONFIG_MODULE_MATCHUART_ROBOTTER
#undef  CONFIG_MODULE_PERLIMPINPIN
#undef  CONFIG_MODULE_PERLIMPINPIN_CREATE_CONFIG
#undef  CONFIG_MODULE_AX12_ROBOTTER
#undef  CONFIG_MODULE_AX12_ROBOTTER_CREATE_CONFIG
