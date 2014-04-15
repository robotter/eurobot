#ifndef APDS9800_CONFIG_H
#define APDS9800_CONFIG_H

/* pin of the microcontroler connected to the Proximity sensor output of the APDS9800 */
#define APDS9800_PS_DOUT_PORT               PORTPIN(D,5u)

/* pin of the microcontroler that drives the IR led of the APDS9800 */ 
#define APDS9800_IR_LED_PWM_PORT               PORTPIN(D,3u)

/* pin of the microconroler that enables the proximity sensor circuit of the APDS9800 */
#define APDS9800_EN_PROXIMITY_SENSOR_PORT   PORTPIN(D,4u)

#define APDS9800_PROXIMITY_ANA_PORT         PORTPIN(A,1u)
#define APDS9800_AMBIANT_LIGHT_ANA_PORT      PORTPIN(A,0u)

/* frequency of the pwm that drives the proximity ir led */
#define APDS9800_IR_LED_FREQUENCY_HZ 10000u

/* maximal IR led pulse width in µs */
#define APDS9800_IR_LED_MAX_PW_us 120u

/* time between falling edge of enable and begining of pwm emission of IR led in µs */
#define APDS9800_ENABLE_TO_LED_ON_WAIT_DURATION_us  20u

/* default number of pulses that should toogle the APDS9800_PS_DOUT_PORT if an object is present
 * if APDS9800_PS_DOUT_PORT doesn't toogle no object is present 
 * must be lower than 16000*/
#define APDS9800_DEFAULT_OBJECT_DETECTION_PULSE_NB_THRESHOLD  100u

#endif //APDS9800_CONFIG_H
