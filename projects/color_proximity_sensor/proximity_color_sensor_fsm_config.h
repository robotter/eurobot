#ifndef PROXIMITY_COLOR_SENSOR_FSM_CONFIG_H
#define PROXIMITY_COLOR_SENSOR_FSM_CONFIG_H

/*
 * @brief PORT of the µC connected to the irq of the TCS37725 
 */
#define OBJECT_DETECTION_TIMEOUT_US         100000

/*
 * @brief PORT of the µC connected to the irq of the TCS37725 
 */
#define TCS37725_IRQ_PORT     PORTD

/*
 * @brief pin of the µC connected to the irq of the TCS37725 
 */
#define TCS37725_IRQ_PIN_NB   7

/*
 * @brief number of detection or not of object with the apds9800 before deciding if object is present or not
 */
#define APDS9800_OBJECT_PRESENCE_CHOICE_THRESHOLD   4


#define PCFSM_RED_LED_LVL     255u
#define PCFSM_GREEN_LED_LVL   255u
#define PCFSM_BLUE_LED_LVL    255u

// integration time in ms (between 2 and 614 ms)
#define TCS37725_INTEGRATION_TIME_MS    200

// number of pulses to detect object (between 1 and 255)
#define TCS37725_PULSE_NB               3

#define TCS37725_PROX_LOW_THRESHOLD           0
#define TCS37725_PROX_HI_THRESHOLD            1000
#define TCS37725_CONSECUTIVE_DETECT_THRESHOLD 5

#define TCS37725_IR_LED_CURRENT       CONTROL_IR_LED_50mA
#define TCS37725_RGBC_GAIN            CONTROL_RGBC_GAIN_1

#endif //PROXIMITY_COLOR_SENSOR_FSM_CONFIG_H
