#ifndef PROXIMITY_COLOR_SENSOR_FSM_CONFIG_H
#define PROXIMITY_COLOR_SENSOR_FSM_CONFIG_H

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

#endif //PROXIMITY_COLOR_SENSOR_FSM_CONFIG_H
