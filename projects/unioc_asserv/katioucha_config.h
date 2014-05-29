#ifndef KATIOUCHA_CONFIG_H
#define KATIOUCHA_CONFIG_H

#define KATIOUCHA_HIGH_PWM_PORT       TCF1
#define KATIOUCHA_HIGH_PWM_CHANNEL    'B'

#define KATIOUCHA_LOW_PWM_PORT       TCF1
#define KATIOUCHA_LOW_PWM_CHANNEL    'A'


                                          // REARM,   TUBE1,  TUBE2,  TUBE3
const int16_t katioucha_servo_position[8] = { 1200,   1700,   1800,   1900,
                                              3700,   3500,   3000,   2200};

/* servos low
 * min servo 1400
 * max servo 3700
 * 1200 servo rearm tubes
 * 1750 tube 1 fire
 * 1780 tube 1+2 fire
 * 1800 tube 1+2+3 fire 
 */

/* servos high
 * max servo 3700
 * 3700 servo rearm tubes
 * 3500 tube 1 fire
 * 3000 tube 1+2 fire
 * 2200 tube 1+2+3 fire 
 */

#endif //KATIOUCHA_CONFIG_H
