#ifndef KATIOUCHA_H
#define KATIOUCHA_H

#include <avarix.h>
#include <inttypes.h>


typedef enum{
KATIOUCHA_RECHARGE = 0u,     //position during projectile recharge 
KATIOUCHA_FIRE_TUBE_1,       // fire first ball
KATIOUCHA_FIRE_TUBE_2,       // fire first and second ball
KATIOUCHA_FIRE_ALL,          // fire all balls
} katioucha_pos_t;

typedef enum{
KATIOUCHA_LINE_LOW = 0u,
KATIOUCHA_LINE_HIGH,
}katioucha_line_t;


// initialize servos to recharge position
void katioucha_init(void);

// update position of servo line with preset values
void katioucha_set_position(katioucha_line_t line, katioucha_pos_t pos);

// update position of servo (apply pos directly to servo)
void katioucha_set_servo_position(katioucha_line_t line, int16_t pos);


#endif //KATIOUCHA_H
