#ifndef SENSOR_H
#define SENSOR_H

#define MAX_OBJECT 8

typedef enum {
  SENSOR_TOP,
} sensor_position_t;

void sensor_init(void);

void sensor_latch(void);
uint8_t sensor_get_object_number(sensor_position_t sid);
double sensor_get_object_angle(sensor_position_t sid, uint8_t id);
double sensor_get_object_distance(sensor_position_t sid, uint8_t id);
double sensor_get_period(sensor_position_t sid);
uint8_t sensor_new_data_available(void);

#endif// SENSOR_H
