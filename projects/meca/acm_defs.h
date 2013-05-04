#ifndef ACM_DEFS_H
#define ACM_DEFS_H

#define FIRST_LVL_CANDLE_NB 12
#define SECOND_LVL_CANDLE_NB 8
#define ACM_CAKE_ENCODER_RESOLUTION 12

typedef enum {
  ACM_BLUE = 0,
  ACM_RED,
} acm_color_t;

typedef enum {
  ACM_ARM_HOMED = 0,
  ACM_ARM_STALLING,
  ACM_ARM_ON_CAKE,
  ACM_ARM_ON_CAKE_BLOW_CANDLE,
  ACM_ARM_ON_CAKE_AVOID_CANDLE,
} acm_arm_config_t;

typedef enum {
  ACM_ARM_SECOND_LVL = 0,
  ACM_ARM_FIRST_LVL_LEFT,
  ACM_ARM_FIRST_LVL_RIGHT,
} acm_arm_t;

typedef enum {
  ACM_CANDLE_UNKNOWN = 0,
  ACM_CANDLE_WHITE,
  ACM_CANDLE_RED,
  ACM_CANDLE_BLUE,
} acm_candle_color_t;

#endif  //ACM_DEFS_H
