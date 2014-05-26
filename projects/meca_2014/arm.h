#ifndef _ARM_H_
#define _ARM_H_

#include <stdint.h>
#include <stdbool.h>

typedef enum {
  A_UPPER,
  A_ELBOW,
  A_WRIST,
}arm_part_t;

typedef struct {
  int32_t upper;
  int16_t elbow;
  int16_t wrist;
}arm_debug_t;

/** @brief Initialize arm */
void arm_init(void);

/** @brief Set arm position */
void arm_set_position(arm_part_t arm, int32_t position);

/** @brief Start arm calibration */
void arm_start_calibration(void);

/** @brief Return TRUE if arm is running and can be used */
uint8_t arm_is_running(void);

/** @brief Update module, shall be called periodically */
void arm_update(void);

/** @brief Activate/deactivate power */
void arm_activate_power(bool);

/** @brief Fill debug structure */
void arm_get_debug(arm_debug_t*);

#endif/*_ARM_H_*/
