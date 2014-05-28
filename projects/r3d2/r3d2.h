#ifndef R3D2_H__
#define R3D2_H__

#include <stdint.h>
#include "r3d2_config.h"


/// R3D2 configuration values
typedef struct {
  /// Motor speed (PWM value from 0 to 32167)
  uint16_t motor_speed;
  /// Motor burst speed, for startup (PWM value from 0 to 32167)
  uint16_t motor_burst;
  /// Motor stuck timeout, in capture counts
  uint8_t motor_timeout;
  /// Capture angle offset, in radians
  float angle_offset;
  /// Capture distance coefficient
  float dist_coef;
} r3d2_conf_t;


/// R3D2 object capture data
typedef struct {
  double angle;
  double dist;
} r3d2_object_t;

/// R3D2 capture data
typedef struct {
  uint8_t count;  ///< number of detected objects
  r3d2_object_t objects[R3D2_OBJECTS_MAX];  ///< detected objects
} r3d2_data_t;


/// Initialize R3D2 state
void r3d2_init(void);

/// Start R3D2 motor, enable sensor
void r3d2_start(void);
/// Stop R3D2 motor, disable sensor
void r3d2_stop(void);

/// Set R3D2 motor speed (PWM value from 0 to 32167)
void r3d2_set_motor_speed(uint16_t speed);

/// Get R3D2 configuration
const r3d2_conf_t *r3d2_get_conf(void);
/// Set R3D2 configuration
void r3d2_set_conf(const r3d2_conf_t *conf);

/** @brief Update capture data
 *
 * Process last capture information and update provided data.
 */
void r3d2_update(r3d2_data_t *data);

/** @brief Calibrate angle offset
 *
 * Set angle offset such as the first object as the given angle.
 * If no object is detected, do nothing.
 *
 * @note r3d2_update() is called.
 */
void r3d2_calibrate_angle(double angle);

/** @brief Calibrate distance coefficient
 *
 * Set distance coefficient such as the first object is at the given distance.
 * If no object is detected, do nothing.
 *
 * @note r3d2_update() is called.
 */
void r3d2_calibrate_dist(double dist);

/// Load R3D2 configuration from EEPROM
void r3d2_conf_load(void);
/// Save R3D2 configuration to EEPROM
void r3d2_conf_save(void);

/// Send R3D2 telemetry
void r3d2_telemetry(rome_intf_t *intf, const r3d2_data_t *data);

#endif
