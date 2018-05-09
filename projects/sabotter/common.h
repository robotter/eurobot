#ifndef COMMON_H__
#define COMMON_H__

#include <stdint.h>

// Various common definitions including stuff defined in main.c

// ROME interfaces
extern rome_intf_t rome_asserv;
extern rome_intf_t rome_meca;
extern rome_intf_t rome_paddock;

// Handle input from all ROME interfaces
void update_rome_interfaces(void);

// R3D2 object detection state
typedef struct {
  float a;
  float r;
  bool detected;
} r3d2_object_t;

/// Team color (or side)
typedef enum {
  TEAM_NONE,
  TEAM_GREEN,
  TEAM_ORANGE,
} team_t;

// Robot state
typedef struct {
  team_t team;
  // asserv, "in position" flags
  struct {
    bool xy:1;
    bool a:1;
    bool autoset:1;
    uint8_t path_i;
    uint8_t path_n;
  } asserv;
  bool gyro_calibration;
  struct {
    int16_t x;
    int16_t y;
    float a;
  }current_pos;
  struct {
    int16_t x;
    int16_t y;
    float a;
  }partner_pos;
  struct {
    int16_t x;
    int16_t y;
  } carrot;
  // R3D2
  struct {
    r3d2_object_t objects[2];
  } r3d2;
  // MECA (galipeur)
  uint8_t meca_state;
  // cylinder
  uint8_t cylinder_nb_slots;
  uint8_t cylinder_nb_empty;
  uint8_t cylinder_nb_good;
  uint8_t cylinder_nb_bad;

  //galipette bumpers
  struct{
    bool left;
    bool right;
  }bumpers;

  //points count
  uint16_t points;
} robot_state_t;

extern robot_state_t robot_state;


#endif
