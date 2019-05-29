#ifndef COMMON_H__
#define COMMON_H__

#include <xbee/xbee.h>
#include <stdint.h>

// Various common definitions including stuff defined in main.c

// ROME interfaces
extern xbee_intf_t xbee_paddock;
#define ROME_DST_ASSERV  ROME_ASSERV_UART
#define ROME_DST_MECA  ROME_MECA_UART
#define ROME_DST_XBEE(addr)  ROME_XBEE_DST(&xbee_paddock, (addr))
#define ROME_DST_BROADCAST  ROME_DST_XBEE(XBEE_BROADCAST)
#define ROME_DST_PADDOCK  ROME_DST_XBEE(0x7061)

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
  TEAM_YELLOW,
  TEAM_PURPLE,
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
  } current_pos;
  struct {
    int16_t x;
    int16_t y;
    float a;
  } partner_pos;
  struct {
    int16_t x;
    int16_t y;
  } carrot;

  // R3D2
  struct {
    r3d2_object_t objects[2];
  } r3d2;

#if (defined GALIPEUR)
  // MECA (galipeur)
  rome_enum_meca_state_t meca_state;
  // atoms manipulators
  int16_t left_pos;  // -1 if unknown
  int16_t right_pos;  // -1 if unknown
  bool left_atoms[3];
  bool right_atoms[3];

#elif (defined GALIPETTE)
  // galipette bumpers
  struct {
    bool left;
    bool right;
  } bumpers;
#endif

  // points count
  uint16_t points;

  // boomotter
  uint32_t boom_age;

  // match timer
  uint8_t match_time;
} robot_state_t;

extern robot_state_t robot_state;

#endif
