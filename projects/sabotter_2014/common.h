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
// Get uptime value
uint32_t get_uptime_us(void);

// Robot state
typedef struct {
  // asserv, "in position" flags
  struct {
    bool xy:1;
    bool a:1;
    bool autoset:1;
  } asserv;
  // arm
  struct {
    int16_t shoulder;
    int16_t elbow;
    int16_t wrist;
  } arm;
  // suckers
  struct {
    bool a:1;
    bool b:1;
  } suckers;
} robot_state_t;

extern robot_state_t robot_state;


#endif
