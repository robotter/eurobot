#ifndef ARMS_H
#define ARMS_H

#include <rome/rome.h>

typedef enum{
  ARM_INIT = 0,
  ARM_IDLE,

  ARM_RESET_ELEVATOR_START = 10,
  ARM_RESET_ELEVATOR_WAIT,

  ARM_ELEVATOR_GO_UP,
  ARM_ELEVATOR_WAIT_UP,

  ARM_ELEVATOR_GO_DOWN,
  ARM_ELEVATOR_WAIT_DOWN,

  ARM_TAKE_ATOMS = 20,

  ARM_RELEASE_ATOMS = 30,

  ARM_CHECK_SUCKERS = 40,
}arm_state_t;

typedef struct {
  uint8_t tm_state;
  arm_state_t state;
  bool up;
  bool atoms[3];
}arm_t;

extern arm_t arm_l;
extern arm_t arm_r;

void arm_take_atoms(arm_t arm);
void arm_release_atoms(arm_t arm);
void arm_elevator_up(arm_t arm);
void arm_elevator_down(arm_t arm);
uint8_t arms_get_tm_state(void);
void arms_init(void);
void arm_update(arm_t arm);
void arms_shutdown(void);

#endif//ARMS_H
