#ifndef STRAT_H__
#define STRAT_H__

#include <stdbool.h>
#include "common.h"


/// Side for autoset
typedef enum {
  AUTOSET_NONE,
  AUTOSET_LEFT,
  AUTOSET_RIGHT,
  AUTOSET_UP,
  AUTOSET_DOWN,
} autoset_side_t;

bool starting_cord_plugged(void);

void strat_init(void);
team_t strat_select_team(void);
void strat_prepare(team_t team);
void strat_wait_start(team_t team);
void strat_run(team_t team);
void strat_test(team_t team);

#endif
