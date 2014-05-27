#ifndef STRAT_H__
#define STRAT_H__

/// Team color (or side)
typedef enum {
  TEAM_NONE,
  TEAM_RED,
  TEAM_YELLOW,
} team_t;

void strat_init(void);
void strat_run(team_t team);
void strat_test(void);

#endif
