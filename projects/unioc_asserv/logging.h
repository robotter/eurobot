#ifndef LOGGING_H
#define LOGGING_H

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

typedef enum {
  LVLDEBUG = 0,
  LVLNOTICE,
  LVLWARNING,
  LVLERROR,
  LVLEMERG,
}logging_level_t;

#define _DWZ(x) do{x}while(0)

#define DEBUG(x, fmt, ...)   _DWZ(logging_report(LVLDEBUG, x, fmt, ##__VA_ARGS__);)
#define NOTICE(x, fmt, ...)  _DWZ(logging_report(LVLNOTICE, x, fmt, ##__VA_ARGS__);)
#define ERROR(x, fmt, ...)   _DWZ(logging_report(LVLERROR, x, fmt, ##__VA_ARGS__);)
#define WARNING(x, fmt, ...) _DWZ(logging_report(LVLWARNING, x, fmt, ##__VA_ARGS__);)
#define EMERG(x, fmt, ...)   _DWZ(logging_report(LVLEMERG, x, fmt, ##__VA_ARGS__);)


static inline void logging_report(logging_level_t lvl, int x, const char *fmt, ...) {
  const char *logging_level_strings[] = { "DEBUG", "NOTICE", "WARNING", "ERROR", "EMERG" };
  // unused
  (void)x;
  // print report type
  printf("%-7s",logging_level_strings[lvl]);
  // print formated text
  va_list argp;
  va_start(argp, fmt);
  vprintf(fmt,argp);
  va_end(argp);
  printf("\n");
}
#endif//LOGGING_H
