#ifndef PUMP_VALVES_H
#define PUMP_VALVES_H

#define PUMPA_PP PORTPIN(D,0)
#define PUMPB_PP PORTPIN(D,1)
#define VALVEA_PP PORTPIN(D,2)
#define VALVEB_PP PORTPIN(C,4)

static inline void pump_valves_init(void) {
  portpin_dirset(&PUMPA_PP);
  portpin_outclr(&PUMPA_PP);

  portpin_dirset(&PUMPB_PP);
  portpin_outclr(&PUMPB_PP);

  portpin_dirset(&VALVEA_PP);
  portpin_outset(&VALVEA_PP);

  portpin_dirset(&VALVEB_PP);
  portpin_outset(&VALVEB_PP);
}

static inline void _activate(portpin_t *pp, bool b) {
  if(b)
    portpin_outset(pp);
  else
    portpin_outclr(pp);
}

static inline void pump_valves_activate_pump_A(bool b) { _activate(&PUMPA_PP,b); };
static inline void pump_valves_activate_pump_B(bool b) { _activate(&PUMPB_PP,b); };
static inline void pump_valves_activate_valve_A(bool b) { _activate(&VALVEA_PP,b); };
static inline void pump_valves_activate_valve_B(bool b) { _activate(&VALVEB_PP,b); };

#endif//PUMP_VALVES_H
