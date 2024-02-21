#ifndef CONTROL_H
#define CONTROL_H

#include "typedefs.h"

//#define THROTTLE_MAX 2048
//#define THROTTLE_ZERO 1024
#define MAX_TARGET_THROTTLE 800
#define IDLE_THROTTLE 300

void control_init(ibus_t *rc, telemetry_t *tlm, flight_t *flt, target_t *trg, states_t *stt, config_t *cfg, waypoint_t *wp, gps_t *g);
void flight_mode_control();
void flight_control();

#endif