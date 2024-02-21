#ifndef NAV_COMM_H
#define NAV_COMM_H

#include "typedefs.h"

#define PIN_NUM_MISO 12
#define PIN_NUM_MOSI 13
#define PIN_NUM_CLK  14
#define PIN_NUM_CS   15

#define HEADER 0x69
#define FOOTER 0x31


void nav_comm_init(nav_data_t *nav, states_t *stt, range_finder_t *rng, flight_t *flt, config_t *cfg);
void master_send_recv_nav_comm(uint8_t *new_config_flag);





















#endif