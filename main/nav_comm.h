#ifndef NAV_COMM_H
#define NAV_COMM_H

#include "typedefs.h"

#define PIN_NUM_MISO 12
#define PIN_NUM_MOSI 13
#define PIN_NUM_CLK  14
#define PIN_NUM_CS   15

#define SEND_HEADER_1 0x10
#define SEND_HEADER_2 0x11
#define SEND_HEADER_3 0x12
#define SEND_HEADER_4 0x13
#define FOOTER 0x31
#define RECV_HEADER 0x69

void nav_comm_init(nav_data_t *nav, states_t *stt, range_finder_t *rng, flight_t *flt, config_t *cfg);
uint8_t *master_send_recv_nav_comm(uint8_t send_data_type);
#endif