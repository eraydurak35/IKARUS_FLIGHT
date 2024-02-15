#ifndef NAV_COMM_H
#define NAV_COMM_H

#include "typedefs.h"

void nav_comm_init(states_t *std, flight_t *flt, range_finder_t *rng, config_t *cfg, data_1_t *d1, data_2_t *d2, data_3_t *d3);
void parse_nav_data(uart_data_t *uart_buff);
void send_range();
void send_flight_status();
void send_nav_config();

#endif