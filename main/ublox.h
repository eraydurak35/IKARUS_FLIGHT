#ifndef UBLOX_H
#define UBLOX_H

#include "typedefs.h"

void gps_init(gps_t *g);
void parse_gps_data(uart_data_t *data_buff);

#endif