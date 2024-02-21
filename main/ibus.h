#ifndef IBUS_H
#define IBUS_H

#include "uart.h"

void ibus_init(ibus_t *rc);
void parse_ibus_data(uart_data_t *recv);


#endif