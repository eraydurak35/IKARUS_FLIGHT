#ifndef BLACKBOX_H
#define BLACKBOX_H

#include <stdio.h>

#define MOUNT_POINT "/sdcard"
#define BB_MISO  19
#define BB_MOSI  23
#define BB_CLK   18
#define BB_CS    5


uint8_t blackbox_init();
uint8_t create_and_open_bin_file();
void close_bin_file();
void write_flight_to_bin_file(uint8_t *data, uint8_t size);
void write_navigation_to_bin_file(uint8_t *data, uint8_t size);

#endif