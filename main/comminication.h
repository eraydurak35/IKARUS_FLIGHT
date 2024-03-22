#ifndef COMM_H
#define COMM_H

#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_private/wifi.h"
#include "typedefs.h"

#define CHANNEL 6
#define DATARATE WIFI_PHY_RATE_54M//WIFI_PHY_RATE_24M

#define TELEM_HEADER 0xFF
#define CONF_HEADER 0XFE
#define WP_HEADER 0xFD
#define MTR_TEST_HEADER 0xFC

void comminication_init(config_t *cfg, waypoint_t *wp, uint8_t *flg, uint8_t *mtr_tst);
void comm_send_telem(telemetry_t *telem);
void comm_send_conf(config_t *conf);
void comm_send_wp();
const uint8_t *get_mag_data();
void comm_send_motor_test_result(float *result);

#endif