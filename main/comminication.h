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

void comminication_init(config_t *cfg, float *mg_cal, waypoint_t *wp, uint8_t *flg);
void comm_send_telem(telemetry_t *telem);
void comm_send_conf(config_t *conf);
void comm_send_wp();

#endif