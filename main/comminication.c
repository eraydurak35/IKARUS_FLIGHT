#include <string.h>
#include "comminication.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_private/wifi.h"
#include "nvs_flash.h"
#include "nv_storage.h"

static const uint8_t drone_mac_address[6] = {0x04, 0x61, 0x05, 0x05, 0x3A, 0xE4};
static const uint8_t ground_station_mac_address[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static esp_now_peer_info_t peerInfo;

static gamepad_t *gamepad_ptr = NULL;
static config_t *config_ptr = NULL;
static waypoint_t *waypoint_ptr = NULL;

static void espnow_receive_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len);
static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status);

void comminication_init(gamepad_t *gmpd, config_t *cfg, waypoint_t *wp)
{
    nvs_flash_init();
    wifi_init_config_t my_config = WIFI_INIT_CONFIG_DEFAULT();
    my_config.ampdu_tx_enable = 0;
    esp_wifi_init(&my_config);
    esp_wifi_start();

    esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE);
    esp_wifi_internal_set_fix_rate(WIFI_IF_STA, true, DATARATE);
    esp_wifi_set_mac(WIFI_IF_STA, &drone_mac_address[0]);

    esp_now_init();
    esp_now_register_send_cb(espnow_send_cb);
    esp_now_register_recv_cb(espnow_receive_cb);
    memcpy(peerInfo.peer_addr, ground_station_mac_address, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);

    gamepad_ptr = gmpd;
    config_ptr = cfg;
    waypoint_ptr = wp;
}

void comm_send_telem(telemetry_t *telem)
{
    uint8_t buffer[sizeof(telemetry_t) + 1];
    buffer[0] = TELEM_HEADER;
    memcpy(buffer + 1, (uint8_t *)telem, sizeof(telemetry_t));
    esp_now_send(ground_station_mac_address, buffer, sizeof(buffer));
}

void comm_send_conf(config_t *conf)
{
    uint8_t buffer[sizeof(config_t) + 1];
    buffer[0] = CONF_HEADER;
    memcpy(buffer + 1, (uint8_t *)conf, sizeof(config_t));
    esp_now_send(ground_station_mac_address, buffer, sizeof(buffer));
}

static void espnow_receive_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    /*     if (data[0] == 0xFF && len == sizeof(gamepad_t) + 1)
        {
            memcpy(gamepad_ptr, data + 1, sizeof(gamepad_t));
        } */
    if (data[0] == 0xFE && len == sizeof(config_t) + 1)
    {

        memcpy(config_ptr, data + 1, sizeof(config_t));
        save_config(config_ptr);
    }
    else if (data[0] == 0xFD && len == 226)
    {
        memcpy(waypoint_ptr->latitude, data + 1, sizeof(waypoint_ptr->latitude));
        memcpy(waypoint_ptr->longitude, data + sizeof(waypoint_ptr->longitude) + 1, sizeof(waypoint_ptr->longitude));
        memcpy(waypoint_ptr->altitude, data + (sizeof(waypoint_ptr->longitude) * 2) + 1, sizeof(waypoint_ptr->altitude));
        /*
        for (int i = 0; i < 25; i++)
        {
            log(i); log("   "); log(waypoint.latitude[i]); log("   "); log(waypoint.longitude[i]); log("   "); logl(waypoint.altitude[i]);
        }
        */
    }
}
static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
}
