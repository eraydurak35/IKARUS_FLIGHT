#include <string.h>
#include "comminication.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_private/wifi.h"
#include "nvs_flash.h"
#include "nv_storage.h"
#include "nav_comm.h"

static const uint8_t drone_mac_address[6] = {0x04, 0x61, 0x05, 0x05, 0x3A, 0xE4};
static const uint8_t ground_station_mac_address[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static esp_now_peer_info_t peerInfo;

static config_t *config_ptr = NULL;
static waypoint_t *waypoint_ptr = NULL;
static uint8_t *new_data_recv_flag = NULL;
static uint8_t *motor_test_num_ptr = NULL;
static const uint8_t *mag_data;

static void espnow_receive_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len);
static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status);

void comminication_init(config_t *cfg, waypoint_t *wp, uint8_t *flg, uint8_t *mtr_tst)
{
    nvs_flash_init();

    wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    // Set Wi-Fi protocol to long range mode
    ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR));

    // Initialize ESP-NOW
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_wifi_set_mac(WIFI_IF_STA, &drone_mac_address[0]));

    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_receive_cb));
    memcpy(peerInfo.peer_addr, ground_station_mac_address, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);

    config_ptr = cfg;
    waypoint_ptr = wp;
    new_data_recv_flag = flg;
    motor_test_num_ptr = mtr_tst;
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

void comm_send_wp()
{
    uint8_t buffer[sizeof(waypoint_ptr->latitude) + sizeof(waypoint_ptr->longitude) + sizeof(waypoint_ptr->altitude) + 1];
    buffer[0] = WP_HEADER;
    memcpy(buffer + 1, waypoint_ptr->latitude, sizeof(waypoint_ptr->latitude));
    memcpy(buffer + 1 + sizeof(waypoint_ptr->latitude), waypoint_ptr->longitude, sizeof(waypoint_ptr->longitude));
    memcpy(buffer + 1 + sizeof(waypoint_ptr->latitude) + sizeof(waypoint_ptr->longitude), waypoint_ptr->altitude, sizeof(waypoint_ptr->altitude));
    esp_now_send(ground_station_mac_address, buffer, sizeof(buffer));
}
void comm_send_motor_test_result(float *result)
{
    uint8_t buffer[sizeof(float) * 4 + 1];
    buffer[0] = MTR_TEST_HEADER;
    memcpy(buffer + 1, result, sizeof(float) * 4);
    esp_now_send(ground_station_mac_address, buffer, sizeof(buffer));
}
static void espnow_receive_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    if (data[0] == 0xFE && len == sizeof(config_t) + 1)
    {
        memcpy(config_ptr, data + 1, sizeof(config_t));
        save_config(config_ptr);
        *new_data_recv_flag = 1;
    }
    else if (data[0] == 0xFD && len == 226)
    {
        memcpy(waypoint_ptr->latitude, data + 1, sizeof(waypoint_ptr->latitude));
        memcpy(waypoint_ptr->longitude, data + sizeof(waypoint_ptr->longitude) + 1, sizeof(waypoint_ptr->longitude));
        memcpy(waypoint_ptr->altitude, data + (sizeof(waypoint_ptr->longitude) * 2) + 1, sizeof(waypoint_ptr->altitude));
        *new_data_recv_flag = 2;
    }
    else if (data[0] == 0xFC && len == 2)
    {
        if (data[1] == 10)
            *new_data_recv_flag = 3;
        else if (data[1] == 20)
            *new_data_recv_flag = 4;

    }
    else if (data[0] == 0xFB && len == 49)
    {
        *new_data_recv_flag = 5;
        mag_data = data;
    }
    else if (data[0] == 0xFA)
    {
        *motor_test_num_ptr = data[1];
    }

}
static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
}

const uint8_t *get_mag_data()
{
    return mag_data;
}