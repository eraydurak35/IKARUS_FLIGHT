/*
 *    8888888 888    d8P         d8888 8888888b.  888     888  .d8888b.        .d888 888 d8b          888      888    
 *      888   888   d8P         d88888 888   Y88b 888     888 d88P  Y88b      d88P"  888 Y8P          888      888    
 *      888   888  d8P         d88P888 888    888 888     888 Y88b.           888    888              888      888    
 *      888   888d88K         d88P 888 888   d88P 888     888  "Y888b.        888888 888 888  .d88b.  88888b.  888888 
 *      888   8888888b       d88P  888 8888888P"  888     888     "Y88b.      888    888 888 d88P"88b 888 "88b 888    
 *      888   888  Y88b     d88P   888 888 T88b   888     888       "888      888    888 888 888  888 888  888 888    
 *      888   888   Y88b   d8888888888 888  T88b  Y88b. .d88P Y88b  d88P      888    888 888 Y88b 888 888  888 Y88b.  
 *    8888888 888    Y88b d88P     888 888   T88b  "Y88888P"   "Y8888P"       888    888 888  "Y88888 888  888  "Y888 
 *                                                                                                888                 
 *                                                                                           Y8b d88P                 
 *                                                                                            "Y88P"                  
 */

// ||############################||
// ||      ESP IDF LIBRARIES     ||
// ||############################||

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_system.h"
#include "driver/i2c.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "rom/gpio.h"
#include <stdio.h>
#include "math.h"
// ||############################||
// ||      CUSTOM LIBRARIES      ||
// ||############################||
#include "control_algorithm.h"
#include "comminication.h"
#include "nv_storage.h"
#include "blackbox.h"
#include "typedefs.h"
#include "nav_comm.h"
#include "filters.h"
#include "tf_luna.h"
#include "ublox.h"
#include "uart.h"
#include "gpio.h"
#include "ibus.h"
#include "esc.h"
#include "i2c.h"

static TaskHandle_t task1_handler;
static TaskHandle_t task2_handler;
static config_t config;
static states_t state;
static flight_t flight;
static target_t target;
static telemetry_t telem;
static range_finder_t range_finder;
static gps_t gps;
static ibus_t radio = {1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000};
static waypoint_t waypoint = {{0}, {0}, {0}, -1, 1};
static nav_data_t nav_data;
static uint8_t new_config_received_flag = 0;


void IRAM_ATTR timer1_callback(void *arg)
{
    xTaskNotifyFromISR(task1_handler, 1, eIncrement, false);
}

void task_1(void *pvParameters)
{
    static uint32_t notification = 0;
    static uint8_t counter1 = 0;
    static uint8_t counter2 = 0;
    static uint8_t is_sd_inserted = 0;
    static uint8_t *nav_data_for_blackbox = NULL;
    static uint8_t is_ok_to_write_to_file = 0;

    is_sd_inserted = blackbox_init();
    gpio_configure();
    dshot_esc_init();
    comminication_init(&config, &waypoint, &new_config_received_flag);
    read_config(&config);
    comm_send_conf(&config);
    i2c_master_init(I2C_NUM_0, SDA1, SCL1, 400000, GPIO_PULLUP_DISABLE);
    control_init(&radio, &telem, &flight, &target, &state, &config, &waypoint, &gps);
    nav_comm_init(&nav_data, &state, &range_finder, &flight, &config);
    telem.battery_voltage = get_bat_volt() * config.v_sens_gain;

    while (1)
    {
        if (xTaskNotifyWait(0, ULONG_MAX, &notification, 1 / portTICK_PERIOD_MS) == pdTRUE)
        {
            nav_data_for_blackbox = master_send_recv_nav_comm(&new_config_received_flag);

            if (is_ok_to_write_to_file == 1 && nav_data_for_blackbox != NULL)
            {
                write_navigation_to_bin_file(nav_data_for_blackbox, 68);
            }

            flight_control();
            counter1++;
            counter2++;
            if (counter1 >= 10) // 100 Hz
            {
                counter1 = 0;
                get_range(&range_finder, &state);
            }
            if (counter2 >= 100) // 10 Hz
            {
                counter2 = 0;
                uint8_t ret = flight_mode_control();
                if (is_sd_inserted == 1)
                {
                    if (ret == 1)
                    {
                        is_ok_to_write_to_file = create_and_open_bin_file();
                    }
                    else if (ret == 2)
                    {
                        close_bin_file();
                        is_ok_to_write_to_file = 0;
                    }
                    
                }

                telem.battery_voltage = (get_bat_volt() * config.v_sens_gain) * 0.005f + telem.battery_voltage * 0.995f;
                telem.pitch = state.pitch_deg;
                telem.roll = state.roll_deg;
                telem.heading = state.heading_deg;
                telem.gyro_x_dps = state.pitch_dps * 100.0f;
                telem.gyro_y_dps = state.roll_dps * 100.0f;
                telem.gyro_z_dps = state.yaw_dps * 100.0f;
                telem.acc_x_ms2 = nav_data.acc_x_ms2;
                telem.acc_y_ms2 = nav_data.acc_y_ms2;
                telem.acc_z_ms2 = nav_data.acc_z_ms2;
                telem.imu_temperature = nav_data.imu_temperature;
                telem.mag_x_mgauss = nav_data.mag_x_gauss;
                telem.mag_y_mgauss = nav_data.mag_y_gauss;
                telem.mag_z_mgauss = nav_data.mag_z_gauss;
                telem.barometer_pressure = nav_data.barometer_pressure;
                telem.barometer_temperature = nav_data.barometer_temperature;
                telem.altitude = nav_data.baro_altitude;
                telem.altitude_calibrated = nav_data.altitude;
                telem.velocity_x_ms = nav_data.velocity_x_ms;
                telem.velocity_y_ms = nav_data.velocity_y_ms;
                telem.velocity_z_ms = nav_data.velocity_z_ms;
                telem.target_pitch = target.pitch;
                telem.target_roll = target.roll;
                telem.target_heading = target.heading;
                telem.target_pitch_dps = target.pitch_degs;
                telem.target_roll_dps = target.roll_degs;
                telem.target_yaw_dps = target.yaw_degs;
                telem.tof_distance_1 = range_finder.range_cm;
                telem.target_altitude = target.altitude;
                telem.target_velocity_x_ms = target.velocity_x_ms;
                telem.target_velocity_y_ms = target.velocity_y_ms;
                telem.target_velocity_z_ms = target.velocity_z_ms;
                telem.flow_quality = nav_data.flow_quality;
                telem.flow_x_velocity = nav_data.flow_x_velocity_ms;
                telem.flow_y_velocity = nav_data.flow_y_velocity_ms;
                telem.gps_fix = gps.fix;
                telem.gps_satCount = gps.satCount;
                telem.gps_latitude = gps.latitude;
                telem.gps_longitude = gps.longitude;
                telem.gps_altitude_m = gps.altitude_mm;
                telem.gps_northVel_ms = gps.northVel_mms;
                telem.gps_eastVel_ms = gps.eastVel_mms;
                telem.gps_downVel_ms = gps.downVel_mms;
                telem.gps_headingOfMotion = gps.headingOfMotion;
                telem.gps_hdop = gps.hdop;
                telem.gps_vdop = gps.vdop;
                telem.gps_latitude_origin = gps.latitude_origin;
                telem.gps_longitude_origin = gps.longitude_origin;
                telem.gps_altitude_origin = gps.altitude_origin_mm;
                telem.target_latitude = target.latitude;
                telem.target_longitude = target.longitude;
                telem.velocity_ms_2d = sqrtf((state.vel_forward_ms * state.vel_forward_ms) + (state.vel_right_ms * state.vel_right_ms));
                telem.tof_distance_2 = target.throttle;

                // 0 full manual 1 altitude_hold 2 position_hold 3 alt+pos_hold 4 alt+pos+waypoint
                if (flight.waypoint_mission_status == 1)
                {
                    telem.flight_mode = 4;
                }
                else if (flight.alt_hold_status == 1)
                {
                    if (flight.pos_hold_status == 1)
                        telem.flight_mode = 3;
                    else
                        telem.flight_mode = 1;
                }
                else if (flight.pos_hold_status == 1)
                    telem.flight_mode = 2;
                else
                    telem.flight_mode = 0;

                comm_send_telem(&telem);
            }
        }
    }
}

void task_2(void *pvParameters)
{
    static uart_data_t uart2_data;
    static uart_data_t uart1_data;
    gps_init(&gps);
    ibus_init(&radio);
    while (1)
    {
        uart_read(UART_NUM_2, &uart2_data, 10);
        uart_read(UART_NUM_1, &uart1_data, 10);
        parse_ibus_data(&uart1_data);
        parse_gps_data(&uart2_data);
    }
}
void app_main(void)
{
    xTaskCreatePinnedToCore(&task_2, "task2", 1024 * 4, NULL, 1, &task2_handler, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(&task_1, "task1", 1024 * 4, NULL, 1, &task1_handler, tskNO_AFFINITY);

    esp_timer_handle_t timer1;
    const esp_timer_create_args_t timer1_args =
    {
        .callback = &timer1_callback,
        .arg = NULL,
        .name = "timer1"
    };
    esp_timer_create(&timer1_args, &timer1);
    esp_timer_start_periodic(timer1, 1000);
}
