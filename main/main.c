// ||############################||
// ||      ESP IDF LIBRARIES     ||
// ||############################||
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "driver/i2c.h"
#include "driver/uart.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "rom/gpio.h"
#include "math.h"
// ||############################||
// ||      CUSTOM LIBRARIES      ||
// ||############################||
#include "i2c.h"
#include "filters.h"
#include "uart.h"
#include "gpio.h"
#include "nv_storage.h"
#include "typedefs.h"
#include "nav_comm.h"
#include "tf_luna.h"
#include "ublox.h"
#include "comminication.h"
#include "ppm.h"
#include "control_algorithm.h"
#include "esc.h"

static TaskHandle_t task1_handler;
static TaskHandle_t task2_handler;
static TaskHandle_t task3_handler;
static config_t config;
static states_t state;
static flight_t flight;
static target_t target;
static telemetry_t telem;
static range_finder_t range_finder;
static gps_t gps;
static gamepad_t gamepad;
static radio_t rc = {0, 0, 0, 0, 0, {1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000}};
static waypoint_t waypoint = {{0}, {0}, {0}, -1, 1};
static data_1_t data1;
static data_2_t data2;
static data_3_t data3;

static uint8_t counter1 = 0;
static uint8_t counter2 = 0;

void IRAM_ATTR rc_ppm_isr(void *args)
{
    rc.current_time = esp_timer_get_time();
    rc.ppm_interrupt_flag++;
}

void IRAM_ATTR timer1_callback(void *arg)
{
    xTaskNotifyFromISR(task1_handler, 1, eIncrement, false);
}

void task_1(void *pvParameters)
{

    gpio_configure();
    dshot_esc_init();
    telem.battery_voltage = get_bat_volt() * config.v_sens_gain;
    comminication_init(&gamepad, &config, &waypoint);
    read_config(&config);
    comm_send_conf(&config);
    i2c_master_init(I2C_NUM_0, SDA1, SCL1, 400000, GPIO_PULLUP_DISABLE);
    control_init(&rc, &telem, &flight, &target, &state, &config, &waypoint, &gps);
    static uint32_t receivedValue = 0;
    while (1)
    {
        if (xTaskNotifyWait(0, ULONG_MAX, &receivedValue, 1 / portTICK_PERIOD_MS) == pdTRUE)
        {
            ppm_parse(&rc);
            flight_control();
            counter1++;
            counter2++;
            if (counter1 >= 10) // 100 Hz
            {
                counter1 = 0;
                get_range(&range_finder, &state);
                send_range();
            }
            if (counter2 >= 100) // 10 Hz
            {
                counter2 = 0;
                flight_mode_control();
                send_flight_status();

                telem.battery_voltage = (get_bat_volt() * config.v_sens_gain) * 0.005f + telem.battery_voltage * 0.995f;
                telem.pitch = state.pitch_deg;
                telem.roll = state.roll_deg;
                telem.heading = state.heading_deg;
                telem.gyro_x_dps = state.pitch_dps;
                telem.gyro_y_dps = state.roll_dps;
                telem.gyro_z_dps = state.yaw_dps;
                telem.acc_x_ms2 = data3.acc_x_ms2 / 100.0f;
                telem.acc_y_ms2 = data3.acc_y_ms2 / 100.0f;
                telem.acc_z_ms2 = data3.acc_z_ms2 / 100.0f;
                telem.imu_temperature = data3.imu_temperature / 100.0f;

                telem.mag_x_mgauss = data3.mag_x_gauss / 10.0f;
                telem.mag_y_mgauss = data3.mag_y_gauss / 10.0f;
                telem.mag_z_mgauss = data3.mag_z_gauss / 10.0f;

                telem.barometer_pressure = data2.barometer_pressure / 10.0f;
                telem.barometer_temperature = data2.barometer_temperature / 100.0f;
                telem.altitude = data2.baro_altitude / 100.0f;
                telem.altitude_calibrated = state.altitude_m;
                telem.velocity_x_ms = state.vel_forward_ms;
                telem.velocity_y_ms = state.vel_right_ms;
                telem.velocity_z_ms = state.vel_up_ms;
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
                telem.flow_quality = data2.flow_quality;
                telem.flow_x_velocity = data2.flow_x_velocity_ms / 10.0f;
                telem.flow_y_velocity = data2.flow_y_velocity_ms / 10.0f;
                telem.gps_fix = gps.fix;
                telem.gps_satCount = gps.satCount;
                telem.gps_latitude = gps.latitude / 10000000.0f;
                telem.gps_longitude = gps.longitude / 10000000.0f;
                telem.gps_altitude_m = gps.altitude_mm / 1000.0f;
                telem.gps_northVel_ms = gps.northVel_mms / 1000.0f;
                telem.gps_eastVel_ms = gps.eastVel_mms / 1000.0f;
                telem.gps_downVel_ms = gps.downVel_mms / 1000.0f;
                telem.gps_headingOfMotion = gps.headingOfMotion / 100000.0f;
                telem.gps_hdop = gps.hdop / 100.0f;
                telem.gps_vdop = gps.vdop / 100.0f;
                telem.gps_latitude_origin = gps.latitude_origin / 10000000.0f;
                telem.gps_longitude_origin = gps.longitude_origin / 10000000.0f;
                telem.gps_altitude_origin = gps.altitude_origin_mm / 1000.0f;
                telem.target_latitude = target.latitude / 10000000.0f;
                telem.target_longitude = target.longitude / 10000000.0f;
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
    static uart_data_t uart1_data;
    uart_begin(UART_NUM_1, 5000000, GPIO_NUM_17, GPIO_NUM_16, UART_PARITY_EVEN);
    nav_comm_init(&state, &flight, &range_finder, &config, &data1, &data2, &data3);
    while (1)
    {
        uart_read(UART_NUM_1, &uart1_data, 1);
        parse_nav_data(&uart1_data);
    }
}

void task_3(void *pvParameters)
{
    static uart_data_t uart2_data;
    gps_init(&gps);
    while (1)
    {
        uart_read(UART_NUM_2, &uart2_data, 10);
        parse_gps_data(&uart2_data);
    }
}
void app_main(void)
{

    xTaskCreatePinnedToCore(&task_3, "task3", 1024 * 4, NULL, 1, &task3_handler, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(&task_2, "task2", 1024 * 4, NULL, 1, &task2_handler, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(&task_1, "task1", 1024 * 4, NULL, 1, &task1_handler, tskNO_AFFINITY);

    gpio_pad_select_gpio(GPIO_NUM_35);
    gpio_set_direction(GPIO_NUM_35, GPIO_MODE_INPUT);
    gpio_pulldown_dis(GPIO_NUM_35);
    gpio_pullup_en(GPIO_NUM_35);
    gpio_set_intr_type(GPIO_NUM_35, GPIO_INTR_POSEDGE);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(GPIO_NUM_35, rc_ppm_isr, NULL);

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
