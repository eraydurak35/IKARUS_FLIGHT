#include "nav_comm.h"
#include "typedefs.h"
#include <string.h>
#include "uart.h"

#define data1_header1 100
#define data1_header2 101
#define data1_footer 102

#define data2_header1 90
#define data2_header2 91
#define data2_footer 92

#define data3_header1 80
#define data3_header2 81
#define data3_footer 82

#define ins_config_header1 70
#define ins_config_header2 71
#define ins_config_footer 72

#define flight_header1 60
#define flight_header2 61
#define flight_footer 62

#define range_finder_header1 50
#define range_finder_header2 51
#define range_finder_footer 52

static flight_t *flight_ptr;
static range_finder_t *range_ptr;
static nav_config_t config_nav;
static config_t *config_ptr;
static states_t *state_ptr;

static data_1_t *data1_ptr;
static data_2_t *data2_ptr;
static data_3_t *data3_ptr;

static const float pitch_trim_deg = -1.23;
static const float roll_trim_deg = 0.4;

static uint8_t validate_checksum(uint8_t *buff, uint8_t size);
static void create_packet(uint8_t *write_buff, uint8_t size, uint8_t header1, uint8_t header2, uint8_t footer);

void nav_comm_init(states_t *std, flight_t *flt, range_finder_t *rng, config_t *cfg, data_1_t *d1, data_2_t *d2, data_3_t *d3)
{
    flight_ptr = flt;
    range_ptr = rng;
    config_ptr = cfg;
    state_ptr = std;
    data1_ptr = d1;
    data2_ptr = d2;
    data3_ptr = d3;
}

void parse_nav_data(uart_data_t *uart_buff)
{
    static uint8_t isHeader1Found = 0;
    static uint8_t isHeader2Found = 0;
    static uint8_t isHeader3Found = 0;
    static uint8_t read_buffer[40];
    static uint8_t byte_counter = 0;
    static uint8_t prev_data_byte;
    static const uint8_t data1_size = sizeof(data_1_t);
    static const uint8_t data2_size = sizeof(data_2_t);
    static const uint8_t data3_size = sizeof(data_3_t);

    for (uint8_t i = 0; i < uart_buff->lenght; i++)
    {

        if (isHeader1Found == 1)
        {
            read_buffer[byte_counter++] = uart_buff->data[i];

            if (byte_counter == data1_size + 2)
            {
                byte_counter = 0;
                isHeader1Found = 0;
                if (read_buffer[data1_size + 1] == data1_footer)
                {
                    if (validate_checksum(read_buffer, data1_size) == 1)
                    {
                        memcpy(data1_ptr, read_buffer, data1_size);
                        state_ptr->pitch_deg = (data1_ptr->pitch / 360.0f) - pitch_trim_deg;
                        state_ptr->roll_deg = (data1_ptr->roll / 360.0f) - roll_trim_deg;
                        state_ptr->heading_deg = data1_ptr->heading / 180.0f;

                        state_ptr->pitch_dps = data1_ptr->pitch_dps / 100.0f;
                        state_ptr->roll_dps = data1_ptr->roll_dps / 100.0f;
                        state_ptr->yaw_dps = data1_ptr->yaw_dps / 100.0f;

                        // printf("%.2f\n", state_ptr->pitch_deg);
                        //  logl("0");
                        //  uart_counter++;
                    }
                }
            }
        }
        else if (isHeader2Found == 1)
        {
            read_buffer[byte_counter++] = uart_buff->data[i];

            if (byte_counter == data2_size + 2)
            {
                byte_counter = 0;
                isHeader2Found = 0;

                if (read_buffer[data2_size + 1] == data2_footer)
                {
                    if (validate_checksum(read_buffer, data2_size) == 1)
                    {
                        memcpy(data2_ptr, read_buffer, data2_size);

                        state_ptr->altitude_m = data2_ptr->altitude / 100.0f;
                        state_ptr->vel_forward_ms = data2_ptr->velocity_x_ms / 1000.0f;
                        state_ptr->vel_right_ms = data2_ptr->velocity_y_ms / 1000.0f;
                        state_ptr->vel_up_ms = data2_ptr->velocity_z_ms / 1000.0f;
                    }
                }
            }
        }
        else if (isHeader3Found == 1)
        {
            read_buffer[byte_counter++] = uart_buff->data[i];

            if (byte_counter == data3_size + 2)
            {
                byte_counter = 0;
                isHeader3Found = 0;
                if (read_buffer[data3_size + 1] == data3_footer)
                {
                    if (validate_checksum(read_buffer, data3_size) == 1)
                    {
                        memcpy(data3_ptr, read_buffer, data3_size);
                    }
                }
            }
        }
        else if (uart_buff->data[i] == data1_header2 && prev_data_byte == data1_header1)
        {
            isHeader1Found = 1;
        }
        else if (uart_buff->data[i] == data2_header2 && prev_data_byte == data2_header1)
        {
            isHeader2Found = 1;
        }
        else if (uart_buff->data[i] == data3_header2 && prev_data_byte == data3_header1)
        {
            isHeader3Found = 1;
        }

        prev_data_byte = uart_buff->data[i];
    }
}

void send_nav_config()
{
    // 44 + 2 header + 1 checksum + 1 footer = 48 bytes
    static uint8_t byteArray[sizeof(nav_config_t) + 4];
    config_nav.notch_1_freq = config_ptr->notch_1_freq;
    config_nav.notch_1_bandwidth = config_ptr->notch_1_bandwidth;
    config_nav.notch_2_freq = config_ptr->notch_2_freq;
    config_nav.notch_2_bandwidth = config_ptr->notch_2_bandwidth;
    config_nav.ahrs_filter_beta = config_ptr->ahrs_filter_beta;
    config_nav.ahrs_filter_zeta = config_ptr->ahrs_filter_zeta;
    config_nav.alt_filter_beta = config_ptr->alt_filter_beta;
    config_nav.mag_declination_deg = config_ptr->mag_declination_deg;
    config_nav.velz_filter_beta = config_ptr->velz_filter_beta;
    config_nav.velz_filter_zeta = config_ptr->velz_filter_zeta;
    config_nav.velxy_filter_beta = config_ptr->velxy_filter_beta;

    memcpy(byteArray + 2, &config_nav, sizeof(nav_config_t));
    create_packet(byteArray, sizeof(nav_config_t), ins_config_header1, ins_config_header2, ins_config_footer);
    uart_write(UART_NUM_1, byteArray, sizeof(nav_config_t) + 4);
}

void send_flight_status()
{
    // 5 + 2 header + 1 checksum + 1 footer = 9 bytes
    static uint8_t byteArray[sizeof(flight_t) + 4];
    memcpy(byteArray + 2, flight_ptr, sizeof(flight_t));
    create_packet(byteArray, sizeof(flight_t), flight_header1, flight_header2, flight_footer);
    uart_write(UART_NUM_1, byteArray, sizeof(flight_t) + 4);
}

void send_range()
{
    // 2 + 2 header + 1 checksum + 1 footer = 6 bytes
    static uint8_t byteArray[sizeof(range_finder_t) + 4];
    memcpy(byteArray + 2, range_ptr, sizeof(range_finder_t));
    create_packet(byteArray, sizeof(range_finder_t), range_finder_header1, range_finder_header2, range_finder_footer);
    uart_write(UART_NUM_1, byteArray, sizeof(range_finder_t) + 4);
}

static uint8_t validate_checksum(uint8_t *buff, uint8_t size)
{
    uint8_t checksum = 0;
    for (uint8_t idx = 0; idx < size; idx++)
    {
        checksum ^= buff[idx];
    }
    if (checksum == buff[size])
    {
        return 1;
    }
    return 0;
}

static void create_packet(uint8_t *write_buff, uint8_t size, uint8_t header1, uint8_t header2, uint8_t footer)
{
    uint8_t checksum = 0;
    write_buff[0] = header1;
    write_buff[1] = header2;
    for (uint8_t idx = 2; idx < size + 2; idx++)
    {
        checksum ^= write_buff[idx];
    }
    write_buff[size + 2] = checksum;
    write_buff[size + 3] = footer;
}