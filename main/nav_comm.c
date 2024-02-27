#include "nav_comm.h"
#include "typedefs.h"
#include "driver/spi_master.h"
#include <string.h>
#include "esp_heap_caps.h"
#include "comminication.h"

static flight_data_t flight_data;
static nav_data_t *nav_data_ptr;
static states_t *state_ptr;
static range_finder_t *range_ptr;
static flight_t *flight_ptr;
static config_t *config_ptr;


static spi_transaction_t trans;
static spi_device_handle_t handle;

static uint8_t *spi_heap_mem_receive = NULL;
static uint8_t *spi_heap_mem_send = NULL;
static const uint8_t spi_trans_byte_size = sizeof(nav_data_t) + 4;

static void checksum_generate(uint8_t *data, uint8_t size, uint8_t *cs1, uint8_t *cs2);
static uint8_t checksum_verify(uint8_t *data, uint8_t size);

void nav_comm_init(nav_data_t *nav, states_t *stt, range_finder_t *rng, flight_t *flt, config_t *cfg)
{
    nav_data_ptr = nav;
    state_ptr = stt;
    range_ptr = rng;
    flight_ptr = flt;
    config_ptr = cfg;

    spi_bus_config_t buscfg =
        {
            .miso_io_num = PIN_NUM_MISO,
            .mosi_io_num = PIN_NUM_MOSI,
            .sclk_io_num = PIN_NUM_CLK,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
            .max_transfer_sz = 0,
        };
    spi_device_interface_config_t devcfg =
        {
            .clock_speed_hz = 5 * 1000 * 1000,
            .mode = 0,
            .spics_io_num = PIN_NUM_CS,
            .queue_size = 7,
        };

    spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    spi_bus_add_device(SPI2_HOST, &devcfg, &handle);

    spi_heap_mem_receive = heap_caps_malloc(spi_trans_byte_size, MALLOC_CAP_DMA);
    spi_heap_mem_send = heap_caps_malloc(spi_trans_byte_size, MALLOC_CAP_DMA);

    memset(spi_heap_mem_receive, 0, spi_trans_byte_size);
    memset(spi_heap_mem_send, 0, spi_trans_byte_size);
}

uint8_t *master_send_recv_nav_comm(uint8_t *new_config_flag)
{
    // prepare send packet

    flight_data.range_cm = range_ptr->range_cm;
    flight_data.arm_status = flight_ptr->arm_status;
/*  flight_data.alt_hold_status = flight_ptr->alt_hold_status;
    flight_data.pos_hold_status = flight_ptr->pos_hold_status;
    flight_data.waypoint_mission_status = flight_ptr->waypoint_mission_status; */

    if (*new_config_flag == 1)
    {
        *new_config_flag = 0;
        flight_data.notch_1_freq = config_ptr->notch_1_freq;
        flight_data.notch_2_freq = config_ptr->notch_2_freq;
        flight_data.notch_1_bandwidth = config_ptr->notch_1_bandwidth;
        flight_data.notch_2_bandwidth = config_ptr->notch_2_bandwidth;

        flight_data.ahrs_filter_beta = config_ptr->ahrs_filter_beta;
        flight_data.ahrs_filter_zeta = config_ptr->ahrs_filter_zeta;
        flight_data.alt_filter_beta = config_ptr->alt_filter_beta;
        flight_data.velz_filter_beta = config_ptr->velz_filter_beta;
        flight_data.velz_filter_zeta = config_ptr->velz_filter_zeta;

        flight_data.velxy_filter_beta = config_ptr->velxy_filter_beta;
        flight_data.mag_declination_deg = config_ptr->mag_declination_deg;
        flight_data.is_new_config = 1;
    }
    else
    {
        flight_data.is_new_config = 0;
    }

    memcpy(spi_heap_mem_send + 1, &flight_data, sizeof(flight_data_t));
    spi_heap_mem_send[0] = HEADER;
    static uint8_t checksum_a, checksum_b;
    checksum_generate(spi_heap_mem_send + 1, spi_trans_byte_size - 4, &checksum_a, &checksum_b);
    spi_heap_mem_send[spi_trans_byte_size - 3] = checksum_a;
    spi_heap_mem_send[spi_trans_byte_size - 2] = checksum_b;
    spi_heap_mem_send[spi_trans_byte_size - 1] = FOOTER;

    // Send data must be bigger than 8 and divisible by 4
    memset(&trans, 0, sizeof(trans));
    trans.length = 8 * spi_trans_byte_size; // 68 bytes
    trans.tx_buffer = spi_heap_mem_send;    // send_buffer;
    trans.rx_buffer = spi_heap_mem_receive; // receive_buffer;
    spi_device_transmit(handle, &trans);

    if (spi_heap_mem_receive[0] == HEADER && spi_heap_mem_receive[spi_trans_byte_size - 1] == FOOTER && checksum_verify(spi_heap_mem_receive, spi_trans_byte_size))
    {
        memcpy(nav_data_ptr, spi_heap_mem_receive + 1, sizeof(nav_data_t));

        state_ptr->pitch_deg = nav_data_ptr->pitch;
        state_ptr->roll_deg = nav_data_ptr->roll;
        state_ptr->heading_deg = nav_data_ptr->heading;

        state_ptr->pitch_dps = nav_data_ptr->pitch_dps;
        state_ptr->roll_dps = nav_data_ptr->roll_dps;
        state_ptr->yaw_dps = nav_data_ptr->yaw_dps;

        state_ptr->vel_forward_ms = nav_data_ptr->velocity_x_ms / 1000.0f;
        state_ptr->vel_right_ms = nav_data_ptr->velocity_y_ms / 1000.0f;
        state_ptr->vel_up_ms = nav_data_ptr->velocity_z_ms / 1000.0f;

        state_ptr->altitude_m = nav_data_ptr->altitude / 100.0f;

        state_ptr->acc_forward_ms2 = nav_data_ptr->acc_x_ned_ms2 / 10.0f;
        state_ptr->acc_right_ms2 = nav_data_ptr->acc_y_ned_ms2 / 10.0f;
        state_ptr->acc_up_ms2 = nav_data_ptr->acc_z_ned_ms2 / 10.0f;

        return spi_heap_mem_receive;
    }

    return NULL;
}

static void checksum_generate(uint8_t *data, uint8_t size, uint8_t *cs1, uint8_t *cs2)
{
    uint8_t checksum1 = 0, checksum2 = 0;

    for (uint8_t i = 0; i < size - 2; i++)
    {
        checksum1 = checksum1 + data[i];
        checksum2 = checksum2 + checksum1;
    }

    *cs1 = checksum1;
    *cs2 = checksum2;
}

static uint8_t checksum_verify(uint8_t *data, uint8_t size)
{
    uint8_t c1, c2;
    checksum_generate(data + 1, size - 4, &c1, &c2);
    if (c1 == data[size - 3] && c2 == data[size - 2])
        return 1;
    return 0;
}
