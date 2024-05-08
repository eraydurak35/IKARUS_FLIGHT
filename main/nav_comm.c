#include "nav_comm.h"
#include "typedefs.h"
#include "driver/spi_master.h"
#include <string.h>
#include "esp_heap_caps.h"
#include "comminication.h"

static nav_data_t *nav_data_ptr;
static states_t *state_ptr;
static range_finder_t *range_ptr;
static flight_t *flight_ptr;
static config_t *config_ptr;
static nav_config_t nav_config;
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

uint8_t *master_send_recv_nav_comm(uint8_t send_data_type)
{

    if (send_data_type == 0)
    {
        spi_heap_mem_send[0] = SEND_HEADER_1;
        spi_heap_mem_send[1] = (uint8_t)(range_ptr->range_cm >> 8);
        spi_heap_mem_send[2] = (uint8_t)(range_ptr->range_cm);
        spi_heap_mem_send[3] = flight_ptr->arm_status;

        static uint8_t checksum_a, checksum_b;
        static uint8_t size = 3;

        checksum_generate(spi_heap_mem_send + 1, size, &checksum_a, &checksum_b);
        spi_heap_mem_send[size + 1] = checksum_a;
        spi_heap_mem_send[size + 2] = checksum_b;
        spi_heap_mem_send[size + 3] = FOOTER;
    }
    else if (send_data_type == 1)
    {
        nav_config.notch_1_freq = config_ptr->notch_1_freq;
        nav_config.notch_2_freq = config_ptr->notch_2_freq;
        nav_config.notch_1_bandwidth = config_ptr->notch_1_bandwidth;
        nav_config.notch_2_bandwidth = config_ptr->notch_2_bandwidth;
        nav_config.ahrs_filter_beta = config_ptr->ahrs_filter_beta;
        nav_config.ahrs_filter_zeta = config_ptr->ahrs_filter_zeta;
        nav_config.alt_filter_beta = config_ptr->alt_filter_beta;
        nav_config.velz_filter_beta = config_ptr->velz_filter_beta;
        nav_config.velz_filter_zeta = config_ptr->velz_filter_zeta;
        nav_config.velxy_filter_beta = config_ptr->velxy_filter_beta;
        nav_config.mag_declination_deg = config_ptr->mag_declination_deg;

        spi_heap_mem_send[0] = SEND_HEADER_2;
        memcpy(spi_heap_mem_send + 1, &nav_config, sizeof(nav_config_t));

        static uint8_t checksum_a, checksum_b;
        checksum_generate(spi_heap_mem_send + 1, sizeof(nav_config_t), &checksum_a, &checksum_b);

        spi_heap_mem_send[sizeof(nav_config_t) + 1] = checksum_a;
        spi_heap_mem_send[sizeof(nav_config_t) + 2] = checksum_b;
        spi_heap_mem_send[sizeof(nav_config_t) + 3] = FOOTER;
    }
    else if (send_data_type == 5)
    {
        const uint8_t *my_data = get_mag_data();
        spi_heap_mem_send[0] = SEND_HEADER_3;
        memcpy(spi_heap_mem_send + 1, my_data + 1, 48);

        static uint8_t checksum_a, checksum_b;
        checksum_generate(spi_heap_mem_send + 1, 48, &checksum_a, &checksum_b);

        spi_heap_mem_send[48 + 1] = checksum_a;
        spi_heap_mem_send[48 + 2] = checksum_b;
        spi_heap_mem_send[48 + 3] = FOOTER;
    }
    else if (send_data_type == 6)
    {
        const uint8_t *my_data = get_acc_data();
        spi_heap_mem_send[0] = SEND_HEADER_4;
        memcpy(spi_heap_mem_send + 1, my_data + 1, 8);

        static uint8_t checksum_a, checksum_b;
        checksum_generate(spi_heap_mem_send + 1, 8, &checksum_a, &checksum_b);

        spi_heap_mem_send[8 + 1] = checksum_a;
        spi_heap_mem_send[8 + 2] = checksum_b;
        spi_heap_mem_send[8 + 3] = FOOTER;
    }



    // Send data must be bigger than 8 and divisible by 4
    memset(&trans, 0, sizeof(trans));
    trans.length = 8 * spi_trans_byte_size; // 68 bytes
    trans.tx_buffer = spi_heap_mem_send;    // send_buffer;
    trans.rx_buffer = spi_heap_mem_receive; // receive_buffer;
    spi_device_transmit(handle, &trans);

    if (spi_heap_mem_receive[0] == RECV_HEADER && spi_heap_mem_receive[spi_trans_byte_size - 1] == FOOTER && checksum_verify(spi_heap_mem_receive, spi_trans_byte_size))
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
    for (uint8_t i = 0; i < size; i++)
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
