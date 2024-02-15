#include "ublox.h"
#include "uart.h"

static uint8_t set_to_921600_baud[28] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0x10, 0x0E, 0x00, 0x07, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x19, 0x4E};
// static uint8_t Disable_GPGGA[16] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x24};
// static uint8_t Disable_GPGSA[16] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x32};
// static uint8_t Disable_GPGSV[11] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x03, 0x00, 0xFD, 0x15};
// static uint8_t Disable_GPGLL[16] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2A};
// static uint8_t Disable_GPRMC[16] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x3F};
// static uint8_t Disable_GPVTG[16] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x46};
static uint8_t set_to_10hz[14] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00, 0x01, 0x00, 0x7A, 0x12};
static uint8_t enable_dop[16] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x04, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x15, 0xCC};
static uint8_t enable_pvt[16] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x07, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x18, 0xE1};
static uint8_t dyn_motion_mod_pedestrian[44] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C,
                                                0x01, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4F, 0x82};
static uint8_t full_power[16] = {0xB5, 0x62, 0x06, 0x86, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x94, 0x5A};

static gps_t *gps_ptr;

static uint8_t ubx_checksum_verify(uint8_t *data, uint8_t dataLenght);

void gps_init(gps_t *g)
{
    gps_ptr = g;

    uart_begin(UART_NUM_2, 9600, 14, 15, UART_PARITY_DISABLE);
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    uart_write(UART_NUM_2, set_to_921600_baud, sizeof(set_to_921600_baud));
    vTaskDelay(100 / portTICK_PERIOD_MS);

    uart_driver_delete(UART_NUM_2);
    uart_begin(UART_NUM_2, 921600, 14, 15, UART_PARITY_DISABLE);

    uart_write(UART_NUM_2, set_to_10hz, sizeof(set_to_10hz));
    vTaskDelay(100 / portTICK_PERIOD_MS);

    uart_write(UART_NUM_2, enable_pvt, sizeof(enable_pvt));
    vTaskDelay(100 / portTICK_PERIOD_MS);

    uart_write(UART_NUM_2, enable_dop, sizeof(enable_dop));
    vTaskDelay(100 / portTICK_PERIOD_MS);

    uart_write(UART_NUM_2, dyn_motion_mod_pedestrian, sizeof(dyn_motion_mod_pedestrian));
    vTaskDelay(100 / portTICK_PERIOD_MS);

    uart_write(UART_NUM_2, full_power, sizeof(full_power));
}

void parse_gps_data(uart_data_t *data_buff)
{
    // static uint8_t data;
    static uint8_t PVT_Message_Found = 0;
    static uint8_t DOP_Message_Found = 0;
    static uint8_t prev_data;
    static uint16_t start_message;
    static uint8_t RecBytes[100];
    static uint8_t byte_counter;
    static const uint8_t PVT_Message_Size = 90;
    static const uint8_t DOP_Message_Size = 24;

    for (uint8_t i = 0; i < data_buff->lenght; i++)
    {
        // data = Serial2.read();

        if (PVT_Message_Found == 0 && DOP_Message_Found == 0)
        {
            start_message = data_buff->data[i] | prev_data << 8;
            if (start_message == 260)
            {
                DOP_Message_Found = 1;
                RecBytes[0] = 0x01;
                RecBytes[1] = 0x04;
                byte_counter = 2;
                prev_data = 0;
            }
            else if (start_message == 263)
            {
                PVT_Message_Found = 1;
                RecBytes[0] = 0x01;
                RecBytes[1] = 0x07;
                byte_counter = 2;
                prev_data = 0;
            }
            else
            {
                prev_data = data_buff->data[i];
            }
        }
        else if (DOP_Message_Found == 1)
        {
            RecBytes[byte_counter] = data_buff->data[i];
            byte_counter++;

            if (byte_counter == DOP_Message_Size)
            {
                byte_counter = 0;
                DOP_Message_Found = 0;

                if (ubx_checksum_verify(RecBytes, DOP_Message_Size) == 1)
                {
                    gps_ptr->vdop = RecBytes[14] | RecBytes[15] << 8;
                    gps_ptr->hdop = RecBytes[16] | RecBytes[17] << 8;
                }
            }
        }
        else if (PVT_Message_Found == 1)
        {
            RecBytes[byte_counter] = data_buff->data[i];
            byte_counter++;

            if (byte_counter == PVT_Message_Size)
            {
                byte_counter = 0;
                PVT_Message_Found = 0;

                if (ubx_checksum_verify(RecBytes, PVT_Message_Size) == 1)
                {
                    gps_ptr->fix = RecBytes[24];
                    gps_ptr->satCount = RecBytes[27];
                    gps_ptr->longitude = RecBytes[28] | RecBytes[29] << 8 | RecBytes[30] << 16 | RecBytes[31] << 24;       // deg
                    gps_ptr->latitude = RecBytes[32] | RecBytes[33] << 8 | RecBytes[34] << 16 | RecBytes[35] << 24;        // deg
                    gps_ptr->altitude_mm = RecBytes[40] | RecBytes[41] << 8 | RecBytes[42] << 16 | RecBytes[43] << 24;     // mm
                    gps_ptr->northVel_mms = RecBytes[52] | RecBytes[53] << 8 | RecBytes[54] << 16 | RecBytes[55] << 24;    // mm/s
                    gps_ptr->eastVel_mms = RecBytes[56] | RecBytes[57] << 8 | RecBytes[58] << 16 | RecBytes[59] << 24;     // mm/s
                    gps_ptr->downVel_mms = RecBytes[60] | RecBytes[61] << 8 | RecBytes[62] << 16 | RecBytes[63] << 24;     // mm/s
                    gps_ptr->headingOfMotion = RecBytes[68] | RecBytes[69] << 8 | RecBytes[70] << 16 | RecBytes[71] << 24; // deg

                    // printf("%d\n", gps_ptr->satCount);
                }
            }
        }
    }
}

static uint8_t ubx_checksum_verify(uint8_t *data, uint8_t dataLenght)
{
    uint8_t CK_A = 0, CK_B = 0;

    for (uint8_t i = 0; i < dataLenght - 2; i++)
    {
        CK_A = CK_A + data[i];
        CK_B = CK_B + CK_A;
    }

    if (CK_A == data[dataLenght - 2] && CK_B == data[dataLenght - 1])
        return 1;
    return 0;
}
