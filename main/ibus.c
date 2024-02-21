#include "ibus.h"
#include "uart.h"
#include <string.h>


ibus_t *radio_ptr;

static void process_new_line(uint8_t *bytes);

void ibus_init(ibus_t *rc)
{
    uart_begin(UART_NUM_1, 115200, 33, 35, UART_PARITY_DISABLE);
    radio_ptr = rc;
}

void parse_ibus_data(uart_data_t *recv)
{
    static uint8_t new_line_found = 0;
    static const uint8_t size = 30;
    static uint8_t buff[30] = {0};
    static uint8_t byte_counter = 0;
    static const uint8_t protocol_lenght = 0x20;
    static const uint8_t command_code = 0x40;
    static uint8_t prev_byte = 0;


    for (uint8_t i = 0; i < recv->lenght; i++)
    {

        if (new_line_found == 1)
        {
            buff[byte_counter++] = recv->data[i];

            if (byte_counter == size)
            {
                byte_counter = 0;
                new_line_found = 0;
                process_new_line(buff);
            }
        }
        else if (recv->data[i] == command_code && prev_byte == protocol_lenght)
        {
            new_line_found = 1;
        }
        prev_byte = recv->data[i];
    }
}


static void process_new_line(uint8_t *bytes)
{

 	uint16_t checksum_cal = 0xffff;
	uint16_t checksum_ibus;

    checksum_cal -= 0x20;
    checksum_cal -= 0x40;

	for(int i = 0; i < 28; i++)
	{
		checksum_cal -= bytes[i];
	}

	checksum_ibus = bytes[29] << 8 | bytes[28]; // checksum value from ibus

    if (checksum_ibus == checksum_cal)
    {
        memcpy(radio_ptr, bytes, 28);
        //printf("%d\n", radio_ptr->ch0);
    }
}