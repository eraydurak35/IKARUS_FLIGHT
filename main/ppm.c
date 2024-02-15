#include "ppm.h"
#include "typedefs.h"

void ppm_parse(radio_t *rc)
{
    static uint8_t counter = 0;

    if (rc->ppm_interrupt_flag > 0)
    {
        rc->pulse_lenght = rc->current_time - rc->prev_time;
        rc->prev_time = rc->current_time;

        if (rc->ppm_interrupt_flag > 1 || ((rc->pulse_lenght > 2500 || rc->pulse_lenght < 500) && rc->ppm_start_found == 1))
        {
            rc->prev_time = rc->current_time;
            rc->pulse_lenght = 0;
            rc->ppm_start_found = 0;
            counter = 0;
        }
        rc->ppm_interrupt_flag = 0;

        if (rc->ppm_start_found == 1)
        {
            rc->channel[counter] = (rc->pulse_lenght * 3 + rc->channel[counter] * 7) / 10;
            counter++;
            if (counter == 8)
            {
                counter = 0;
                rc->ppm_start_found = 0;
                // flightModeSelector();
            }
        }
        else if (rc->pulse_lenght > 3000)
        {
            rc->ppm_start_found = 1;

            // printf("%d\n", rc->channel[0]);
            /*
            Serial.print(rc.channel[0]);
            Serial.print(",");
            Serial.print(rc.channel[1]);
            Serial.print(",");
            Serial.print(rc.channel[2]);
            Serial.print(",");
            Serial.print(rc.channel[3]);
            Serial.print(",");
            Serial.print(rc.channel[4]);
            Serial.print(",");
            Serial.print(rc.channel[5]);
            Serial.print(",");
            Serial.print(rc.channel[6]);
            Serial.print(",");
            Serial.println(rc.channel[7]);
            */
        }
    }
}