#include "motor_test.h"
#include "typedefs.h"
#include "esc.h"
#include "math.h"

static uint16_t counter = 0;
static float result[4];
static float base_noise_level = 0;

uint8_t motor_test(nav_data_t *data, uint8_t motor_number)
{
    // test takes 6.6 seconds
    // first take 100 (100 ms) measurements and determine noise floor
    if (counter < 100)
    {
        base_noise_level += sqrtf(data->acc_x_ms2 * data->acc_x_ms2 + data->acc_y_ms2 * data->acc_y_ms2 + data->acc_z_ms2 * data->acc_z_ms2);
    }
    else if (counter == 100)
    {
        // then run the desired motor to hover level
        base_noise_level /= 100.0f;

        if (motor_number == 1)
            write_throttle(1000, 0, 0, 0);
        else if (motor_number == 2)
            write_throttle(0, 1000, 0, 0);
        else if (motor_number == 3)
            write_throttle(0, 0, 1000, 0);
        else if (motor_number == 4)
            write_throttle(0, 0, 0, 1000);

        result[motor_number - 1] = 0;
    }
    else if (counter > 1600 && counter <= 6600)
    {
        float acc_vector = sqrtf(data->acc_x_ms2 * data->acc_x_ms2 + data->acc_y_ms2 * data->acc_y_ms2 + data->acc_z_ms2 * data->acc_z_ms2) - base_noise_level;
        result[motor_number - 1] += acc_vector * acc_vector;
    }
    else if (counter > 6600)
    {
        write_throttle(0, 0, 0, 0);
        result[motor_number - 1] = sqrtf((result[motor_number - 1] / 5000.0f));
        counter = 0;
        base_noise_level = 0;
        return 0;
    }

    counter++;
    return 1;

}

float *get_motor_test_results()
{
    return result;
}