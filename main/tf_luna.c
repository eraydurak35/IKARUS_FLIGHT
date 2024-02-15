#include "tf_luna.h"
#include "typedefs.h"
#include "i2c.h"
#include "filters.h"
#include "math.h"

static uint8_t buffer[2];

void get_range(range_finder_t *range_ptr, states_t *state_ptr)
{
    i2c_read_data(I2C_NUM_0, SLAVE_ADDRESS, OUTX_L_G, buffer, 2);

    range_ptr->range_cm = meadianFilter((int16_t)(buffer[0] | buffer[1] << 8));
    range_ptr->range_cm *= cosf(fabs(state_ptr->pitch_deg) * DEG_TO_RAD) * cosf(fabs(state_ptr->roll_deg) * DEG_TO_RAD);
}