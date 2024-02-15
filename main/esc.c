#include "esc.h"
#include "driver/rmt_tx.h"
#include "dshot_esc_encoder.h"


static rmt_channel_handle_t esc_channel_1 = NULL;
static rmt_channel_handle_t esc_channel_2 = NULL;
static rmt_channel_handle_t esc_channel_3 = NULL;
static rmt_channel_handle_t esc_channel_4 = NULL;

static rmt_encoder_handle_t dshot_encoder_1 = NULL;
static rmt_encoder_handle_t dshot_encoder_2 = NULL;
static rmt_encoder_handle_t dshot_encoder_3 = NULL;
static rmt_encoder_handle_t dshot_encoder_4 = NULL;

static dshot_esc_throttle_t throttle_1;
static dshot_esc_throttle_t throttle_2;
static dshot_esc_throttle_t throttle_3;
static dshot_esc_throttle_t throttle_4;

static rmt_transmit_config_t tx_config = 
{
    .loop_count = -1, // infinite loop
};


void dshot_esc_init()
{

    throttle_1.throttle = 0;
    throttle_1.telemetry_req = false;

    throttle_2.throttle = 0;
    throttle_2.telemetry_req = false;

    throttle_3.throttle = 0;
    throttle_3.telemetry_req = false;

    throttle_4.throttle = 0;
    throttle_4.telemetry_req = false;

    rmt_tx_channel_config_t tx_chan_config_1 = 
    {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .gpio_num = MOTOR_LB_PIN,
        .mem_block_symbols = 64,
        .resolution_hz = DSHOT_ESC_RESOLUTION_HZ,
        .trans_queue_depth = 10,
    };

    rmt_tx_channel_config_t tx_chan_config_2 = 
    {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .gpio_num = MOTOR_LT_PIN,
        .mem_block_symbols = 64,
        .resolution_hz = DSHOT_ESC_RESOLUTION_HZ,
        .trans_queue_depth = 10,
    };

    rmt_tx_channel_config_t tx_chan_config_3 = 
    {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .gpio_num = MOTOR_RB_PIN,
        .mem_block_symbols = 64,
        .resolution_hz = DSHOT_ESC_RESOLUTION_HZ,
        .trans_queue_depth = 10,
    };

    rmt_tx_channel_config_t tx_chan_config_4 = 
    {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .gpio_num = MOTOR_RT_PIN,
        .mem_block_symbols = 64,
        .resolution_hz = DSHOT_ESC_RESOLUTION_HZ,
        .trans_queue_depth = 10,
    };

    rmt_new_tx_channel(&tx_chan_config_1, &esc_channel_1);
    rmt_new_tx_channel(&tx_chan_config_2, &esc_channel_2);
    rmt_new_tx_channel(&tx_chan_config_3, &esc_channel_3);
    rmt_new_tx_channel(&tx_chan_config_4, &esc_channel_4);

    dshot_esc_encoder_config_t encoder_config = 
    {
        .resolution = DSHOT_ESC_RESOLUTION_HZ,
        .baud_rate = 300000, // DSHOT300 protocol
        .post_delay_us = 50, // extra delay between each frame
    };

    rmt_new_dshot_esc_encoder(&encoder_config, &dshot_encoder_1);
    rmt_new_dshot_esc_encoder(&encoder_config, &dshot_encoder_2);
    rmt_new_dshot_esc_encoder(&encoder_config, &dshot_encoder_3);
    rmt_new_dshot_esc_encoder(&encoder_config, &dshot_encoder_4);

    rmt_enable(esc_channel_1);
    rmt_enable(esc_channel_2);
    rmt_enable(esc_channel_3);
    rmt_enable(esc_channel_4);

    rmt_transmit(esc_channel_1, dshot_encoder_1, &throttle_1, sizeof(throttle_1), &tx_config);
    rmt_transmit(esc_channel_2, dshot_encoder_2, &throttle_2, sizeof(throttle_2), &tx_config);
    rmt_transmit(esc_channel_3, dshot_encoder_3, &throttle_3, sizeof(throttle_3), &tx_config);
    rmt_transmit(esc_channel_4, dshot_encoder_4, &throttle_4, sizeof(throttle_4), &tx_config);
}

void write_throttle(uint16_t thr1, uint16_t thr2, uint16_t thr3, uint16_t thr4)
{
    throttle_1.throttle = thr1;
    throttle_2.throttle = thr2;
    throttle_3.throttle = thr3;
    throttle_4.throttle = thr4;
    
    rmt_transmit(esc_channel_1, dshot_encoder_1, &throttle_1, sizeof(throttle_1), &tx_config);
    rmt_transmit(esc_channel_2, dshot_encoder_2, &throttle_2, sizeof(throttle_2), &tx_config);
    rmt_transmit(esc_channel_3, dshot_encoder_3, &throttle_3, sizeof(throttle_3), &tx_config);
    rmt_transmit(esc_channel_4, dshot_encoder_4, &throttle_4, sizeof(throttle_4), &tx_config);

    rmt_disable(esc_channel_1);
    rmt_disable(esc_channel_2);
    rmt_disable(esc_channel_3);
    rmt_disable(esc_channel_4);

    rmt_enable(esc_channel_1);
    rmt_enable(esc_channel_2);
    rmt_enable(esc_channel_3);
    rmt_enable(esc_channel_4);
}