#include "gpio.h"
#include "driver/ledc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

adc_cali_handle_t adc1_cali_chan6_handle = NULL;
adc_oneshot_unit_handle_t adc1_handle;

static void adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten);

void gpio_configure()
{
    adc_oneshot_unit_init_cfg_t init_config1 =
        {
            .unit_id = ADC_UNIT_1,
        };
    adc_oneshot_new_unit(&init_config1, &adc1_handle);

    adc_oneshot_chan_cfg_t config =
        {
            .bitwidth = ADC_BITWIDTH_DEFAULT,
            .atten = ADC_ATTEN_DB_11,
        };
    adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_6, &config);
    adc_calibration_init(ADC_UNIT_1, ADC_CHANNEL_6, ADC_ATTEN_DB_11);


    ledc_timer_config_t timer_conf = 
    {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution    = LEDC_TIMER_10_BIT,
        .timer_num  = LEDC_TIMER_0,
        .freq_hz    = 1000
    };
	ledc_timer_config(&timer_conf);

	ledc_channel_config_t ledc_conf =
    {
        .gpio_num   = BUZZ_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_0,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 0,
    };
	ledc_channel_config(&ledc_conf);

}


void start_beep(uint32_t freq)
{
    ledc_set_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0, freq);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 512);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

void stop_beep()
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

static void adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten)
{
    adc_cali_line_fitting_config_t cali_config =
        {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
    adc_cali_create_scheme_line_fitting(&cali_config, &adc1_cali_chan6_handle);
}

float get_bat_volt()
{
    static int adc_6_raw = 0;
    static int volt_raw = 0;
    adc_oneshot_read(adc1_handle, ADC_CHANNEL_6, &adc_6_raw);
    adc_cali_raw_to_voltage(adc1_cali_chan6_handle, adc_6_raw, &volt_raw);
    return (float)(volt_raw / 1000.0f);
}

