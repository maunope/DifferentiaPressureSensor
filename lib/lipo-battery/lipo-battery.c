#include "lipo-battery.h"
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

static const char *TAG = "lipo-battery";

// For some boards (e.g., Olimex ESP32-S3-Lipo with GPIO6), adc_oneshot_io_to_channel()
// may not correctly map the GPIO to the ADC channel. Define this to hardcode the
// ADC channel if you face this issue. If this is not defined, the code will
// attempt to determine the channel dynamically from the GPIO number.
// For GPIO6 on ESP32-S3, the correct channel is ADC_CHANNEL_5.
#define BATTERY_READER_ADC_CHANNEL ADC_CHANNEL_5

static adc_oneshot_unit_handle_t s_adc_unit_handle;
static adc_cali_handle_t s_adc_cali_handle = NULL;
static adc_channel_t s_adc_channel;
static gpio_num_t s_pwr_gpio_num = -1;
static float s_voltage_divider_ratio = 2.0f; // Default to a 1:1 divider (ratio of 2.0)

// Number of ADC samples to average for a more stable reading
#define NUM_SAMPLES   3

// LiPo battery discharge curve (voltage -> percentage)
// This is a simplified table. For higher accuracy, you might need a more detailed curve.
static const int s_voltage_map[][2] = {
    {4200, 100}, // 4.20V
    {4100, 90},  // 4.10V
    {4000, 80},  // 4.00V
    {3900, 70},  // 3.90V
    {3800, 60},  // 3.80V
    {3700, 50},  // 3.70V (Nominal)
    {3600, 30},  // 3.60V
    {3500, 15},  // 3.50V
    {3300, 5},   // 3.30V
    {3200, 0}    // 3.20V (Cutoff)
};
static const int s_voltage_map_size = sizeof(s_voltage_map) / sizeof(s_voltage_map[0]);

esp_err_t battery_reader_init(gpio_num_t adc_gpio_num, gpio_num_t pwr_gpio_num, float voltage_divider_ratio) {
    // --- ADC Unit Init ---
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    adc_oneshot_new_unit(&init_config, &s_adc_unit_handle);


    // --- ADC Channel Config ---
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT, // Auto-select
        .atten = ADC_ATTEN_DB_0, // 0-0.95V range
    };
    
#ifdef BATTERY_READER_ADC_CHANNEL
    s_adc_channel = BATTERY_READER_ADC_CHANNEL;
    ESP_LOGW(TAG, "Using hardcoded ADC channel %d for battery reading.", s_adc_channel);
#else
    ESP_ERROR_CHECK(adc_oneshot_io_to_channel(adc_gpio_num, ADC_UNIT_1, &s_adc_channel));
#endif
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc_unit_handle, s_adc_channel, &config));

    ESP_LOGI(TAG, "Configured ADC channel: %d", s_adc_channel);

    // --- ADC Calibration (for more accurate voltage readings) ---
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .chan = s_adc_channel,
        .atten = ADC_ATTEN_DB_0,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    esp_err_t ret = adc_cali_create_scheme_curve_fitting(&cali_config, &s_adc_cali_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ADC calibration scheme creation failed %s", esp_err_to_name(ret));
    }

    // Configure the power sense pin as an input
    s_pwr_gpio_num = pwr_gpio_num;
    if (s_pwr_gpio_num != -1) {
        gpio_set_direction(s_pwr_gpio_num, GPIO_MODE_INPUT);
        // The pin is externally pulled down, so no internal pull-up/down is needed.
    }

       /*
    // Two-Point Calibration (uncomment this for classic ESP32)
    adc_cali_two_point_config_t cali_config_tp = { 
        // ... config settings ...
    };
    ret = adc_cali_create_scheme_two_point(&cali_config_tp, &s_adc_cali_handle);
    */

    s_voltage_divider_ratio = voltage_divider_ratio;
    ESP_LOGI(TAG, "Battery reader initialized on GPIO %d, divider ratio: %.2f", adc_gpio_num, s_voltage_divider_ratio);
    return ESP_OK;
}

float battery_reader_get_voltage(void) {
    int adc_raw_sum = 0;
    for (int i = 0; i < NUM_SAMPLES; i++) {
        int adc_raw;
   
        esp_err_t ret = adc_oneshot_read(s_adc_unit_handle, s_adc_channel, &adc_raw);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "ADC read failed: %s", esp_err_to_name(ret));
            return 0.0f;
        }
        adc_raw_sum += adc_raw;
    }
    int adc_raw_avg = adc_raw_sum / NUM_SAMPLES;

  
    int voltage_mv = 0;
    if (s_adc_cali_handle) {
        esp_err_t ret = adc_cali_raw_to_voltage(s_adc_cali_handle, adc_raw_avg, &voltage_mv);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "ADC calibration failed");
            return 0.0f;
        }
    } else {
        // Fallback if calibration is not available
        // This is a rough approximation for 0dB attenuation.
        // The full-scale voltage is ~950mV.
        voltage_mv = adc_raw_avg * 950 / 4095;
    }

    // Account for the voltage divider
    float battery_voltage = (float)(voltage_mv * s_voltage_divider_ratio) / 1000.0f;
    return battery_voltage;
}

int battery_reader_get_percentage(void) {
    float voltage = battery_reader_get_voltage();
    int voltage_mv = (int)(voltage * 1000);

    if (voltage_mv <= s_voltage_map[s_voltage_map_size - 1][0]) {
        return 0;
    }
    if (voltage_mv >= s_voltage_map[0][0]) {
        return 100;
    }

    for (int i = 0; i < s_voltage_map_size - 1; i++) {
        if (voltage_mv >= s_voltage_map[i + 1][0]) {
            // Linear interpolation between two points
            return s_voltage_map[i + 1][1] + 
                   ((voltage_mv - s_voltage_map[i + 1][0]) * (s_voltage_map[i][1] - s_voltage_map[i + 1][1])) / 
                   (s_voltage_map[i][0] - s_voltage_map[i + 1][0]);
        }
    }
    return 0; // Should not be reached
}

int battery_is_externally_powered(void) 
{
    if (s_pwr_gpio_num != -1) {
        return gpio_get_level(s_pwr_gpio_num);
    }
    return 0;
}