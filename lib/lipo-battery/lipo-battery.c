#include "lipo-battery.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc_cal.h" // For legacy calibration API
#include "esp_idf_version.h" // For ESP_IDF_VERSION_VAL

static const char *TAG = "lipo-battery";

static adc_oneshot_unit_handle_t s_adc_unit_handle;
static adc_cali_handle_t s_adc_cali_handle = NULL;
static adc_channel_t s_adc_channel;
static float s_voltage_divider_ratio = 2.0f; // Default to a 1:1 divider (ratio of 2)

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

esp_err_t battery_reader_init(gpio_num_t adc_gpio_num, float voltage_divider_ratio) {
    // --- ADC Unit Init ---
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &s_adc_unit_handle));

    // --- ADC Channel Config ---
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT, // Auto-select
        .atten = ADC_ATTEN_DB_11,         // 11dB attenuation for full range (up to ~3.1V)
    };
    ESP_ERROR_CHECK(adc_oneshot_io_to_channel(adc_gpio_num, ADC_UNIT_1, &s_adc_channel));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc_unit_handle, s_adc_channel, &config));

    // --- ADC Calibration (for more accurate voltage readings) ---
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .chan = s_adc_channel,
        .atten = ADC_ATTEN_DB_11,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    esp_err_t ret = adc_cali_create_scheme_curve_fitting(&cali_config, &s_adc_cali_handle);
#else // For older IDF versions
    esp_adc_cal_characteristics_t *adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_BITWIDTH_DEFAULT, 1100, adc_chars);
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        ESP_LOGI(TAG, "ADC calibrated using eFuse Vref");
        s_adc_cali_handle = (adc_cali_handle_t)adc_chars; // Store characteristics
    } else {
        ESP_LOGW(TAG, "ADC calibration not supported for this device/eFuse config.");
        free(adc_chars);
    }
    esp_err_t ret = (s_adc_cali_handle != NULL) ? ESP_OK : ESP_FAIL;
#endif
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "ADC Calibration failed, readings will be less accurate.");
    }
    s_voltage_divider_ratio = voltage_divider_ratio;
    ESP_LOGI(TAG, "Battery reader initialized on GPIO %d, divider ratio: %.2f", adc_gpio_num, s_voltage_divider_ratio);
    return ESP_OK;
}

float battery_reader_get_voltage(void) {
    int adc_raw;
    int voltage_mv;

    esp_err_t ret = adc_oneshot_read(s_adc_unit_handle, s_adc_channel, &adc_raw);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ADC read failed");
        return 0.0f;
    }

    if (s_adc_cali_handle) {
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
        ret = adc_cali_raw_to_voltage(s_adc_cali_handle, adc_raw, &voltage_mv);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "ADC calibration failed");
            return 0.0f;
        }
#else
        voltage_mv = esp_adc_cal_raw_to_voltage(adc_raw, (const esp_adc_cal_characteristics_t*)s_adc_cali_handle);
#endif
    } else {
        // Fallback if calibration is not available
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
        voltage_mv = adc_raw * 2450 / 4095; // Approximate for 11dB attenuation
#endif
    }

    // Account for the voltage divider
    float battery_voltage = (float)voltage_mv * s_voltage_divider_ratio / 1000.0f;
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