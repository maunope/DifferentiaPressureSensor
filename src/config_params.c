#include "config_params.h"
#include "../lib/config-manager/config-manager.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "ConfigParams";

// Global instance of the configuration parameters
static config_params_t g_config_params;

/**
 * @brief Default configuration values.
 * These are used if no values are found in NVS.
 */
static const config_params_t s_default_params = {
    .battery_voltage_divider_ratio = 4.1428f,
    .inactivity_timeout_ms = 30000,
    .sleep_duration_ms = 60000,
    .log_interval_ms = 59500
};

void config_params_init(void) {
    ESP_LOGI(TAG, "Loading configuration from flash...");

    // Load values from NVS, using defaults if a key is not found.
    g_config_params.battery_voltage_divider_ratio = config_get_float("system.BATTERY_VOLTAGE_DIVIDER_RATIO", s_default_params.battery_voltage_divider_ratio);
    g_config_params.inactivity_timeout_ms = config_get_int("system.INACTIVITY_TIMEOUT_MS", s_default_params.inactivity_timeout_ms);
    g_config_params.sleep_duration_ms = config_get_int("system.SLEEP_DURATION_MS", s_default_params.sleep_duration_ms);
    g_config_params.log_interval_ms = config_get_int("system.LOG_INTERVAL_MS", s_default_params.log_interval_ms);

    ESP_LOGI(TAG, "--- Loaded Configuration ---");
    ESP_LOGI(TAG, "BATTERY_VOLTAGE_DIVIDER_RATIO: %.4f", g_config_params.battery_voltage_divider_ratio);
    ESP_LOGI(TAG, "INACTIVITY_TIMEOUT_MS: %lu", (unsigned long)g_config_params.inactivity_timeout_ms);
    ESP_LOGI(TAG, "SLEEP_DURATION_MS: %llu", g_config_params.sleep_duration_ms);
    ESP_LOGI(TAG, "LOG_INTERVAL_MS: %lu", (unsigned long)g_config_params.log_interval_ms);
    ESP_LOGI(TAG, "--------------------------");
}

const config_params_t* config_params_get(void) {
    // Return a const pointer to prevent modification outside of this module.
    return &g_config_params;
}