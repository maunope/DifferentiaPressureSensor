#include "config_params.h"
#include "../lib/config_manager/config_manager.h"
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
    .battery_voltage_treshold = 3.55f,
    .inactivity_timeout_ms = 30000,
    .sleep_duration_ms = 60000,
    .log_interval_ms = 59500,
    .wifi_ssid = "",
    .wifi_password = "",
    .d6fph_model = D6FPH_MODEL_0505AD3,
    .hf_sleep_duration_ms = 5000,
    .hf_log_interval_ms = 4900,
    .kf_temp_q = 0.01f,       // Process noise for temperature (low, as it changes slowly)
    .kf_temp_r = 0.5f,        // Measurement noise for temperature
    .kf_press_q = 0.01f,      // Process noise for pressure
    .kf_press_r = 0.5f,       // Measurement noise for pressure
    .kf_diff_press_q = 0.1f,  // Process noise for differential pressure (can be a bit more noisy)
    .kf_diff_press_r = 4.0f,  // Measurement noise for differential pressure
    .kf_batt_v_q = 0.001f,    // Process noise for battery voltage (very slow change)
    .kf_batt_v_r = 0.1f,      // Measurement noise for battery voltage
    // High-frequency defaults. 'r' (measurement noise) is sensor-dependent and should be the same as normal mode.
    // 'q' (process noise) is scaled down because there is less time for the value to change between samples (5s vs 60s -> 12x).
    .kf_temp_q_hf = 0.01f / 12.0f,       // kf_temp_q / 12
    .kf_temp_r_hf = 0.5f,                 // kf_temp_r
    .kf_press_q_hf = 0.01f / 12.0f,      // kf_press_q / 12
    .kf_press_r_hf = 0.5f,                // kf_press_r
    .kf_diff_press_q_hf = 0.1f / 12.0f,  // kf_diff_press_q / 12
    .kf_diff_press_r_hf = 4.0f,           // kf_diff_press_r
    .kf_batt_v_q_hf = 0.001f / 12.0f,    // kf_batt_v_q / 12
    .kf_batt_v_r_hf = 0.1f,               // kf_batt_v_r
};

void config_params_init(void) {
    ESP_LOGI(TAG, "Loading configuration from flash...");

    // Load values from NVS, using defaults if a key is not found.
    g_config_params.battery_voltage_divider_ratio = config_get_float("v_div_ratio", s_default_params.battery_voltage_divider_ratio); // Index 0
    g_config_params.battery_voltage_treshold = config_get_float("b_v_thresh", s_default_params.battery_voltage_treshold);     // Index 1
    g_config_params.inactivity_timeout_ms = config_get_int("inactive_ms", s_default_params.inactivity_timeout_ms);                 // Index 2
    g_config_params.sleep_duration_ms = config_get_int("sleep_ms", s_default_params.sleep_duration_ms);                             // Index 3
    g_config_params.log_interval_ms = config_get_int("log_int_ms", s_default_params.log_interval_ms);                             // Index 4
    config_get_string("ssid", s_default_params.wifi_ssid, g_config_params.wifi_ssid, sizeof(g_config_params.wifi_ssid));           // Index 5
    config_get_string("pass", s_default_params.wifi_password, g_config_params.wifi_password, sizeof(g_config_params.wifi_password)); // Index 6
    g_config_params.d6fph_model = (d6fph_sensor_model_t)config_get_int("d6fph_model", s_default_params.d6fph_model);                 // Index 7
    g_config_params.hf_sleep_duration_ms = config_get_int("hf_sleep_ms", s_default_params.hf_sleep_duration_ms);                     // Index 8
    g_config_params.hf_log_interval_ms = config_get_int("hf_log_int_ms", s_default_params.hf_log_interval_ms);                     // Index 9
    g_config_params.kf_temp_q = config_get_float("kf_temp_q", s_default_params.kf_temp_q);
    g_config_params.kf_temp_r = config_get_float("kf_temp_r", s_default_params.kf_temp_r);
    g_config_params.kf_press_q = config_get_float("kf_press_q", s_default_params.kf_press_q);
    g_config_params.kf_press_r = config_get_float("kf_press_r", s_default_params.kf_press_r);
    g_config_params.kf_diff_press_q = config_get_float("kf_diff_press_q", s_default_params.kf_diff_press_q);
    g_config_params.kf_diff_press_r = config_get_float("kf_diff_press_r", s_default_params.kf_diff_press_r);
    g_config_params.kf_batt_v_q = config_get_float("kf_batt_v_q", s_default_params.kf_batt_v_q);
    g_config_params.kf_batt_v_r = config_get_float("kf_batt_v_r", s_default_params.kf_batt_v_r);
    // Load HF Kalman parameters
    g_config_params.kf_temp_q_hf = config_get_float("kf_temp_q_hf", s_default_params.kf_temp_q_hf);
    g_config_params.kf_temp_r_hf = config_get_float("kf_temp_r_hf", s_default_params.kf_temp_r_hf);
    g_config_params.kf_press_q_hf = config_get_float("kf_press_q_hf", s_default_params.kf_press_q_hf);
    g_config_params.kf_press_r_hf = config_get_float("kf_press_r_hf", s_default_params.kf_press_r_hf);
    g_config_params.kf_diff_press_q_hf = config_get_float("kf_diff_press_q_hf", s_default_params.kf_diff_press_q_hf);
    g_config_params.kf_diff_press_r_hf = config_get_float("kf_diff_press_r_hf", s_default_params.kf_diff_press_r_hf);
    g_config_params.kf_batt_v_q_hf = config_get_float("kf_batt_v_q_hf", s_default_params.kf_batt_v_q_hf);
    g_config_params.kf_batt_v_r_hf = config_get_float("kf_batt_v_r_hf", s_default_params.kf_batt_v_r_hf);

    ESP_LOGI(TAG, "--- Loaded Configuration ---");
    ESP_LOGI(TAG, "v_div_ratio: %.4f", g_config_params.battery_voltage_divider_ratio);
    ESP_LOGI(TAG, "b_v_thresh: %.2f", g_config_params.battery_voltage_treshold);
    ESP_LOGI(TAG, "inactive_ms: %lu", (unsigned long)g_config_params.inactivity_timeout_ms);
    ESP_LOGI(TAG, "sleep_ms: %llu", g_config_params.sleep_duration_ms);
    ESP_LOGI(TAG, "log_int_ms: %lu", (unsigned long)g_config_params.log_interval_ms);
    ESP_LOGI(TAG, "hf_sleep_ms: %llu", g_config_params.hf_sleep_duration_ms);
    ESP_LOGI(TAG, "hf_log_int_ms: %lu", (unsigned long)g_config_params.hf_log_interval_ms);
    ESP_LOGI(TAG, "ssid: '%s'", g_config_params.wifi_ssid);
    ESP_LOGI(TAG, "pass: %s", strlen(g_config_params.wifi_password) > 0 ? "(set)" : "(not set)");
    ESP_LOGI(TAG, "d6fph_model: %d", g_config_params.d6fph_model);
    ESP_LOGI(TAG, "--- Kalman Filter ---");
    ESP_LOGI(TAG, "Temp (Q/R): %.4f/%.4f", g_config_params.kf_temp_q, g_config_params.kf_temp_r);
    ESP_LOGI(TAG, "Press (Q/R): %.4f/%.4f", g_config_params.kf_press_q, g_config_params.kf_press_r);
    ESP_LOGI(TAG, "Diff.Press (Q/R): %.4f/%.4f", g_config_params.kf_diff_press_q, g_config_params.kf_diff_press_r);
    ESP_LOGI(TAG, "Batt. V (Q/R): %.4f/%.4f", g_config_params.kf_batt_v_q, g_config_params.kf_batt_v_r);
    ESP_LOGI(TAG, "--- Kalman Filter (HF) ---");
    ESP_LOGI(TAG, "Temp (Q/R): %.4f/%.4f", g_config_params.kf_temp_q_hf, g_config_params.kf_temp_r_hf);
    ESP_LOGI(TAG, "Press (Q/R): %.4f/%.4f", g_config_params.kf_press_q_hf, g_config_params.kf_press_r_hf);
    ESP_LOGI(TAG, "Diff.Press (Q/R): %.4f/%.4f", g_config_params.kf_diff_press_q_hf, g_config_params.kf_diff_press_r_hf);
    ESP_LOGI(TAG, "Batt. V (Q/R): %.4f/%.4f", g_config_params.kf_batt_v_q_hf, g_config_params.kf_batt_v_r_hf);
    ESP_LOGI(TAG, "--------------------------");
}

const config_params_t* config_params_get(void) {
    // Return a const pointer to prevent modification outside of this module.
    return &g_config_params;
}