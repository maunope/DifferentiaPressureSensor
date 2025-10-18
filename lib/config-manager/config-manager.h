#pragma once

#include "esp_err.h"
#include <stddef.h>

#define CONFIG_STRING_MAX_LEN 256

/**
 * @brief Initializes the configuration manager.
 *
 * This function initializes the underlying NVS (Non-Volatile Storage) system.
 * It must be called once before any other config functions are used.
 *
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t config_init(void);

/**
 * @brief Loads configuration from an .ini file on the SD card into flash (NVS).
 *
 * This function reads all key-value pairs from the specified file and writes
 * them to the NVS partition in an atomic operation.
 *
 * @param config_filepath Path to the .ini file on the SD card (e.g., "/sdcard/config.ini").
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t config_load_from_sdcard_to_flash(const char* config_filepath);

/**
 * @brief Gets an integer value from the configuration.
 *
 * @param key The key for the desired value.
 * @param default_val The value to return if the key is not found.
 * @return The integer value from flash, or default_val if not found.
 */
int config_get_int(const char* key, int default_val);

/**
 * @brief Gets a float value from the configuration.
 *
 * @param key The key for the desired value.
 * @param default_val The value to return if the key is not found.
 * @return The float value from flash, or default_val if not found.
 */
float config_get_float(const char* key, float default_val);

/**
 * @brief Gets a string value from the configuration.
 *
 * If the key is not found, the output buffer will contain an empty string.
 *
 * @param key The key for the desired value.
 * @param default_val The string to return if the key is not found.
 * @param out_buf Buffer to copy the string into.
 * @param buf_len Size of the output buffer.
 */
void config_get_string(const char* key, const char* default_val, char* out_buf, size_t buf_len);