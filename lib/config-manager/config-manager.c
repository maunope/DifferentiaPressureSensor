#include "config-manager.h"
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"

static const char *TAG = "ConfigManager";
static const char *NVS_NAMESPACE = "app_config";

// --- Helper Functions ---

/**
 * @brief Trims leading and trailing whitespace from a string.
 */
static char* trim_whitespace(char *str) {
    char *end;
    while (isspace((unsigned char)*str)) str++;
    if (*str == 0) return str;
    end = str + strlen(str) - 1;
    while (end > str && isspace((unsigned char)*end)) end--;
    
    // Add null terminator
    *(end + 1) = '\0';

    return str;
}

/**
 * @brief Parses a single line and stores the key-value pair in NVS.
 */
static esp_err_t parse_and_store_line(nvs_handle_t nvs_handle, char *line)
{
    char *key = strtok(line, "=");
    char *value = strtok(NULL, "\n\r"); // Stop at newline or carriage return

    if (key && value) {
        key = trim_whitespace(key);
        value = trim_whitespace(value);
        if (strlen(key) > 0) {
            ESP_LOGI(TAG, "Storing to NVS: key='%s', value='%s'", key, value);
            return nvs_set_str(nvs_handle, key, value);
        }
    }
    return ESP_ERR_INVALID_ARG;
}

// --- Public API Implementation ---

esp_err_t config_init(void) {
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    return ESP_OK;
}

esp_err_t config_load_from_sdcard_to_flash(const char* config_filepath) {
    FILE *f = fopen(config_filepath, "r");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open config file: %s", config_filepath);
        return ESP_FAIL;
    }

    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS namespace for writing: %s", esp_err_to_name(err));
        fclose(f);
        return err;
    }

    // Erase the namespace to ensure a clean slate before loading new config
    nvs_erase_all(nvs_handle);

    char line[256];
    while (fgets(line, sizeof(line), f)) {
        char* trimmed_line = trim_whitespace(line);

        // Ignore comments and empty lines
        if (trimmed_line[0] == '#' || trimmed_line[0] == ';' || trimmed_line[0] == '\0' || trimmed_line[0] == '[') {
            continue;
        }
        err = parse_and_store_line(nvs_handle, trimmed_line);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to store line in NVS: %s", esp_err_to_name(err));
            // Continue parsing other lines
        }
    }

    fclose(f);

    // Commit all changes to flash
    err = nvs_commit(nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to commit NVS changes: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Configuration from SD card loaded and committed to flash.");
    }
    
    nvs_close(nvs_handle);
    return err;
}

int config_get_int(const char* key, int default_val) {
    char buf[32];
    // Get the value as a string first.
    config_get_string(key, "", buf, sizeof(buf));
    // If a value was retrieved, convert it to an integer.
    if (strlen(buf) > 0) {
        return atoi(buf);
    }
    // Otherwise, return the default.
    return default_val;
}

float config_get_float(const char* key, float default_val) {
    char buf[32];
    config_get_string(key, "", buf, sizeof(buf));
    if (strlen(buf) > 0) {
        return atof(buf);
    }
    return default_val;
}

void config_get_string(const char* key, const char* default_val, char* out_buf, size_t buf_len) {
    nvs_handle_t nvs_handle = 0; // Initialize to 0
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS for reading: %s", esp_err_to_name(err));
        goto set_default;
    }

    ESP_LOGI(TAG, "Retrieving string for key '%s' from NVS", key);
    size_t required_size = 0;
    // First, get the required size
    err = nvs_get_str(nvs_handle, key, NULL, &required_size);
    if (err != ESP_OK) {
        if (err == ESP_ERR_NVS_NOT_FOUND) {
            ESP_LOGI(TAG, "Key '%s' not found in NVS, using default value: '%s'", key, default_val);
        } else {
            ESP_LOGW(TAG, "NVS get_str (size check) failed for key '%s': %s", key, esp_err_to_name(err));
        }
        goto set_default;
    }

    if (required_size > buf_len) {
        ESP_LOGW(TAG, "Buffer too small for key '%s'. Required: %zu, Available: %zu", key, required_size, buf_len);
        goto set_default;
    }

    err = nvs_get_str(nvs_handle, key, out_buf, &required_size);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Retrieved string for key '%s': '%s'", key, out_buf);
    } else { // Should not happen if size check passed, but for safety
        ESP_LOGW(TAG, "NVS get_str failed for key '%s': %s", key, esp_err_to_name(err));
        goto set_default;
    }

    nvs_close(nvs_handle);
    return;

set_default:
    strncpy(out_buf, default_val, buf_len - 1);
    out_buf[buf_len - 1] = '\0';
    if (nvs_handle) {
        nvs_close(nvs_handle);
    }
}