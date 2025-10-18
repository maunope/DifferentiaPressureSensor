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
    end[1] = '\0';
    return str;
}

/**
 * @brief Parses a single line and stores the key-value pair in NVS.
 */
static esp_err_t parse_and_store_line(nvs_handle_t nvs_handle, char *line) {
    char *key = strtok(line, "=");
    char *value = strtok(NULL, "");

    if (key && value) {
        key = trim_whitespace(key);
        value = trim_whitespace(value);
        ESP_LOGD(TAG, "Storing to NVS: key='%s', value='%s'", key, value);
        return nvs_set_str(nvs_handle, key, value);
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

    char line[CONFIG_STRING_MAX_LEN + 100]; // Buffer for key + value + overhead
    while (fgets(line, sizeof(line), f)) {
        // Ignore comments and empty lines
        if (line[0] == '#' || line[0] == ';' || line[0] == '\n' || line[0] == '\r') {
            continue;
        }
        err = parse_and_store_line(nvs_handle, line);
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
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) return default_val;

    int32_t value = default_val;
    err = nvs_get_i32(nvs_handle, key, &value);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW(TAG, "NVS get_i32 failed for key '%s': %s", key, esp_err_to_name(err));
    }
    nvs_close(nvs_handle);
    return value;
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
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        strncpy(out_buf, default_val, buf_len - 1);
        out_buf[buf_len - 1] = '\0';
        return;
    }

    size_t required_size = buf_len;
    err = nvs_get_str(nvs_handle, key, out_buf, &required_size);

    if (err == ESP_ERR_NVS_NOT_FOUND) {
        strncpy(out_buf, default_val, buf_len - 1);
        out_buf[buf_len - 1] = '\0';
    } else if (err != ESP_OK) {
        ESP_LOGW(TAG, "NVS get_string failed for key '%s': %s", key, esp_err_to_name(err));
        strncpy(out_buf, default_val, buf_len - 1);
        out_buf[buf_len - 1] = '\0';
    }

    nvs_close(nvs_handle);
}