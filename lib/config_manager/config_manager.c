#include "config_manager.h"
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"

static const char *TAG = "ConfigManager";
static const char *NVS_NAMESPACE = "config";

// --- Helper Functions ---

/**
 * @brief Trims leading and trailing whitespace from a string.
 */
static char *trim_whitespace(char *str)
{
    char *end;
    // Trim leading space
    while (isspace((unsigned char)*str))
        str++;

    if (*str == 0) // All spaces?
        return str;

    // Trim trailing space
    end = str + strlen(str) - 1;
    while (end > str && isspace((unsigned char)*end))
        end--;

    // Write new null terminator
    *(end + 1) = '\0';

    return str;
}

/**
 * @brief Parses a single line and stores the key-value pair in NVS.
 */
static esp_err_t parse_and_store_line(nvs_handle_t nvs_handle, char *line)
{
    ESP_LOGI(TAG, "Input line for parsing: '%s'", line);

    // Find the first '=' character which separates the key and value.
    char *separator = strchr(line, '=');
    if (separator == NULL)
    {
        ESP_LOGE(TAG, "  -> Parsing error: No '=' separator found.");
        return ESP_ERR_INVALID_ARG;
    }

    // Null-terminate the key part and get a pointer to the value part.
    *separator = '\0';
    char *key = trim_whitespace(line);
    char *value = trim_whitespace(separator + 1);

    if (strlen(key) > 0 && strlen(value) > 0)
    {
        ESP_LOGI(TAG, "  -> Parsed and storing to NVS: key=[%s], value=[%s]", key, value);
        esp_err_t err= nvs_set_str(nvs_handle, key, value);
        ESP_LOGI(TAG, "  -> nvs_set_str result: %s", esp_err_to_name(err));
        return err;
    }
    else
    {

        ESP_LOGW(TAG, "  -> Parsing resulted in empty key or value after trim.");
    }

    return ESP_ERR_INVALID_ARG;
}

// --- Public API Implementation ---

esp_err_t config_init(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    return ESP_OK;
}

esp_err_t config_load_from_sdcard_to_flash(const char *config_filepath)
{
    FILE *f = fopen(config_filepath, "r");
    if (f == NULL)
    {
        ESP_LOGE(TAG, "Failed to open config file: %s", config_filepath);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Loading config from %s and writing to NVS...", config_filepath);

    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to open NVS namespace for writing: %s", esp_err_to_name(err));
        fclose(f);
        return err;
    }

    char line[256];
    ESP_LOGD(TAG, "Processing config file line by line...");
    while (fgets(line, sizeof(line), f))
    {
        char *trimmed_line = trim_whitespace(line);
        ESP_LOGD(TAG, "config_load_from_sdcard_to_flash: After trim: '%s'", trimmed_line);

        // Ignore comments and empty lines
        if (trimmed_line[0] == '#' || trimmed_line[0] == ';' || trimmed_line[0] == '\0' || trimmed_line[0] == '[')
        {
            continue;
        }
        ESP_LOGI(TAG, "config_load_from_sdcard_to_flash: Raw line from file: '%s'", trimmed_line);

        // strtok modifies the string, so we use a copy for parsing
        char line_copy[256];
        strncpy(line_copy, trimmed_line, sizeof(line_copy) - 1);
        line_copy[sizeof(line_copy) - 1] = '\0';

        err = parse_and_store_line(nvs_handle, line_copy);
        if (err != ESP_OK)
        {
            ESP_LOGW(TAG, "Failed to store line in NVS: '%s'", trimmed_line);
            // Continue parsing other lines
        }
    }

    fclose(f);

    // Commit all changes to flash
    err = nvs_commit(nvs_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to commit NVS changes: %s", esp_err_to_name(err));
    }
    else
    {
        ESP_LOGI(TAG, "Configuration from SD card has been processed and committed to flash.");
    }

    nvs_close(nvs_handle);
    return err;
}

int config_get_int(const char *key, int default_val)
{
    char buf[32];
    // Get the value as a string first.
    config_get_string(key, "", buf, sizeof(buf));
    // If a value was retrieved, convert it to an integer.
    if (strlen(buf) > 0)
    {
        return atoi(buf);
    }
    // Otherwise, return the default.
    return default_val;
}

float config_get_float(const char *key, float default_val)
{
    char buf[32];
    config_get_string(key, "", buf, sizeof(buf));
    if (strlen(buf) > 0)
    {
        return atof(buf);
    }
    return default_val;
}

void config_get_string(const char *key, const char *default_val, char *out_buf, size_t buf_len)
{
    nvs_handle_t nvs_handle = 0; // Initialize to 0
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to open NVS for reading: %s", esp_err_to_name(err));
        goto set_default;
    }

    ESP_LOGD(TAG, "Retrieving string for key '%s' from NVS", key);
    size_t required_size = 0;
    // First, get the required size
    err = nvs_get_str(nvs_handle, key, NULL, &required_size);
    if (err != ESP_OK)
    {
        if (err == ESP_ERR_NVS_NOT_FOUND)
        {
            ESP_LOGD(TAG, "Key '%s' not found in NVS, using default value: '%s'", key, default_val);
        }
        else
        {
            ESP_LOGW(TAG, "NVS get_str (size check) failed for key '%s': %s", key, esp_err_to_name(err));
        }
        goto set_default;
    }

    if (required_size > buf_len)
    {
        ESP_LOGW(TAG, "Buffer too small for key '%s'. Required: %zu, Available: %zu", key, required_size, buf_len);
        goto set_default;
    }

    err = nvs_get_str(nvs_handle, key, out_buf, &required_size);
    if (err == ESP_OK)
    {
        ESP_LOGD(TAG, "Retrieved string for key '%s': '%s'", key, out_buf);
    }
    else
    { // Should not happen if size check passed, but for safety
        ESP_LOGW(TAG, "NVS get_str failed for key '%s': %s", key, esp_err_to_name(err));
        goto set_default;
    }

    nvs_close(nvs_handle);
    return;

set_default:
    strncpy(out_buf, default_val, buf_len - 1);
    out_buf[buf_len - 1] = '\0';
    if (nvs_handle)
    {
        nvs_close(nvs_handle);
    }
}
