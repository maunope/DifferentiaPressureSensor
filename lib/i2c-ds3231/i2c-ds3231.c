#include "i2c-ds3231.h"
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <time.h>
#include <sys/time.h>

#define DS3231_ADDR 0x68
static const char* TAG = "ds3231";

/**
 * @brief Converts a BCD (Binary-Coded Decimal) value to a decimal value.
 * @param val The BCD value to convert.
 * @return uint8_t The decimal equivalent.
 */
static uint8_t bcd2dec(uint8_t val) { return ((val >> 4) * 10 + (val & 0x0F)); }

/**
 * @brief Converts a decimal value to a BCD (Binary-Coded Decimal) value.
 * @param val The decimal value to convert.
 * @return uint8_t The BCD equivalent.
 */
static uint8_t dec2bcd(uint8_t val) { return ((val / 10) << 4) | (val % 10); }

/**
 * @brief Initializes the DS3231 device descriptor.
 * @param rtc Pointer to the DS3231 device descriptor to initialize.
 * @param i2c_num The I2C port number to use.
 * @param address The I2C address of the DS3231.
 * @return esp_err_t `ESP_OK` on success, `ESP_ERR_INVALID_ARG` if rtc is NULL.
 */
esp_err_t ds3231_init(ds3231_t *rtc, i2c_master_bus_handle_t bus_handle, uint8_t address) {
    if (!rtc) {
        return ESP_ERR_INVALID_ARG;
    }
    if (address == 0) {
        address = DS3231_I2C_ADDR_DEFAULT; // Use default if 0 is passed
    }
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_7,
        .device_address = address,
        .scl_speed_hz = 100000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &rtc->i2c_dev_handle));
    return ESP_OK;
}

/**
 * @brief Gets the current time from the DS3231 RTC.
 *
 * Reads the time registers from the RTC and populates a `struct tm`.
 * @param rtc Pointer to the initialized DS3231 device descriptor.
 * @param timeinfo Pointer to a `struct tm` to store the retrieved time (in UTC).
 * @return esp_err_t `ESP_OK` on success, or an I2C communication error code on failure.
 */
esp_err_t ds3231_get_time(ds3231_t *rtc, struct tm *timeinfo) {
    uint8_t data[7];
    uint8_t reg_addr = 0x00;

    esp_err_t ret;
    //F*ck those cheap *ss modules from Aliexpress, sometimes it NAKs the first read attempt
    int retries = 3;
    do {
        ret = i2c_master_transmit_receive(rtc->i2c_dev_handle, &reg_addr, 1, data, 7, pdMS_TO_TICKS(500));
        if (ret == ESP_OK) break;
        vTaskDelay(pdMS_TO_TICKS(10)); // Small delay between retries
    } while (--retries > 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get time after multiple retries: %s", esp_err_to_name(ret));
        return ret;
    }

    timeinfo->tm_sec  = bcd2dec(data[0]);
    timeinfo->tm_min  = bcd2dec(data[1]);
    timeinfo->tm_hour = bcd2dec(data[2]);
    timeinfo->tm_mday = bcd2dec(data[4]);
    timeinfo->tm_mon  = bcd2dec(data[5]) - 1;
    timeinfo->tm_year = bcd2dec(data[6]) + 100; // Years since 1900, DS3231 gives YY
    return ESP_OK;
}

/**
 * @brief Sets the time on the DS3231 RTC.
 *
 * Writes the time from a `struct tm` to the RTC's time registers.
 * @param rtc Pointer to the initialized DS3231 device descriptor.
 * @param timeinfo Pointer to a `struct tm` containing the time to set (in UTC).
 * @return esp_err_t `ESP_OK` on success, or an I2C communication error code on failure.
 */
esp_err_t ds3231_set_time(ds3231_t *rtc, const struct tm *timeinfo) {
    uint8_t data[7];
    data[0] = dec2bcd(timeinfo->tm_sec);
    data[1] = dec2bcd(timeinfo->tm_min);
    data[2] = dec2bcd(timeinfo->tm_hour);
    data[3] = 0; // Day of week not used
    data[4] = dec2bcd(timeinfo->tm_mday);
    data[5] = dec2bcd(timeinfo->tm_mon + 1);
    data[6] = dec2bcd(timeinfo->tm_year % 100);

    uint8_t write_buf[8] = {0x00}; // reg_addr + 7 bytes of data
    memcpy(&write_buf[1], data, 7);

    esp_err_t ret;
    int retries = 3;
    do {
        ret = i2c_master_transmit(rtc->i2c_dev_handle, write_buf, sizeof(write_buf), pdMS_TO_TICKS(500));
        if (ret == ESP_OK) break;
        vTaskDelay(pdMS_TO_TICKS(10)); // Small delay between retries
    } while (--retries > 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set time after multiple retries: %s", esp_err_to_name(ret));
    }
    return ret;
}

/**
 * @brief Sets the time on the DS3231 RTC from a `time_t` timestamp.
 *
 * Converts a `time_t` (seconds since epoch) into a `struct tm` and sets the RTC.
 * @param rtc Pointer to the initialized DS3231 device descriptor.
 * @param timestamp The `time_t` timestamp (in UTC) to set.
 * @return esp_err_t `ESP_OK` on success, or an error code on failure.
 */
esp_err_t ds3231_set_time_t(ds3231_t *rtc, time_t timestamp) {
    struct tm timeinfo;
    // The timestamp is UTC (seconds since epoch).
    // We need to convert it to a struct tm representing UTC.
    gmtime_r(&timestamp, &timeinfo);
    return ds3231_set_time(rtc, &timeinfo);
}

/**
 * @brief Sets the RTC time to the application's build time.
 *
 * This function parses the `__DATE__` and `__TIME__` macros to determine the
 * build timestamp, sets the RTC to this time (in UTC), and updates the system time.
 * It handles timezone conversion from the local build environment to UTC.
 * @param rtc Pointer to the DS3231 device descriptor.
 * @return `ESP_OK` on success, or an error code on failure.
 */
esp_err_t ds3231_set_time_to_build_time(ds3231_t *rtc)
{
    if (!rtc) {
        return ESP_ERR_INVALID_ARG;
    }

    // Set the timezone to correctly interpret the local build time
    setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 1);
    tzset();

    struct tm build_time;
    const char *date_str = __DATE__; // "Mmm dd yyyy"
    const char *time_str = __TIME__; // "hh:mm:ss"

    // Parse date and time
    sscanf(time_str, "%d:%d:%d", &build_time.tm_hour, &build_time.tm_min, &build_time.tm_sec);

    char month_str[4];
    int year;
    sscanf(date_str, "%s %d %d", month_str, &build_time.tm_mday, &year);
    build_time.tm_year = year - 1900;

    const char *months[] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
    for (int i = 0; i < 12; i++)
    {
        if (strcmp(month_str, months[i]) == 0)
        {
            build_time.tm_mon = i;
            break;
        }
    }

    // Convert local build time to a UTC timestamp, then back to a UTC-based struct tm
    time_t build_ts = mktime(&build_time);
    if (ds3231_set_time_t(rtc, build_ts) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set RTC time.");
        // Restore the system timezone to UTC before returning
        setenv("TZ", "UTC0", 1);
        tzset();
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "RTC time set to build time: %s %s", date_str, time_str);
    struct timeval tv = {.tv_sec = build_ts};
    settimeofday(&tv, NULL);

    setenv("TZ", "UTC0", 1);
    tzset();
    return ESP_OK;
}