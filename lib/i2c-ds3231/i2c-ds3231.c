#include "i2c-ds3231.h"
#include <string.h>
#include "esp_log.h"

#define DS3231_ADDR 0x68
static const char* TAG = "ds3231";

static uint8_t bcd2dec(uint8_t val) { return ((val >> 4) * 10 + (val & 0x0F)); }
static uint8_t dec2bcd(uint8_t val) { return ((val / 10) << 4) | (val % 10); }

esp_err_t ds3231_init(ds3231_t *rtc, i2c_port_t i2c_num, uint8_t address) {
    if (!rtc) {
        return ESP_ERR_INVALID_ARG;
    }
    rtc->i2c_num = i2c_num;
    rtc->address = address;
    return ESP_OK;
}

int ds3231_get_time(ds3231_t *rtc, struct tm *timeinfo) {
    uint8_t data[7];
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    // Write-then-read transaction
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (rtc->address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x00, true); // Start at register 0x00
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (rtc->address << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 7, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);

    esp_err_t ret;
    //F*ck those cheap *ss modules from Aliexpress, sometimes it NAKs the first read attempt
    int retries = 3;
    do {
        ret = i2c_master_cmd_begin(rtc->i2c_num, cmd, 500 / portTICK_PERIOD_MS);
        if (ret == ESP_OK) break;
        vTaskDelay(pdMS_TO_TICKS(10)); // Small delay between retries
    } while (--retries > 0);

    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get time after multiple retries: %s", esp_err_to_name(ret));
        return -1;
    }

    timeinfo->tm_sec  = bcd2dec(data[0]);
    timeinfo->tm_min  = bcd2dec(data[1]);
    timeinfo->tm_hour = bcd2dec(data[2]);
    timeinfo->tm_mday = bcd2dec(data[4]);
    timeinfo->tm_mon  = bcd2dec(data[5]) - 1;
    timeinfo->tm_year = bcd2dec(data[6]) + 100; // Years since 1900, DS3231 gives YY
    return 0;
}

int ds3231_set_time(ds3231_t *rtc, const struct tm *timeinfo) {
    uint8_t data[7];
    data[0] = dec2bcd(timeinfo->tm_sec);
    data[1] = dec2bcd(timeinfo->tm_min);
    data[2] = dec2bcd(timeinfo->tm_hour);
    data[3] = 0; // Day of week not used
    data[4] = dec2bcd(timeinfo->tm_mday);
    data[5] = dec2bcd(timeinfo->tm_mon + 1);
    data[6] = dec2bcd(timeinfo->tm_year % 100);

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (rtc->address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x00, true); // Start at register 0
    i2c_master_write(cmd, data, 7, true);
    i2c_master_stop(cmd);

    esp_err_t ret;
    int retries = 3;
    do {
        ret = i2c_master_cmd_begin(rtc->i2c_num, cmd, 500 / portTICK_PERIOD_MS);
        if (ret == ESP_OK) break;
        vTaskDelay(pdMS_TO_TICKS(10)); // Small delay between retries
    } while (--retries > 0);

    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set time after multiple retries: %s", esp_err_to_name(ret));
    }
    return (ret == ESP_OK) ? 0 : -1;
}

int ds3231_set_time_t(ds3231_t *rtc, time_t timestamp) {
    struct tm timeinfo;
    localtime_r(&timestamp, &timeinfo);
    return ds3231_set_time(rtc, &timeinfo);
}