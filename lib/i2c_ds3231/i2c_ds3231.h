#pragma once
#include "driver/i2c_master.h"
#include <time.h>
#include "esp_err.h"

#define DS3231_I2C_ADDR_DEFAULT 0x68

typedef struct {
    i2c_master_dev_handle_t i2c_dev_handle;
} ds3231_t;

esp_err_t ds3231_init(ds3231_t *rtc, i2c_master_bus_handle_t bus_handle, uint8_t address);

/**
 * @brief Initializes the DS3231 RTC with the default I2C address (0x68).
 * This is a convenience macro that calls ds3231_init with the default address.
 */
#define ds3231_init_default(rtc, i2c_num) ds3231_init(rtc, i2c_num, DS3231_I2C_ADDR_DEFAULT)

esp_err_t ds3231_get_time(ds3231_t *rtc, struct tm *timeinfo);

esp_err_t ds3231_set_time(ds3231_t *rtc, const struct tm *timeinfo);

esp_err_t ds3231_set_time_t(ds3231_t *rtc, time_t timestamp);

esp_err_t ds3231_set_time_to_build_time(ds3231_t *rtc);