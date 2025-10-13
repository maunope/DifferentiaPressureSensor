#pragma once
#include "driver/i2c.h"
#include <time.h>
#include "esp_err.h"

typedef struct {
    i2c_port_t i2c_num;
    uint8_t address;
} ds3231_t;

esp_err_t ds3231_init(ds3231_t *rtc, i2c_port_t i2c_num, uint8_t address);

esp_err_t ds3231_get_time(ds3231_t *rtc, struct tm *timeinfo);

esp_err_t ds3231_set_time(ds3231_t *rtc, const struct tm *timeinfo);

esp_err_t ds3231_set_time_t(ds3231_t *rtc, time_t timestamp);

esp_err_t ds3231_set_time_to_build_time(ds3231_t *rtc);