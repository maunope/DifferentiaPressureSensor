#pragma once
#include "driver/i2c.h"
#include <time.h>

typedef struct {
    i2c_port_t i2c_num;
    uint8_t address;
} ds3231_t;

// Create DS3231 instance (reuse I2C master)
void ds3231_init(ds3231_t *rtc, i2c_port_t i2c_num, uint8_t address);

// Read current time from DS3231
int ds3231_get_time(ds3231_t *rtc, struct tm *timeinfo);

// Set DS3231 time
int ds3231_set_time(ds3231_t *rtc, const struct tm *timeinfo);

// Compensate timestamp for DST (CEST rules)
time_t ds3231_compensate_dst(time_t timestamp);