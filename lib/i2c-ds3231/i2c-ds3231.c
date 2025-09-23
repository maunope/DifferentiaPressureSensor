#include "i2c-ds3231.h"
#include <string.h>

#define DS3231_ADDR 0x68

static uint8_t bcd2dec(uint8_t val) { return ((val >> 4) * 10 + (val & 0x0F)); }
static uint8_t dec2bcd(uint8_t val) { return ((val / 10) << 4) | (val % 10); }

void ds3231_init(ds3231_t *rtc, i2c_port_t i2c_num, uint8_t address) {
    rtc->i2c_num = i2c_num;
    rtc->address = address;
}

int ds3231_get_time(ds3231_t *rtc, struct tm *timeinfo) {
    uint8_t data[7];
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (rtc->address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x00, true); // Start at register 0
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (rtc->address << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 7, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(rtc->i2c_num, cmd, 100 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) return -1;

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
    esp_err_t ret = i2c_master_cmd_begin(rtc->i2c_num, cmd, 100 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return (ret == ESP_OK) ? 0 : -1;
}

// CEST DST compensation: +1 hour from last Sunday in March to last Sunday in October
time_t ds3231_compensate_dst(time_t timestamp) {
    struct tm t;
    localtime_r(&timestamp, &t);
    int year = t.tm_year + 1900;

    // Find last Sunday in March
    struct tm last_march = { .tm_year = year - 1900, .tm_mon = 2, .tm_mday = 31, .tm_hour = 2, .tm_min = 0, .tm_sec = 0 };
    mktime(&last_march);
    last_march.tm_mday -= last_march.tm_wday;
    time_t dst_start = mktime(&last_march);

    // Find last Sunday in October
    struct tm last_oct = { .tm_year = year - 1900, .tm_mon = 9, .tm_mday = 31, .tm_hour = 3, .tm_min = 0, .tm_sec = 0 };
    mktime(&last_oct);
    last_oct.tm_mday -= last_oct.tm_wday;
    time_t dst_end = mktime(&last_oct);

    if (timestamp >= dst_start && timestamp < dst_end) {
        return timestamp + 3600; // Add 1 hour
    }
    return timestamp;
}