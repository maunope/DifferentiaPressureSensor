#include "i2c_ds3231.h"
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <time.h>
#include <sys/time.h>

#define DS3231_ADDR 0x68
static const char *TAG = "ds3231";

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
esp_err_t ds3231_init(ds3231_t *rtc, i2c_master_bus_handle_t bus_handle, uint8_t address)
{
    if (!rtc)
    {
        return ESP_ERR_INVALID_ARG;
    }
    if (address == 0)
    {
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
esp_err_t ds3231_get_time(ds3231_t *rtc, struct tm *timeinfo)
{
    uint8_t data[7];
    uint8_t reg_addr = 0x00;

    esp_err_t ret;
    // F*ck those cheap *ss modules from Aliexpress, sometimes it NAKs the first read attempt
    int retries = 3;
    do
    {
        ret = i2c_master_transmit_receive(rtc->i2c_dev_handle, &reg_addr, 1, data, 7, pdMS_TO_TICKS(500));
        if (ret == ESP_OK)
            break;
        vTaskDelay(pdMS_TO_TICKS(10)); // Small delay between retries
    } while (--retries > 0);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to get time after multiple retries: %s", esp_err_to_name(ret));
        return ret;
    }

    timeinfo->tm_sec = bcd2dec(data[0]);
    timeinfo->tm_min = bcd2dec(data[1]);
    timeinfo->tm_hour = bcd2dec(data[2]);
    timeinfo->tm_mday = bcd2dec(data[4]);
    timeinfo->tm_mon = bcd2dec(data[5]) - 1;
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
esp_err_t ds3231_set_time(ds3231_t *rtc, const struct tm *timeinfo)
{
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
    do
    {
        ret = i2c_master_transmit(rtc->i2c_dev_handle, write_buf, sizeof(write_buf), pdMS_TO_TICKS(500));
        if (ret == ESP_OK)
            break;
        vTaskDelay(pdMS_TO_TICKS(10)); // Small delay between retries
    } while (--retries > 0);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set time after multiple retries: %s", esp_err_to_name(ret));
    }
    return ret;
}

/**
 * @brief Sets the DS3231 to a default configuration for low power and resilience.
 *
 * This function configures the DS3231 control register (0x0E) with a set of
 * recommended defaults and clears critical status flags in the status register (0x0F).
* We are setting the device for LOWEST POWER and HIGH RESILIENCE (Run on battery, disable SQW on battery, no alarms).
 ------------------------------------------------------------------------------------------------------------------
* CONTROL REGISTER (0x0E) - Target 0x04 (Binary 0000 0100):
*- EOSC (B7):  0 (RUN on VBAT)  <- HIGH RESILIENCE
*- BBSQW (B6): 0 (Disabled on VBAT) <- LOW POWER
*- CONV (B5):  0 (Auto Temp)
*- RS2/RS1 (B4/B3): 0/0 (ignored in SQW mode)
*- INTCN (B2): 0 (Square-Wave mode)
*- A2IE/A1IE (B1/B0): 0/0 (Alarms Disabled)
*
* STATUS REGISTER (0x0F) - Mask to Clear 0x86 (Binary 1000 0110):
*- OSF (B7):   Cleared (0) <- HIGH RESILIENCE (Time is valid)
*- A2F (B2):   Cleared (0)
*- A1F (B1):   Cleared (0)
*- EN32KHZ (B6):   Preserved/Set (1)
* ------------------------------------------------------------------------------------------------------------------
 *
 * @param rtc Pointer to the initialized DS3231 device descriptor.
 * @return esp_err_t `ESP_OK` on success, or an I2C communication error code on failure.
 */
esp_err_t ds3231_set_default_config(ds3231_t *rtc)
{
    esp_err_t ret;

    // 1. Set the Control Register (0x0E) to the lowest-power, no-alarm value (0x04).
    // (EOSC=0, BBSQW=0, CONV=0, INTCN=0, A2IE=0, A1IE=0)
    uint8_t control_reg_val = 0x04;
    uint8_t write_buf_ctrl[2] = {0x0E, control_reg_val};
    ret = i2c_master_transmit(rtc->i2c_dev_handle, write_buf_ctrl, sizeof(write_buf_ctrl), pdMS_TO_TICKS(500));
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set default control register config: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "DS3231 control register set to optimal low-power (0x%02X)", control_reg_val);

    // 2. Clear the Oscillator Stop Flag (OSF), Alarm Flags (A1F, A2F) in the Status Register (0x0F).
    uint8_t status_reg;
    uint8_t reg_addr = 0x0F;
    ret = i2c_master_transmit_receive(rtc->i2c_dev_handle, &reg_addr, 1, &status_reg, 1, pdMS_TO_TICKS(500));
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read status register for clearing flags: %s", esp_err_to_name(ret));
        return ret;
    }

    // Mask is ~0x86 (Clear bits 7, 2, 1 -> OSF, A2F, A1F)
    uint8_t clear_mask = 0x86;
    uint8_t new_status_val = (uint8_t)(status_reg & ~clear_mask);

    // Ensure EN32KHZ (Bit 6) remains set, as it is a common requirement.
    // If you do not need 32kHz output, remove this line.
    new_status_val |= 0x40;

    uint8_t write_buf_stat[2] = {0x0F, new_status_val};
    return i2c_master_transmit(rtc->i2c_dev_handle, write_buf_stat, sizeof(write_buf_stat), pdMS_TO_TICKS(500));
}

/**
 * @brief Sets the time on the DS3231 RTC from a `time_t` timestamp.
 *
 * Converts a `time_t` (seconds since epoch) into a `struct tm` and sets the RTC.
 * @param rtc Pointer to the initialized DS3231 device descriptor.
 * @param timestamp The `time_t` timestamp (in UTC) to set.
 * @return esp_err_t `ESP_OK` on success, or an error code on failure.
 */
esp_err_t ds3231_set_time_t(ds3231_t *rtc, time_t timestamp)
{
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
    if (!rtc)
    {
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
    if (ds3231_set_time_t(rtc, build_ts) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set RTC time.");
        // Restore the system timezone to UTC before returning
        setenv("TZ", "UTC0", 1);
        tzset();
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "RTC time set to build time: %s %s", date_str, time_str);
    struct timeval tv = {.tv_sec = build_ts};
    settimeofday(&tv, NULL);
    return ESP_OK;
}

/**
 * @brief Dumps the DS3231 control and status registers to the log.
 *
 * Reads and decodes the flags in the control (0x0E) and status (0x0F) registers,
 * printing their state for debugging purposes.
 * @param rtc Pointer to the initialized DS3231 device descriptor.
 * @return esp_err_t `ESP_OK` on success, or an I2C communication error code on failure.
 */
esp_err_t ds3231_dump_registers(ds3231_t *rtc)
{
    uint8_t control_reg, status_reg;
    uint8_t reg_addr;
    esp_err_t ret;

    // Read Control Register (0x0E)
    reg_addr = 0x0E;
    ret = i2c_master_transmit_receive(rtc->i2c_dev_handle, &reg_addr, 1, &control_reg, 1, pdMS_TO_TICKS(500));
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read control register: %s", esp_err_to_name(ret));
        return ret;
    }

    // Read Status Register (0x0F)
    reg_addr = 0x0F;
    ret = i2c_master_transmit_receive(rtc->i2c_dev_handle, &reg_addr, 1, &status_reg, 1, pdMS_TO_TICKS(500));
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read status register: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "DS3231 Register Dump:");
    ESP_LOGI(TAG, "---------------------");
    ESP_LOGI(TAG, "Control Register (0x0E): 0x%02X", control_reg);
    ESP_LOGI(TAG, "  - EOSC (Enable Oscillator): %d", (control_reg >> 7) & 0x01);
    ESP_LOGI(TAG, "  - BBSQW (Battery-Backed Square-Wave Enable): %d", (control_reg >> 6) & 0x01);
    ESP_LOGI(TAG, "  - CONV (Convert Temperature): %d", (control_reg >> 5) & 0x01);
    ESP_LOGI(TAG, "  - RS2, RS1 (Rate Select): %d, %d", (control_reg >> 4) & 0x01, (control_reg >> 3) & 0x01);
    ESP_LOGI(TAG, "  - INTCN (Interrupt Control): %d", (control_reg >> 2) & 0x01);
    ESP_LOGI(TAG, "  - A2IE (Alarm 2 Interrupt Enable): %d", (control_reg >> 1) & 0x01);
    ESP_LOGI(TAG, "  - A1IE (Alarm 1 Interrupt Enable): %d", control_reg & 0x01);

    ESP_LOGI(TAG, "Status Register (0x0F): 0x%02X", status_reg);
    ESP_LOGI(TAG, "  - OSF (Oscillator Stop Flag): %d", (status_reg >> 7) & 0x01);
    ESP_LOGI(TAG, "  - EN32KHZ (Enable 32kHz Output): %d", (status_reg >> 3) & 0x01);
    ESP_LOGI(TAG, "  - BSY (Busy): %d", (status_reg >> 2) & 0x01);
    ESP_LOGI(TAG, "  - A2F (Alarm 2 Flag): %d", (status_reg >> 1) & 0x01);
    ESP_LOGI(TAG, "  - A1F (Alarm 1 Flag): %d", status_reg & 0x01);
    ESP_LOGI(TAG, "---------------------");

    return ESP_OK;
}