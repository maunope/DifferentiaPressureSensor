#pragma once
#include "driver/i2c_master.h"
#include <time.h>
#include "esp_err.h"

#define DS3231_I2C_ADDR_DEFAULT 0x68

typedef struct {
    i2c_master_dev_handle_t i2c_dev_handle;
} ds3231_t;

/**
 * @brief DS3231 Control Register (0x0E) flags.
 */
typedef struct {
    uint8_t a1ie : 1;  /*!< Alarm 1 Interrupt Enable */
    uint8_t a2ie : 1;  /*!< Alarm 2 Interrupt Enable */
    uint8_t intcn : 1; /*!< Interrupt Control */
    uint8_t rs1 : 1;   /*!< Rate Select 1 */
    uint8_t rs2 : 1;   /*!< Rate Select 2 */
    uint8_t conv : 1;  /*!< Convert Temperature */
    uint8_t bbsqw : 1; /*!< Battery-Backed Square-Wave Enable */
    uint8_t eosc : 1;  /*!< Enable Oscillator */
} ds3231_control_reg_t;

/**
 * @brief Sets the DS3231 to a default configuration for low power and resilience.
 * @param rtc Pointer to the initialized DS3231 device descriptor.
 * @return esp_err_t `ESP_OK` on success, or an I2C communication error code on failure.
 */
esp_err_t ds3231_set_default_config(ds3231_t *rtc);

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

esp_err_t ds3231_dump_registers(ds3231_t *rtc);