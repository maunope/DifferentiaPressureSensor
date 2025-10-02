#pragma once

#include "driver/gpio.h"
#include "esp_err.h"

/**
 * @brief Configuration for the battery reader.
 *
 * @param adc_gpio_num The GPIO pin connected to the battery voltage divider.
 * @param voltage_divider_ratio The ratio of the voltage divider (e.g., 2.0 for a divider with two equal resistors).
 */
esp_err_t battery_reader_init(gpio_num_t adc_gpio_num, float voltage_divider_ratio);

/**
 * @brief Reads the current battery voltage.
 *
 * @return The battery voltage in volts (V). Returns 0.0f on failure.
 */
float battery_reader_get_voltage(void);

/**
 * @brief Estimates the battery charge level as a percentage.
 *
 * This is based on a typical 3.7V LiPo discharge curve.
 *
 * @return The estimated battery charge percentage (0-100).
 */
int battery_reader_get_percentage(void);