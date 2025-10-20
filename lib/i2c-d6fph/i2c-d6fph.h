#pragma once

#include "driver/i2c.h"
#include "esp_err.h"
#include <stdint.h>

#define D6FPH_I2C_ADDR_DEFAULT 0x6C

/**
 * @brief Enum for the different D6F-PH sensor models.
 */
typedef enum
{
    D6FPH_MODEL_0025AD1, // 0 to 250 Pa
    D6FPH_MODEL_0505AD3, // -50 to +50 Pa
    D6FPH_MODEL_5050AD4, // -500 to +500 Pa
} d6fph_sensor_model_t;

/**
 * @brief Device descriptor for the D6F-PH sensor.
 */
typedef struct
{
    i2c_port_t i2c_port;
    uint8_t i2c_addr;
    d6fph_sensor_model_t model;
    bool is_initialized;

    // Model-specific calculation values
    float range;
    float multiplier;
    float subtractor;
} d6fph_t;

/**
 * @brief Initializes the D6F-PH sensor.
 *
 * @param dev Pointer to the D6F-PH device descriptor.
 * @param port The I2C port number.
 * @param i2c_addr The I2C address of the sensor.
 * @param model The specific model of the D6F-PH sensor.
 * @return esp_err_t `ESP_OK` on success, or an error code on failure.
 */
esp_err_t d6fph_init(d6fph_t *dev, i2c_port_t port, uint8_t i2c_addr, d6fph_sensor_model_t model);

/**
 * @brief Initializes the D6F-PH sensor with the default I2C address (0x6C).
 * This is a convenience macro that calls d6fph_init with the default address.
 */
#define d6fph_init_default(dev, port, model) d6fph_init(dev, port, D6FPH_I2C_ADDR_DEFAULT, model)

/**
 * @brief Reads the differential pressure from the sensor.
 * @param dev Pointer to the initialized D6F-PH device descriptor.
 * @param pressure Pointer to a float to store the pressure in Pascals (Pa).
 * @return esp_err_t `ESP_OK` on success, or an error code on failure.
 */
esp_err_t d6fph_read_pressure(d6fph_t *dev, float *pressure);