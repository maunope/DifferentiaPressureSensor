#include "i2c-d6fph.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

static const char *TAG = "D6F-PH";

#define I2C_MASTER_TIMEOUT_MS 1000

// D6F-PH Register Addresses and Commands
#define D6FPH_CMD_INIT 0x0B
#define D6FPH_CMD_INIT_PARAM 0x00
#define D6FPH_CMD_TRIGGER_READ 0xD0
#define D6FPH_CMD_TRIGGER_PARAM_H 0x40
#define D6FPH_CMD_TRIGGER_PARAM_L 0x18
#define D6FPH_CMD_READ_RESULT 0xD0
#define D6FPH_CMD_READ_PARAM_H 0x51
#define D6FPH_CMD_READ_PARAM_L 0x2C

/**
 * @brief Writes a command sequence to the D6F-PH sensor.
 */
static esp_err_t d6fph_write_cmd(d6fph_t *dev, const uint8_t *cmd_buffer, size_t len)
{
    return i2c_master_write_to_device(dev->i2c_port, dev->i2c_addr, cmd_buffer, len, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
}

/**
 * @brief Reads data from the D6F-PH sensor.
 */
static esp_err_t d6fph_read_data(d6fph_t *dev, uint8_t *data_buffer, size_t len)
{
    return i2c_master_read_from_device(dev->i2c_port, dev->i2c_addr, data_buffer, len, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
}

/**
 * @brief Triggers a measurement and reads the result from the sensor.
 */
static esp_err_t d6fph_trigger_and_read(d6fph_t *dev, uint16_t *raw_value)
{
    esp_err_t ret;

    // Step 1: Send command to trigger measurement
    const uint8_t trigger_cmd[] = {D6FPH_CMD_TRIGGER_READ, D6FPH_CMD_TRIGGER_PARAM_H, D6FPH_CMD_TRIGGER_PARAM_L};
    ret = d6fph_write_cmd(dev, trigger_cmd, sizeof(trigger_cmd));
    if (ret != ESP_OK)
    { 
        return ret;
    }

    // Step 2: Wait for the measurement to complete (sensor datasheet indicates 33ms)
    vTaskDelay(pdMS_TO_TICKS(35));

    // Step 3: Send command to read the result
    const uint8_t read_cmd[] = {D6FPH_CMD_READ_RESULT, D6FPH_CMD_READ_PARAM_H, D6FPH_CMD_READ_PARAM_L};
    ret = d6fph_write_cmd(dev, read_cmd, sizeof(read_cmd));
    if (ret != ESP_OK)
    { 
        return ret;
    }

    // Step 4: Read the 2 data bytes and 1 CRC byte
    uint8_t read_buffer[3];
    ret = d6fph_read_data(dev, read_buffer, sizeof(read_buffer));
    if (ret != ESP_OK)
    { 
        return ret;
    }

    // TODO: Implement CRC8 check if needed for higher reliability.
    // The Arduino library does not seem to use it.

    *raw_value = (read_buffer[0] << 8) | read_buffer[1];

    return ESP_OK;
}

esp_err_t d6fph_init(d6fph_t *dev, i2c_port_t port, uint8_t i2c_addr, d6fph_sensor_model_t model)
{
    if (!dev)
    {
        return ESP_ERR_INVALID_ARG;
    }

    dev->i2c_port = port;
    dev->i2c_addr = i2c_addr;
    dev->model = model;
    dev->is_initialized = false; // Default to not initialized

    switch (model)
    {
    case D6FPH_MODEL_0025AD1:
        dev->range = 250.0f;
        dev->multiplier = 1.0f;
        dev->subtractor = 0.0f;
        break;
    case D6FPH_MODEL_0505AD3:
        dev->range = 50.0f;
        dev->multiplier = 2.0f;
        dev->subtractor = 50.0f;
        break;
    case D6FPH_MODEL_5050AD4:
    default:
        dev->range = 500.0f;
        dev->multiplier = 2.0f;
        dev->subtractor = 500.0f;
        break;
    }

    // Send initialization command: Write 0x00 to register 0x0B
    const uint8_t init_cmd[] = {D6FPH_CMD_INIT, D6FPH_CMD_INIT_PARAM};
    esp_err_t ret = d6fph_write_cmd(dev, init_cmd, sizeof(init_cmd));
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "D6F-PH initialization failed: %s", esp_err_to_name(ret));
        dev->is_initialized = false;
        return ret;
    }

    ESP_LOGI(TAG, "D6F-PH sensor initialized successfully at address 0x%02X", i2c_addr);
    dev->is_initialized = true;
    return ESP_OK;
}

esp_err_t d6fph_read_pressure(d6fph_t *dev, float *pressure)
{
    if (!dev || !dev->is_initialized) {
        *pressure = NAN;
        return ESP_ERR_INVALID_STATE;
    }

    uint16_t raw_value;
    esp_err_t ret = d6fph_trigger_and_read(dev, &raw_value);

    if (ret != ESP_OK)
    {
        *pressure = NAN;
        return ret;
    }

    // Formula from datasheet/Arduino library: P = ((Output - 1024) * Range * Multiplier / 60000) - Subtractor
    *pressure = ((float)raw_value - 1024.0f) * dev->range * dev->multiplier / 60000.0f - dev->subtractor;

    return ESP_OK;
}