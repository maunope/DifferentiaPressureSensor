#include "i2c-d6fph.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

static const char *TAG = "D6F-PH";

#define I2C_MASTER_TIMEOUT_MS 1000

// D6F-PH Register Addresses and Commands
#define D6FPH_CMD_INIT_1 0x0B
#define D6FPH_CMD_INIT_2 0x00
#define D6FPH_CMD_COMMON 0xD0
#define D6FPH_CMD_READ_EEPROM 0x07

/**
 * @brief Writes a command with a 16-bit register address to the D6F-PH sensor.
 */
static esp_err_t d6fph_write_reg16(d6fph_t *dev, uint16_t reg, const uint8_t *data, size_t len)
{
    // The sensor protocol uses a 16-bit register address.
    // We send this as the first two bytes of the write buffer.
    uint8_t write_buf[len + 2];
    write_buf[0] = (reg >> 8) & 0xFF;
    write_buf[1] = reg & 0xFF;
    if (len > 0) {
        memcpy(&write_buf[2], data, len);
    }

    esp_err_t ret = i2c_master_write_to_device(dev->i2c_port, dev->i2c_addr, write_buf, sizeof(write_buf), pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C write failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

static esp_err_t d6fph_trigger_and_read(d6fph_t *dev, uint16_t *raw_value)
{
    esp_err_t ret;
    
    // Step 1: Trigger measurement
    const uint8_t trigger_data[] = {0x40, 0x18, 0x06};
    ret = d6fph_write_reg16(dev, 0x00D0, trigger_data, sizeof(trigger_data));
    if (ret != ESP_OK) return ret;

    // Step 2: Wait for measurement to complete
    vTaskDelay(pdMS_TO_TICKS(50));

    // Step 3: Prepare to read result
    const uint8_t read_prep_data[] = {0x51, 0x2C};
    ret = d6fph_write_reg16(dev, 0x00D0, read_prep_data, sizeof(read_prep_data));
    if (ret != ESP_OK) return ret;

    // Step 4: Read the 2 data bytes
    uint8_t read_reg = D6FPH_CMD_READ_EEPROM;
    uint8_t read_buffer[2]; // We only need the 2 data bytes
    ret = i2c_master_write_read_device(dev->i2c_port, dev->i2c_addr, &read_reg, 1, read_buffer, sizeof(read_buffer), pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    if (ret != ESP_OK)
    { 
        ESP_LOGE(TAG, "I2C read failed: %s", esp_err_to_name(ret));
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
    ESP_LOGI(TAG, "Initializing D6F-PH sensor at address 0x%02X", i2c_addr);

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

    // Send initialization command: Write 0x0B, 0x00
    ESP_LOGI(TAG, "Sending initialization command...");
    // This is a write to register 0x0B00 with no data payload
    esp_err_t ret = d6fph_write_reg16(dev, 0x0B00, NULL, 0);
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
        ESP_LOGE(TAG, "Device not initialized, cannot read pressure.");
        *pressure = NAN;
        return ESP_ERR_INVALID_STATE;
    }

    uint16_t raw_value;
    esp_err_t ret = d6fph_trigger_and_read(dev, &raw_value);

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to trigger and read from sensor: %s", esp_err_to_name(ret));
        *pressure = NAN;
        return ret;
    }

    ESP_LOGD(TAG, "Calculating pressure with: raw=%u, range=%.1f, mult=%.1f, sub=%.1f", raw_value, dev->range, dev->multiplier, dev->subtractor);
    // Formula from datasheet/Arduino library:
    // For +/- 500Pa (D6F-PH-5050AD4): P = ((Output - 1024) * 1000 / 60000) - 500
    // For +/- 250Pa (D6F-PH-0025AD1): P = ((Output - 1024) * 250 / 60000)
    *pressure = (((float)raw_value - 1024.0f) * dev->range / 60000.0f) * dev->multiplier - dev->subtractor;
    ESP_LOGD(TAG, "Calculated pressure: %.2f Pa", *pressure);

    return ESP_OK;
}