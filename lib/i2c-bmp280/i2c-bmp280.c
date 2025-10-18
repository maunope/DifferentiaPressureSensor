#include "i2c-bmp280.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_MASTER_TIMEOUT_MS 1000

#define BMP280_CHIP_ID_REG 0xD0
#define BMP280_CTRL_MEAS_REG 0xF4
#define BMP280_CONFIG_REG 0xF5
#define BMP280_CALIB_DATA_START 0x88
#define BMP280_REG_STATUS 0xF3

// Status register bits
#define BMP280_STATUS_MEASURING (1 << 3)
#define BMP280_STATUS_IM_UPDATE (1 << 0)
#define BMP280_MODE_FORCED 0x01

#define BMP280_I2C_ADDRESS 0X60
static const char *TAG = "BMP280";
static int32_t t_fine;

/**
 * @brief Writes a single byte to a register on the BMP280.
 *
 * @param dev Pointer to the BMP280 device descriptor.
 * @param reg_addr The register address to write to.
 * @param data The byte of data to write.
 * @return esp_err_t `ESP_OK` on success, or an error code on failure.
 */
static esp_err_t bmp280_i2c_write_byte(const bmp280_t *dev, uint8_t reg_addr, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev->i2c_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(dev->i2c_port, cmd, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd); 
    return ret;
}

/**
 * @brief Reads a sequence of bytes from a starting register on the BMP280.
 *
 * @param dev Pointer to the BMP280 device descriptor.
 * @param reg_addr The starting register address to read from.
 * @param data Pointer to the buffer to store the read data.
 * @param len The number of bytes to read.
 * @return esp_err_t `ESP_OK` on success, or an error code on failure.
 */
static esp_err_t bmp280_i2c_read_bytes(const bmp280_t *dev, uint8_t reg_addr, uint8_t *data, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev->i2c_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev->i2c_addr << 1) | I2C_MASTER_READ, true);
    if (len > 1)
    {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(dev->i2c_port, cmd, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd); 
    return ret;
}

/**
 * @brief Reads the factory-programmed calibration data from the BMP280.
 *
 * @param dev Pointer to the BMP280 device descriptor.
 * @return esp_err_t `ESP_OK` on success, or an error code on failure.
 */
static esp_err_t bmp280_read_calibration_data(bmp280_t *dev)
{
    uint8_t calib_data_raw[24];
    esp_err_t ret = bmp280_i2c_read_bytes(dev, BMP280_CALIB_DATA_START, calib_data_raw, 24);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read calibration data.");
        return ret;
    }

    dev->calib_data.dig_T1 = (uint16_t)((calib_data_raw[1] << 8) | calib_data_raw[0]);
    dev->calib_data.dig_T2 = (int16_t)((calib_data_raw[3] << 8) | calib_data_raw[2]);
    dev->calib_data.dig_T3 = (int16_t)((calib_data_raw[5] << 8) | calib_data_raw[4]);
    dev->calib_data.dig_P1 = (uint16_t)((calib_data_raw[7] << 8) | calib_data_raw[6]);
    dev->calib_data.dig_P2 = (int16_t)((calib_data_raw[9] << 8) | calib_data_raw[8]);
    dev->calib_data.dig_P3 = (int16_t)((calib_data_raw[11] << 8) | calib_data_raw[10]);
    dev->calib_data.dig_P4 = (int16_t)((calib_data_raw[13] << 8) | calib_data_raw[12]);
    dev->calib_data.dig_P5 = (int16_t)((calib_data_raw[15] << 8) | calib_data_raw[14]);
    dev->calib_data.dig_P6 = (int16_t)((calib_data_raw[17] << 8) | calib_data_raw[16]);
    dev->calib_data.dig_P7 = (int16_t)((calib_data_raw[19] << 8) | calib_data_raw[18]);
    dev->calib_data.dig_P8 = (int16_t)((calib_data_raw[21] << 8) | calib_data_raw[20]);
    dev->calib_data.dig_P9 = (int16_t)((calib_data_raw[23] << 8) | calib_data_raw[22]);

    ESP_LOGI(TAG, "Calibration Data Read:");
    ESP_LOGI(TAG, "dig_T1: %u, dig_T2: %d, dig_T3: %d", dev->calib_data.dig_T1, dev->calib_data.dig_T2, dev->calib_data.dig_T3);
    ESP_LOGI(TAG, "dig_P1: %u, dig_P2: %d, dig_P3: %d, dig_P4: %d", dev->calib_data.dig_P1, dev->calib_data.dig_P2, dev->calib_data.dig_P3, dev->calib_data.dig_P4);
    ESP_LOGI(TAG, "dig_P5: %d, dig_P6: %d, dig_P7: %d, dig_P8: %d, dig_P9: %d", dev->calib_data.dig_P5, dev->calib_data.dig_P6, dev->calib_data.dig_P7, dev->calib_data.dig_P8, dev->calib_data.dig_P9);

    return ESP_OK;
}

/**
 * @brief Initializes the BMP280 sensor.
 *
 * This function checks the chip ID, reads calibration data, and configures the
 * sensor for normal operation with default oversampling and filter settings.
 * @param dev Pointer to the BMP280 device descriptor to initialize.
 * @param port The I2C port number to use.
 * @param i2c_addr The I2C address of the sensor.
 * @return esp_err_t `ESP_OK` on success, or an error code on failure.
 */
esp_err_t bmp280_init(bmp280_t *dev, i2c_port_t port, uint8_t i2c_addr)
{
    dev->i2c_port = port;
    dev->i2c_addr = i2c_addr;

    esp_err_t ret;
    uint8_t chip_id;

    ret = bmp280_i2c_read_bytes(dev, BMP280_CHIP_ID_REG, &chip_id, 1);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read chip ID: %s", esp_err_to_name(ret));
        return ret;
    }

    if (chip_id != 0x58 && chip_id != 0x60) // Accept BMP280 (0x58) or BME280 (0x60)
    {
        ESP_LOGE(TAG, "BMP/BME280 not found, chip ID: 0x%02X (Expected 0x58 or 0x60)", chip_id);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "BMP/BME280 found, chip ID: 0x%02X", chip_id);

    ret = bmp280_read_calibration_data(dev);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read BMP280 calibration data.");
        return ret;
    }

    // Set control register to normal mode with 16x oversampling for both temp and pressure
    ret = bmp280_i2c_write_byte(dev, BMP280_CTRL_MEAS_REG, 0x5F);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write to CTRL_MEAS register.");
        return ret;
    }
    
    // Set config register for 0.5ms standby time and filter coefficient 16
    ret = bmp280_i2c_write_byte(dev, BMP280_CONFIG_REG, 0x10);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write to CONFIG register.");
        return ret;
    }

    return ESP_OK;
}

/**
 * @brief Reads the raw, uncompensated temperature value from the sensor.
 *
 * @param dev Pointer to the BMP280 device descriptor.
 * @return int32_t The raw ADC value for temperature. Returns 0 on read failure.
 */
int32_t bmp280_read_raw_temp(bmp280_t *dev)
{
    uint8_t raw_data[3];
    if (bmp280_i2c_read_bytes(dev, 0xFA, raw_data, 3) != ESP_OK) {
        return 0; // Return 0 on read failure
    }
    return (int32_t)((raw_data[0] << 12) | (raw_data[1] << 4) | (raw_data[2] >> 4));
}

/**
 * @brief Reads the raw, uncompensated pressure value from the sensor.
 *
 * @param dev Pointer to the BMP280 device descriptor.
 * @return int32_t The raw ADC value for pressure. Returns 0 on read failure.
 */
int32_t bmp280_read_raw_pressure(bmp280_t *dev)
{
    uint8_t raw_data[3];
    if (bmp280_i2c_read_bytes(dev, 0xF7, raw_data, 3) != ESP_OK) {
        return 0; // Return 0 on read failure
    }
    return (int32_t)((raw_data[0] << 12) | (raw_data[1] << 4) | (raw_data[2] >> 4));
}

/**
 * @brief Compensates the raw temperature ADC value to get a temperature in degrees Celsius.
 *
 * This function uses the calibration data read during initialization. It also
 * calculates the `t_fine` value required for pressure compensation.
 * @param dev Pointer to the BMP280 device descriptor.
 * @param adc_T The raw temperature ADC value from `bmp280_read_raw_temp`.
 * @return float The compensated temperature in degrees Celsius.
 */
float bmp280_compensate_temperature(bmp280_t *dev, int32_t adc_T)
{
    int32_t var1, var2;
    var1 = ((((adc_T >> 3) - ((int32_t)dev->calib_data.dig_T1 << 1))) * ((int32_t)dev->calib_data.dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)dev->calib_data.dig_T1)) * ((adc_T >> 4) - ((int32_t)dev->calib_data.dig_T1))) >> 12) * ((int32_t)dev->calib_data.dig_T3)) >> 14;
    t_fine = var1 + var2;
    float T = (t_fine * 5 + 128) >> 8;
    return T / 100.0;
}

/**
 * @brief Compensates the raw pressure ADC value to get a pressure in Pascals.
 *
 * This function uses the calibration data and the `t_fine` value calculated
 * by `bmp280_compensate_temperature`.
 * @param dev Pointer to the BMP280 device descriptor.
 * @param adc_P The raw pressure ADC value from `bmp280_read_raw_pressure`.
 * @return long The compensated pressure in Pascals (Pa).
 */
long bmp280_compensate_pressure(bmp280_t *dev, int32_t adc_P)
{
    int32_t var1, var2;
    uint32_t p;

    var1 = (((int32_t)t_fine) >> 1) - (int32_t)64000;
    var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t)dev->calib_data.dig_P6);
    var2 = var2 + ((var1 * ((int32_t)dev->calib_data.dig_P5)) << 1);
    var2 = (var2 >> 2) + (((int32_t)dev->calib_data.dig_P4) << 16);
    var1 = (((dev->calib_data.dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((int32_t)dev->calib_data.dig_P2) * var1) >> 1)) >> 18;
    var1 = (((32768 + var1)) * ((int32_t)dev->calib_data.dig_P1)) >> 15;
    if (var1 == 0)
    {
        return 0; // Avoid division by zero
    }
    p = (((uint32_t)(((int32_t)1048576) - adc_P) - (var2 >> 12))) * 3125;
    if (p < 0x80000000)
    {
        p = (p << 1) / ((uint32_t)var1);
    }
    else
    {
        p = (p / (uint32_t)var1) * 2;
    }
    var1 = (((int32_t)dev->calib_data.dig_P9) * ((int32_t)(((p >> 3) * (p >> 3)) >> 13))) >> 12;
    var2 = (((int32_t)(p >> 2)) * ((int32_t)dev->calib_data.dig_P8)) >> 13;
    p = (uint32_t)((int32_t)p + ((var1 + var2 + dev->calib_data.dig_P7) >> 4));

    return (long)p;
}

esp_err_t bmp280_force_read(bmp280_t *dev, float *temperature, long *pressure)
{
    if (!dev) return ESP_ERR_INVALID_ARG;

    const int MAX_RETRIES = 3;
    const int RETRY_DELAY_MS = 50;

    // 1. Read the current ctrl_meas register
    uint8_t ctrl_meas;
    esp_err_t err;
    for (int i = 0; i < MAX_RETRIES; i++) {
        err = bmp280_i2c_read_bytes(dev, BMP280_CTRL_MEAS_REG, &ctrl_meas, 1);
        if (err == ESP_OK) break;
        vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
    }
    if (err != ESP_OK) {
        // This log is commented out as per your request to be less verbose.
        // The calling function in datalogger_task will handle the final error logging.
        // ESP_LOGE(TAG, "Failed to read CTRL_MEAS for forced read.");
        return err;
    }

    // 2. Set the sensor to "Forced mode"
    // This retains the oversampling settings but triggers a single measurement.
    ctrl_meas = (ctrl_meas & 0xFC) | BMP280_MODE_FORCED;
    err = bmp280_i2c_write_byte(dev, BMP280_CTRL_MEAS_REG, ctrl_meas);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write CTRL_MEAS for forced read.");
        return err;
    }

    // 3. Poll the status register until the measurement is complete.
    // The 'measuring' bit (bit 3) will be 0 when done.
    uint8_t status;
    do {
        vTaskDelay(pdMS_TO_TICKS(5)); // Small delay to avoid busy-waiting too fast
        err = bmp280_i2c_read_bytes(dev, BMP280_REG_STATUS, &status, 1);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read STATUS during poll.");
            return err;
        }
    } while (status & BMP280_STATUS_MEASURING);

    // 4. Now that the measurement is complete, read the fresh data.
    int32_t raw_temp = bmp280_read_raw_temp(dev);
    int32_t raw_press = bmp280_read_raw_pressure(dev);

    if (raw_temp == 0 || raw_press == 0) { // Check for read failure
        return ESP_FAIL;
    }

    *temperature = bmp280_compensate_temperature(dev, raw_temp);
    *pressure = bmp280_compensate_pressure(dev, raw_press);

    return ESP_OK;
}
