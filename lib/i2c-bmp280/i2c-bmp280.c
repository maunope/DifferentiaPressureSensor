#include "i2c-bmp280.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_TIMEOUT_MS 1000

#define BMP280_SENSOR_ADDR 0x76
#define BMP280_CHIP_ID_REG 0xD0
#define BMP280_CTRL_MEAS_REG 0xF4
#define BMP280_CONFIG_REG 0xF5
#define BMP280_CALIB_DATA_START 0x88

#define BMP280_I2C_ADDRESS 0X60

static const char *TAG = "BMP280";
static bmp280_calib_data_t calib_data;
static int32_t t_fine;

static esp_err_t bmp280_i2c_write_byte(i2c_port_t i2c_num, uint8_t reg_addr, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMP280_SENSOR_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "BMP280 I2C write byte failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

static esp_err_t bmp280_i2c_read_bytes(i2c_port_t i2c_num, uint8_t reg_addr, uint8_t *data, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMP280_SENSOR_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMP280_SENSOR_ADDR << 1) | I2C_MASTER_READ, true);
    if (len > 1)
    {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "BMP280 I2C read bytes failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t bmp280_read_calibration_data(void)
{
    uint8_t calib_data_raw[24];
    esp_err_t ret = bmp280_i2c_read_bytes(I2C_MASTER_NUM, BMP280_CALIB_DATA_START, calib_data_raw, 24);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read calibration data.");
        return ret;
    }

    calib_data.dig_T1 = (uint16_t)((calib_data_raw[1] << 8) | calib_data_raw[0]);
    calib_data.dig_T2 = (int16_t)((calib_data_raw[3] << 8) | calib_data_raw[2]);
    calib_data.dig_T3 = (int16_t)((calib_data_raw[5] << 8) | calib_data_raw[4]);
    calib_data.dig_P1 = (uint16_t)((calib_data_raw[7] << 8) | calib_data_raw[6]);
    calib_data.dig_P2 = (int16_t)((calib_data_raw[9] << 8) | calib_data_raw[8]);
    calib_data.dig_P3 = (int16_t)((calib_data_raw[11] << 8) | calib_data_raw[10]);
    calib_data.dig_P4 = (int16_t)((calib_data_raw[13] << 8) | calib_data_raw[12]);
    calib_data.dig_P5 = (int16_t)((calib_data_raw[15] << 8) | calib_data_raw[14]);
    calib_data.dig_P6 = (int16_t)((calib_data_raw[17] << 8) | calib_data_raw[16]);
    calib_data.dig_P7 = (int16_t)((calib_data_raw[19] << 8) | calib_data_raw[18]);
    calib_data.dig_P8 = (int16_t)((calib_data_raw[21] << 8) | calib_data_raw[20]);
    calib_data.dig_P9 = (int16_t)((calib_data_raw[23] << 8) | calib_data_raw[22]);

    ESP_LOGI(TAG, "Calibration Data Read:");
    ESP_LOGI(TAG, "dig_T1: %u, dig_T2: %d, dig_T3: %d", calib_data.dig_T1, calib_data.dig_T2, calib_data.dig_T3);
    ESP_LOGI(TAG, "dig_P1: %u, dig_P2: %d, dig_P3: %d, dig_P4: %d", calib_data.dig_P1, calib_data.dig_P2, calib_data.dig_P3, calib_data.dig_P4);
    ESP_LOGI(TAG, "dig_P5: %d, dig_P6: %d, dig_P7: %d, dig_P8: %d, dig_P9: %d", calib_data.dig_P5, calib_data.dig_P6, calib_data.dig_P7, calib_data.dig_P8, calib_data.dig_P9);

    return ESP_OK;
}

esp_err_t bmp280_init(void)
{
    esp_err_t ret;
    uint8_t chip_id;

    ret = bmp280_i2c_read_bytes(I2C_MASTER_NUM, BMP280_CHIP_ID_REG, &chip_id, 1);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read chip ID: %s", esp_err_to_name(ret));
        return ret;
    }

    if (chip_id != BMP280_I2C_ADDRESS)
    {
        ESP_LOGE(TAG, "BMP280 not found, chip ID: 0x%02X (Expected %02x)", chip_id,BMP280_I2C_ADDRESS);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "BMP280 found, chip ID: 0x%02X", chip_id);

    ret = bmp280_read_calibration_data();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read BMP280 calibration data.");
        return ret;
    }

    // Set control register to normal mode with 16x oversampling for both temp and pressure
    ret = bmp280_i2c_write_byte(I2C_MASTER_NUM, BMP280_CTRL_MEAS_REG, 0x5F); 
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write to CTRL_MEAS register.");
        return ret;
    }
    
    // Set config register for 0.5ms standby time and filter coefficient 16
    ret = bmp280_i2c_write_byte(I2C_MASTER_NUM, BMP280_CONFIG_REG, 0x10);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write to CONFIG register.");
        return ret;
    }

    return ESP_OK;
}

int32_t bmp280_read_raw_temp(void)
{
    uint8_t raw_data[3];
    bmp280_i2c_read_bytes(I2C_MASTER_NUM, 0xFA, raw_data, 3);
    return (int32_t)((raw_data[0] << 12) | (raw_data[1] << 4) | (raw_data[2] >> 4));
}

int32_t bmp280_read_raw_pressure(void)
{
    uint8_t raw_data[3];
    bmp280_i2c_read_bytes(I2C_MASTER_NUM, 0xF7, raw_data, 3);
    return (int32_t)((raw_data[0] << 12) | (raw_data[1] << 4) | (raw_data[2] >> 4));
}

float bmp280_compensate_temperature(int32_t adc_T)
{
    int32_t var1, var2;
    var1 = ((((adc_T >> 3) - ((int32_t)calib_data.dig_T1 << 1))) * ((int32_t)calib_data.dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)calib_data.dig_T1)) * ((adc_T >> 4) - ((int32_t)calib_data.dig_T1))) >> 12) * ((int32_t)calib_data.dig_T3)) >> 14;
    t_fine = var1 + var2;
    float T = (t_fine * 5 + 128) >> 8;
    return T / 100.0;
}

long bmp280_compensate_pressure(int32_t adc_P)
{
    int32_t var1, var2;
    uint32_t p;

    var1 = (((int32_t)t_fine) >> 1) - (int32_t)64000;
    var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t)calib_data.dig_P6);
    var2 = var2 + ((var1 * ((int32_t)calib_data.dig_P5)) << 1);
    var2 = (var2 >> 2) + (((int32_t)calib_data.dig_P4) << 16);
    var1 = (((calib_data.dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((int32_t)calib_data.dig_P2) * var1) >> 1)) >> 18;
    var1 = (((32768 + var1)) * ((int32_t)calib_data.dig_P1)) >> 15;
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
    var1 = (((int32_t)calib_data.dig_P9) * ((int32_t)(((p >> 3) * (p >> 3)) >> 13))) >> 12;
    var2 = (((int32_t)(p >> 2)) * ((int32_t)calib_data.dig_P8)) >> 13;
    p = (uint32_t)((int32_t)p + ((var1 + var2 + calib_data.dig_P7) >> 4));

    return (long)p;
}
