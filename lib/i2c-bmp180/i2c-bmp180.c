#include "i2c-bmp180.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_TIMEOUT_MS 1000

#define BMP180_SENSOR_ADDR 0x77
#define BMP180_CHIP_ID_REG 0xD0
#define BMP180_CTRL_MEAS_REG 0xF4
#define BMP180_TEMP_CAL_MSB_REG 0xF6
#define BMP180_PRESS_CAL_MSB_REG 0xF6

#define BMP180_READ_TEMP_CMD 0x2E
#define BMP180_READ_PRESSURE_0_CMD 0x34

#define BMP180_OSS 0

static const char *TAG = "BMP180";
static bmp180_calib_data_t calib_data;
static long B5;

static esp_err_t bmp180_i2c_write_byte(i2c_port_t i2c_num, uint8_t reg_addr, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMP180_SENSOR_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "BMP180 I2C write byte failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

static esp_err_t bmp180_i2c_read_bytes(i2c_port_t i2c_num, uint8_t reg_addr, uint8_t *data, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMP180_SENSOR_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMP180_SENSOR_ADDR << 1) | I2C_MASTER_READ, true);
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
        ESP_LOGE(TAG, "BMP180 I2C read bytes failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

static int16_t read_s16(uint8_t reg_addr)
{
    uint8_t raw_data[2];
    bmp180_i2c_read_bytes(I2C_MASTER_NUM, reg_addr, raw_data, 2);
    return (int16_t)((raw_data[0] << 8) | raw_data[1]);
}

static uint16_t read_u16(uint8_t reg_addr)
{
    uint8_t raw_data[2];
    bmp180_i2c_read_bytes(I2C_MASTER_NUM, reg_addr, raw_data, 2);
    return (uint16_t)((raw_data[0] << 8) | raw_data[1]);
}

esp_err_t bmp180_read_calibration_data(void)
{
    calib_data.AC1 = read_s16(0xAA);
    calib_data.AC2 = read_s16(0xAC);
    calib_data.AC3 = read_s16(0xAE);
    calib_data.AC4 = read_u16(0xB0);
    calib_data.AC5 = read_u16(0xB2);
    calib_data.AC6 = read_u16(0xB4);
    calib_data.B1 = read_s16(0xB6);
    calib_data.B2 = read_s16(0xB8);
    calib_data.MB = read_s16(0xBA);
    calib_data.MC = read_s16(0xBC);
    calib_data.MD = read_s16(0xBE);

    ESP_LOGI(TAG, "Calibration Data Read:");
    ESP_LOGI(TAG, "AC1: %d, AC2: %d, AC3: %d", calib_data.AC1, calib_data.AC2, calib_data.AC3);
    ESP_LOGI(TAG, "AC4: %u, AC5: %u, AC6: %u", calib_data.AC4, calib_data.AC5, calib_data.AC6);
    ESP_LOGI(TAG, "B1: %d, B2: %d", calib_data.B1, calib_data.B2);
    ESP_LOGI(TAG, "MB: %d, MC: %d, MD: %d", calib_data.MB, calib_data.MC, calib_data.MD);

    return ESP_OK;
}

esp_err_t bmp180_init(void)
{
    esp_err_t ret;
    uint8_t chip_id;

    ret = bmp180_i2c_read_bytes(I2C_MASTER_NUM, BMP180_CHIP_ID_REG, &chip_id, 1);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read chip ID: %s", esp_err_to_name(ret));
        return ret;
    }

    if (chip_id != 0x55)
    {
        ESP_LOGE(TAG, "BMP180 not found, chip ID: 0x%02X (Expected 0x55)", chip_id);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "BMP180 found, chip ID: 0x%02X", chip_id);

    ret = bmp180_read_calibration_data();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read BMP180 calibration data.");
        return ret;
    }

    return ESP_OK;
}

int32_t bmp180_read_raw_temp(void)
{
    bmp180_i2c_write_byte(I2C_MASTER_NUM, BMP180_CTRL_MEAS_REG, BMP180_READ_TEMP_CMD);
    vTaskDelay(pdMS_TO_TICKS(5));
    uint8_t raw_data[2];
    bmp180_i2c_read_bytes(I2C_MASTER_NUM, BMP180_TEMP_CAL_MSB_REG, raw_data, 2);
    return (int32_t)((raw_data[0] << 8) | raw_data[1]);
}

int32_t bmp180_read_raw_pressure(void)
{
    uint8_t pressure_cmd = BMP180_READ_PRESSURE_0_CMD + (BMP180_OSS << 6);
    bmp180_i2c_write_byte(I2C_MASTER_NUM, BMP180_CTRL_MEAS_REG, pressure_cmd);

    int delay_ms;
    switch (BMP180_OSS)
    {
    case 0:
        delay_ms = 5;
        break;
    case 1:
        delay_ms = 8;
        break;
    case 2:
        delay_ms = 14;
        break;
    case 3:
        delay_ms = 26;
        break;
    default:
        delay_ms = 5;
        break;
    }
    vTaskDelay(pdMS_TO_TICKS(delay_ms));

    uint8_t raw_data[3];
    bmp180_i2c_read_bytes(I2C_MASTER_NUM, BMP180_PRESS_CAL_MSB_REG, raw_data, 3);
    return (int32_t)(((raw_data[0] << 16) | (raw_data[1] << 8) | raw_data[2]) >> (8 - BMP180_OSS));
}

//todo fix, it's reading gibberish
float bmp180_compensate_temperature(int32_t UT)
{
    long X1 = (UT - (long)calib_data.AC6) * calib_data.AC5 / 32768;
    long X2 = ((long)calib_data.MC * 2048) / (X1 + calib_data.MD);
    B5 = X1 + X2;
    float T = (B5 + 8) / 16.0;
    return T / 10.0;
}

long bmp180_compensate_pressure(int32_t UP)
{
    long B6 = B5 - 4000;
    long X1 = (calib_data.B2 * (B6 * B6 / 4096)) / 2048;
    long X2 = (calib_data.AC2 * B6) / 2048;
    long X3 = X1 + X2;
    long B3 = (((long)calib_data.AC1 * 4 + X3) << BMP180_OSS) / 4;

    X1 = (calib_data.AC3 * B6) / 8192;
    X2 = (calib_data.B1 * ((B6 * B6) / 2048)) / 16384;
    X3 = ((X1 + X2) + 2) / 4;
    uint32_t B4 = (calib_data.AC4 * (uint32_t)(X3 + 32768)) / 1000;
    uint32_t B7 = ((uint32_t)UP - B3) * (50000 >> BMP180_OSS);

    long p;
    if (B7 < 0x80000000)
    {
        p = (B7 * 2) / B4;
    }
    else
    {
        p = (B7 / B4) * 2;
    }

    X1 = (p / 256) * (p / 256);
    X1 = (X1 * 3038) / 65536;
    X2 = (-7357 * p) / 65536;
    p = p + ((X1 + X2 + 3791) / 16);

    return p;
}