// main.c
// ESP-IDF script for reading I2C barometric sensor (BMP180) and printing data.

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/i2c.h" // Include I2C driver
#include "sdkconfig.h"  // For CONFIG_I2C_MASTER_SCL and CONFIG_I2C_MASTER_SDA

// --- I2C Configuration ---
#define I2C_MASTER_SCL_IO           GPIO_NUM_21    // GPIO pin for I2C SCL
#define I2C_MASTER_SDA_IO           GPIO_NUM_20    // GPIO pin for I2C SDA
#define I2C_MASTER_NUM              I2C_NUM_0      // I2C port number
#define I2C_MASTER_FREQ_HZ          100000         // I2C clock frequency
#define I2C_MASTER_TX_BUF_DISABLE   0              // I2C master does not need buffer
#define I2C_MASTER_RX_BUF_DISABLE   0              // I2C master does not need buffer
#define I2C_MASTER_TIMEOUT_MS       1000           // I2C transaction timeout

// --- BMP180 Sensor Specifics ---
#define BMP180_SENSOR_ADDR          0x77           // I2C address of BMP180 (SDA HIGH)
#define BMP180_CHIP_ID_REG          0xD0           // Chip ID register (should be 0x55)
#define BMP180_CTRL_MEAS_REG        0xF4           // Control Measurement register
#define BMP180_TEMP_CAL_MSB_REG     0xF6           // Uncompensated Temperature MSB
#define BMP180_PRESS_CAL_MSB_REG    0xF6           // Uncompensated Pressure MSB (same register but different data length)
#define BMP180_CALIB_DATA_START     0xAA           // Calibration data start address

// Control register values for measurement
#define BMP180_READ_TEMP_CMD        0x2E
#define BMP180_READ_PRESSURE_0_CMD  0x34 // OSS = 0, ultra low power
#define BMP180_READ_PRESSURE_1_CMD  0x74 // OSS = 1, standard
#define BMP180_READ_PRESSURE_2_CMD  0xB4 // OSS = 2, high res
#define BMP180_READ_PRESSURE_3_CMD  0xF4 // OSS = 3, ultra high res

// Oversampling Setting (OSS) - Adjust as needed
// 0: ultra low power, 1: standard, 2: high resolution, 3: ultra high resolution
#define BMP180_OSS                  0 // For simplicity, using ultra low power (0)


static const char *TAG = "BMP180_SENSOR"; // Tag for ESP_LOG messages

// --- BMP180 Calibration Data Structure ---
typedef struct {
    int16_t AC1;
    int16_t AC2;
    int16_t AC3;
    uint16_t AC4;
    uint16_t AC5;
    uint16_t AC6;
    int16_t B1;
    int16_t B2;
    int16_t MB;
    int16_t MC;
    int16_t MD;
} bmp180_calib_data_t;

static bmp180_calib_data_t calib_data; // Global variable to store calibration data

// --- I2C Master Initialization ---
static esp_err_t i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C parameter config failed: %s", esp_err_to_name(err));
        return err;
    }
    err = i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(err));
    }
    return err;
}

// --- Custom I2C Master Write Byte (Renamed to avoid conflict) ---
static esp_err_t bmp180_i2c_write_byte(i2c_port_t i2c_num, uint8_t reg_addr, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMP180_SENSOR_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BMP180 I2C write byte failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

// --- Custom I2C Master Read Bytes (Renamed to avoid conflict) ---
static esp_err_t bmp180_i2c_read_bytes(i2c_port_t i2c_num, uint8_t reg_addr, uint8_t *data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMP180_SENSOR_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd); // Repeated start
    i2c_master_write_byte(cmd, (BMP180_SENSOR_ADDR << 1) | I2C_MASTER_READ, true);
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BMP180 I2C read bytes failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

// --- Read a 16-bit signed calibration value ---
static int16_t read_s16(uint8_t reg_addr) {
    uint8_t raw_data[2];
    bmp180_i2c_read_bytes(I2C_MASTER_NUM, reg_addr, raw_data, 2);
    return (int16_t)((raw_data[0] << 8) | raw_data[1]);
}

// --- Read a 16-bit unsigned calibration value ---
static uint16_t read_u16(uint8_t reg_addr) {
    uint8_t raw_data[2];
    bmp180_i2c_read_bytes(I2C_MASTER_NUM, reg_addr, raw_data, 2);
    return (uint16_t)((raw_data[0] << 8) | raw_data[1]);
}

// --- Read BMP180 Calibration Data ---
static esp_err_t bmp180_read_calibration_data(void) {
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

// --- BMP180 Initialization ---
static esp_err_t bmp180_init(void) {
    esp_err_t ret;
    uint8_t chip_id;

    // Read chip ID to confirm sensor presence
    ret = bmp180_i2c_read_bytes(I2C_MASTER_NUM, BMP180_CHIP_ID_REG, &chip_id, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read chip ID: %s", esp_err_to_name(ret));
        return ret;
    }

    if (chip_id != 0x55) { // BMP180 chip ID is 0x55
        ESP_LOGE(TAG, "BMP180 not found, chip ID: 0x%02X (Expected 0x55)", chip_id);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "BMP180 found, chip ID: 0x%02X", chip_id);

    // Read calibration data
    ret = bmp180_read_calibration_data();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read BMP180 calibration data.");
        return ret;
    }

    return ESP_OK;
}

// --- Read Uncompensated Temperature (UT) ---
static int32_t bmp180_read_raw_temp(void) {
    bmp180_i2c_write_byte(I2C_MASTER_NUM, BMP180_CTRL_MEAS_REG, BMP180_READ_TEMP_CMD);
    vTaskDelay(pdMS_TO_TICKS(5)); // Wait for 4.5ms conversion time for temperature
    uint8_t raw_data[2];
    bmp180_i2c_read_bytes(I2C_MASTER_NUM, BMP180_TEMP_CAL_MSB_REG, raw_data, 2);
    return (int32_t)((raw_data[0] << 8) | raw_data[1]);
}

// --- Read Uncompensated Pressure (UP) ---
static int32_t bmp180_read_raw_pressure(void) {
    uint8_t pressure_cmd = BMP180_READ_PRESSURE_0_CMD + (BMP180_OSS << 6); // Combine command with OSS
    bmp180_i2c_write_byte(I2C_MASTER_NUM, BMP180_CTRL_MEAS_REG, pressure_cmd);

    // Delay based on oversampling setting (OSS)
    int delay_ms;
    switch (BMP180_OSS) {
        case 0: delay_ms = 5; break;  // 4.5ms
        case 1: delay_ms = 8; break;  // 7.5ms
        case 2: delay_ms = 14; break; // 13.5ms
        case 3: delay_ms = 26; break; // 25.5ms
        default: delay_ms = 5; break;
    }
    vTaskDelay(pdMS_TO_TICKS(delay_ms));

    uint8_t raw_data[3];
    bmp180_i2c_read_bytes(I2C_MASTER_NUM, BMP180_PRESS_CAL_MSB_REG, raw_data, 3);
    // BMP180 raw pressure is 19-bit (MSB, LSB, XLSB first 3 bits)
    return (int32_t)(((raw_data[0] << 16) | (raw_data[1] << 8) | raw_data[2]) >> (8 - BMP180_OSS));
}

// Global variable for temperature compensation
long B5;

// --- Compensate Temperature (returns temperature in 0.1 deg C) ---
float bmp180_compensate_temperature(int32_t UT) {
    long X1 = (UT - (long)calib_data.AC6) * calib_data.AC5 / 32768;
    long X2 = ((long)calib_data.MC * 2048) / (X1 + calib_data.MD);
    B5 = X1 + X2;
    float T = (B5 + 8) / 16.0; // Temperature in 0.1 deg C, convert to deg C
    return T / 10.0;
}

// --- Compensate Pressure (returns pressure in Pa) ---
long bmp180_compensate_pressure(int32_t UP) {
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
    if (B7 < 0x80000000) {
        p = (B7 * 2) / B4;
    } else {
        p = (B7 / B4) * 2;
    }

    X1 = (p / 256) * (p / 256);
    X1 = (X1 * 3038) / 65536;
    X2 = (-7357 * p) / 65536;
    p = p + ((X1 + X2 + 3791) / 16);

    return p; // Pressure in Pa
}

// --- Main Application Entry Point ---
void app_main(void) {
    // Initialize I2C
    esp_err_t err = i2c_master_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C master initialization failed, stopping.");
        return;
    }

    // Initialize BMP180 sensor (reads calibration data)
    err = bmp180_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "BMP180 sensor initialization failed, stopping.");
        return;
    }

    int32_t uncomp_temp, uncomp_press;
    float temperature_c;
    long pressure_pa;

    ESP_LOGI(TAG, "Starting BMP180 sensor readings...");

    while (1) {
        // 1. Read uncompensated temperature
        uncomp_temp = bmp180_read_raw_temp();
        if (uncomp_temp == -1) { // -1 could indicate an error from the read function
            ESP_LOGE(TAG, "Failed to read uncompensated temperature.");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        // 2. Compensate temperature
        temperature_c = bmp180_compensate_temperature(uncomp_temp);

        // 3. Read uncompensated pressure
        uncomp_press = bmp180_read_raw_pressure();
        if (uncomp_press == -1) { // -1 could indicate an error from the read function
            ESP_LOGE(TAG, "Failed to read uncompensated pressure.");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        // 4. Compensate pressure
        pressure_pa = bmp180_compensate_pressure(uncomp_press);

        ESP_LOGI(TAG, "Temperature: %.2f C, Pressure: %ld Pa", temperature_c, pressure_pa);

        vTaskDelay(pdMS_TO_TICKS(2000)); // Read sensor every 5 seconds
    }
}
