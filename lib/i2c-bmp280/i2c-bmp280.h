#ifndef I2C_BMP280_H
#define I2C_BMP280_H

#include "driver/i2c_master.h"
#include <stdint.h>
#include "esp_err.h"

#define BMP280_SENSOR_ADDR 0x76

// BMP280 Calibration data structure
typedef struct {
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;
} bmp280_calib_data_t;

typedef struct {
    i2c_master_dev_handle_t i2c_dev_handle;
    bmp280_calib_data_t calib_data;
} bmp280_t;

// Function prototypes
esp_err_t bmp280_init(bmp280_t *dev, i2c_master_bus_handle_t bus_handle, uint8_t i2c_addr);

/**
 * @brief Initializes the BMP280 sensor with the default I2C address (0x76).
 * This is a convenience macro that calls bmp280_init with the default address.
 */
#define bmp280_init_default(dev, port) bmp280_init(dev, port, BMP280_SENSOR_ADDR)

int32_t bmp280_read_raw_temp(bmp280_t *dev);
int32_t bmp280_read_raw_pressure(bmp280_t *dev);
float bmp280_compensate_temperature(bmp280_t *dev, int32_t adc_T);
long bmp280_compensate_pressure(bmp280_t *dev, int32_t adc_P);
esp_err_t bmp280_force_read(bmp280_t *dev, float *temperature, long *pressure);

/**
 * @brief Reads compensated temperature and pressure data from the BMP280 sensor.
 *
 * @param dev Pointer to the BMP280 device descriptor.
 * @param temperature Pointer to a float to store the compensated temperature in degrees Celsius.
 * @param pressure Pointer to a long to store the compensated pressure in Pascals.
 * @return esp_err_t `ESP_OK` on success, or an error code on failure.
 */
esp_err_t bmp280_read_fixed(bmp280_t *dev, int32_t *temperature, uint32_t *pressure);

#endif // I2C_BMP280_H
