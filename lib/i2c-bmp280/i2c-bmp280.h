#ifndef I2C_BMP280_H
#define I2C_BMP280_H

#include "driver/i2c.h"
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
    i2c_port_t i2c_port;
    uint8_t i2c_addr;
    bmp280_calib_data_t calib_data;
} bmp280_t;

// Function prototypes
esp_err_t bmp280_init(bmp280_t *dev, i2c_port_t port, uint8_t addr);
int32_t bmp280_read_raw_temp(bmp280_t *dev);
int32_t bmp280_read_raw_pressure(bmp280_t *dev);
float bmp280_compensate_temperature(bmp280_t *dev, int32_t adc_T);
long bmp280_compensate_pressure(bmp280_t *dev, int32_t adc_P);

#endif // I2C_BMP280_H
