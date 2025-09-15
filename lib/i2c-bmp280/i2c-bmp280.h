#ifndef I2C_BMP280_H
#define I2C_BMP280_H

#include <stdint.h>
#include "esp_err.h"

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

// Function prototypes
esp_err_t bmp280_init(void);
int32_t bmp280_read_raw_temp(void);
int32_t bmp280_read_raw_pressure(void);
float bmp280_compensate_temperature(int32_t adc_T);
long bmp280_compensate_pressure(int32_t adc_P);

#endif // I2C_BMP280_H
