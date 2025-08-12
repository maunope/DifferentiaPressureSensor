#pragma once

#include "esp_err.h"

typedef struct
{
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

esp_err_t bmp180_init(void);
esp_err_t bmp180_read_calibration_data(void);
int32_t bmp180_read_raw_temp(void);
int32_t bmp180_read_raw_pressure(void);
float bmp180_compensate_temperature(int32_t UT);
long bmp180_compensate_pressure(int32_t UP);
