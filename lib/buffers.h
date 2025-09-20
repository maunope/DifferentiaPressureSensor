#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// Shared sensor data buffer
typedef struct {
    int writeStatus;
    float pressure_pa;
    float temperature_c;
} sensor_buffer_t;

// Global buffer and mutex declarations
extern sensor_buffer_t g_sensor_buffer;
extern SemaphoreHandle_t g_sensor_buffer_mutex;

