#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include <time.h>

// Shared sensor data buffer
typedef struct {
    int writeStatus;
    long pressure_pa;
    float temperature_c;
    time_t timestamp;
    bool timestamp_from_rtc; 
    float battery_voltage;
    int battery_percentage;
    int battery_externally_powered;
    int sd_card_file_count;
    int sd_card_free_bytes;
} sensor_buffer_t;

// Global buffer and mutex declarations
extern sensor_buffer_t g_sensor_buffer;
extern SemaphoreHandle_t g_sensor_buffer_mutex;
extern SemaphoreHandle_t g_i2c_bus_mutex;

// --- Main app command queue ---
typedef enum {
    APP_CMD_NONE,
    APP_CMD_SET_RTC_BUILD_TIME,
    APP_CMD_GET_SD_FILE_COUNT,
    APP_CMD_GET_SD_FREE_SPACE,
    APP_CMD_FORMAT_SD_CARD,
    APP_CMD_ACTIVITY_DETECTED,

    APP_CMD_REFRESH_SENSOR_DATA,
} app_command_t;

extern QueueHandle_t g_app_cmd_queue;

// --- Command execution status for UI feedback ---
typedef enum {
    CMD_STATUS_IDLE,
    CMD_STATUS_PENDING,
    CMD_STATUS_SUCCESS,
    CMD_STATUS_FAIL
} command_status_t;

extern volatile command_status_t g_command_status;
extern SemaphoreHandle_t g_command_status_mutex;
