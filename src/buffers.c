#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include <math.h>
#include "../lib/buffers.h"

/**
 * @file buffers.c
 * @brief Defines the global shared data structures and RTOS synchronization primitives.
 *
 * This file serves as the central repository for all shared state in the application,
 * including the main sensor data buffer and the various mutexes and queues used for
 * inter-task communication and resource protection.
 */

// --- Global buffer and mutex definition ---
// This is the single source of truth for the global sensor buffer.
sensor_buffer_t g_sensor_buffer = {
    .temperature_c = NAN,
    .pressure_pa = 0, // Use 0 as the sentinel for an invalid integer reading
    .diff_pressure_pa = NAN,
    .battery_voltage = NAN,
    .battery_percentage = 0,
    .battery_externally_powered = 0,
    .writeStatus = WRITE_STATUS_UNKNOWN,
    .timestamp = 0,
    .last_successful_write_ts = 0,
    .sd_card_file_count = -2, // -2 indicates loading
    .sd_card_free_bytes = -2, // -2 indicates loading
    .high_freq_mode_enabled = false,
    .uptime_seconds = 0,
    .bmp280_available = false,
    .d6fph_available = false,
    .ds3231_available = false,
    .usb_msc_connected = false,
    .sensor_read_error = false,
    .datalogger_paused = false,
};

SemaphoreHandle_t g_sensor_buffer_mutex = NULL;
QueueHandle_t g_app_cmd_queue = NULL;
SemaphoreHandle_t g_command_status_mutex = NULL;
SemaphoreHandle_t g_i2c_bus_mutex = NULL;
volatile command_status_t g_command_status = CMD_STATUS_IDLE;
QueueHandle_t g_datalogger_cmd_queue = NULL;

// Event group to signal when main initialization is complete
EventGroupHandle_t g_init_event_group = NULL;