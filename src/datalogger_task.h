#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "../lib/i2c-bmp280/i2c-bmp280.h"
#include "../lib/i2c-d6fph/i2c-d6fph.h"

// --- Datalogger command queue ---
typedef enum {
    DATALOGGER_CMD_NONE,
    DATALOGGER_CMD_FORCE_REFRESH,
    /**
     * @brief Command to request the datalogger task to pause its operations and suspend itself.
     *
     * This command-based approach is used instead of forcefully suspending the task from
     * an external context (e.g., from main_task). It allows the datalogger task to finish
     * any critical operations (like an SD card write) and suspend itself at a predictable
     * point in its loop, preventing race conditions, deadlocks, and ensuring data integrity.
     */
    DATALOGGER_CMD_PAUSE_WRITES,
    DATALOGGER_CMD_ROTATE_FILE,
} datalogger_command_t;

extern QueueHandle_t g_datalogger_cmd_queue;

typedef struct {
    bmp280_t *bmp280_dev;
    d6fph_t *d6fph_dev;
    uint32_t log_interval_ms;
    uint32_t hf_log_interval_ms;
} datalogger_task_params_t;

/**
 * @brief Task responsible for periodically reading sensor data and logging it to the SD card.
 *
 * @param pvParameters A pointer to an initialized datalogger_task_params_t struct.
 */
void datalogger_task(void *pvParameters);