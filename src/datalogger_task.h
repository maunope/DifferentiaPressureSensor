#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// --- Datalogger command queue ---
typedef enum {
    DATALOGGER_CMD_NONE,
    DATALOGGER_CMD_FORCE_REFRESH,
    DATALOGGER_CMD_PAUSE_WRITES,
    DATALOGGER_CMD_RESUME_WRITES,
} datalogger_command_t;

extern QueueHandle_t g_datalogger_cmd_queue;

/**
 * @brief Task responsible for periodically reading sensor data and logging it to the SD card.
 */
void datalogger_task(void *pvParameters);