#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include <time.h>
#include <stdbool.h>

#define PROJECT_GITHUB_URL "https://github.com/maunope/DifferentiaPressureSensor/"

// --- Command execution status for UI feedback and internal state ---
typedef enum {
    CMD_STATUS_IDLE,
    CMD_STATUS_PENDING,
    CMD_STATUS_SUCCESS,
    CMD_STATUS_FAIL
} command_status_t;

typedef enum {
    WRITE_STATUS_OK,
    WRITE_STATUS_FAIL,
    WRITE_STATUS_UNKNOWN
} write_status_t;

typedef enum {
    WEB_SERVER_STOPPED,
    WEB_SERVER_STARTING,
    WEB_SERVER_RUNNING,
    WEB_SERVER_FAILED,
    WEB_SERVER_USB_CONNECTED,
} web_server_status_t;

// Shared sensor data buffer
typedef struct {
    write_status_t writeStatus;
    web_server_status_t web_server_status;
    char web_server_url[64];
    long pressure_pa;
    float temperature_c;
    float diff_pressure_pa;
    time_t timestamp;
    time_t last_successful_write_ts; // Timestamp of the last successful write
    bool timestamp_from_rtc; 
    float battery_voltage;
    int battery_percentage;
    int battery_externally_powered;
    int sd_card_file_count;
    int sd_card_free_bytes;
    bool high_freq_mode_enabled;
    uint64_t uptime_seconds;
    char file_to_delete[64];
    bool bmp280_available;
    bool d6fph_available;
    bool ds3231_available;
    bool sensor_read_error;
    command_status_t delete_file_status;
    bool datalogger_paused;
    uint32_t sd_card_free_space_mb;
} sensor_buffer_t;

typedef enum
{
    DATALOGGER_MODE_NORMAL,
    DATALOGGER_MODE_HF,
    DATALOGGER_MODE_PAUSED,
} datalogger_mode_t;

// Global buffer and mutex declarations
extern sensor_buffer_t g_sensor_buffer;
extern SemaphoreHandle_t g_sensor_buffer_mutex;
extern SemaphoreHandle_t g_i2c_bus_mutex;
extern QueueHandle_t g_datalogger_cmd_queue;

// Event group to signal initialization completion
extern EventGroupHandle_t g_init_event_group;
#define INIT_DONE_BIT BIT0

// --- Main app command queue ---
typedef enum {
    APP_CMD_NONE,
    APP_CMD_SET_RTC_BUILD_TIME,
    APP_CMD_GET_SD_FILE_COUNT,
    APP_CMD_GET_SD_FREE_SPACE,
    APP_CMD_FORMAT_SD_CARD,
    APP_CMD_DELETE_FILE,
    APP_CMD_ACTIVITY_DETECTED,
    APP_CMD_SYNC_RTC_NTP,
    APP_CMD_REFRESH_SENSOR_DATA,
    APP_CMD_START_WEB_SERVER,
    APP_CMD_STOP_WEB_SERVER,
    APP_CMD_SET_DATALOGGER_MODE,
    APP_CMD_LOG_COMPLETE_SLEEP_NOW,
} app_command_type_t;

typedef struct {
    app_command_type_t cmd;
    datalogger_mode_t mode; // Payload for SET_DATALOGGER_MODE
} app_command_t;

typedef enum {
    WEB_SERVER_FSM_IDLE,
    WEB_SERVER_FSM_CONNECTING,
} web_server_fsm_state_t;


extern QueueHandle_t g_app_cmd_queue;

extern volatile command_status_t g_command_status;
extern SemaphoreHandle_t g_command_status_mutex;
