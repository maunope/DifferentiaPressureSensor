// ESP-IDF script for reading I2C barometric sensor (BMP280) and rotary encoder,
// printing data to serial and a 2-line I2C LCD using a self-contained driver.

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h" // Required for semaphore
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "sdkconfig.h"
#include "esp_rom_sys.h"
#include "esp_timer.h" // Required for esp_timer_get_time()
#include <time.h>
#include "esp_ota_ops.h"
#include <sys/time.h>
#include <sys/stat.h>
#include "esp_sleep.h"
#include "esp_sntp.h"
#include <unistd.h> // For unlink()
#include <errno.h>  // For errno
#include "esp_netif.h"

#include "../lib/i2c_bmp280/i2c_bmp280.h"
#include "../lib/i2c_d6fph/i2c_d6fph.h"
#include "../lib/i2c_ds3231/i2c_ds3231.h"
#include "../lib/i2c_oled/i2c_oled.h"
#include "../lib/rotary_encoder/rotary_encoder.h"
#include "../lib/spi_sdcard/spi_sdcard.h"
#include "../lib/buffers.h"
#include "../lib/config_manager/config_manager.h"
#include "../lib/lipo_battery/lipo_battery.h"
#include "../lib/ntp_client/ntp_client.h"
#include "../lib/wifi_manager/wifi_manager.h"
#include "../lib/web_server/web_server.h"
#include "ui_render.h"
#include "datalogger_task.h"
#include "config_params.h"
// DO NOT use pins 19 and 20, they are used by the flash memory
// DO NOT use pins 5 and 6, they are used for LiPo battery measurements unless you change the config

#define UNKNOWN_FILE_WRITE_STATUS 2

#define DEVICES_POWER_PIN GPIO_NUM_15
#define OLED_POWER_PIN GPIO_NUM_16

// --- I2C Configuration for peripherals ---
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_SCL_IO GPIO_NUM_37
#define I2C_MASTER_SDA_IO GPIO_NUM_38
#define I2C_MASTER_FREQ_HZ 100000 // 100kHz I2C clock

// --- I2C Configuration for OLED ---
#define I2C_OLED_NUM I2C_NUM_1
#define I2C_OLED_SCL_IO GPIO_NUM_39
#define I2C_OLED_SDA_IO GPIO_NUM_40

// --- Rotary Encoder Configuration ---
#define ROTARY_ENCODER_PIN_A GPIO_NUM_41
#define ROTARY_ENCODER_PIN_B GPIO_NUM_42
#define ROTARY_ENCODER_BUTTON_PIN GPIO_NUM_2
#define BUTTON_DEBOUNCE_TIME_MS 80

// --- Battery ADC Configuration ---
#define BATTERY_PWR_PIN GPIO_NUM_5 // This pin is used to enable the battery voltage divider
#define BATTERY_ADC_PIN GPIO_NUM_6

#define OLED_I2C_ADDR 0x3C
#define DS3231_I2C_ADDR 0x68
#define BMP280_I2C_ADDR 0x76
#define D6FPH_I2C_ADDR 0x6C

// --- RTC Memory for State Persistence ---
// These variables retain their values across deep sleep cycles.
static RTC_DATA_ATTR write_status_t rtc_last_write_status = WRITE_STATUS_UNKNOWN;
static RTC_DATA_ATTR time_t rtc_last_successful_write_ts = 0;
static RTC_DATA_ATTR datalogger_mode_t rtc_datalogger_mode = DATALOGGER_MODE_NORMAL;
static RTC_DATA_ATTR uint64_t rtc_total_awake_time_s = 0;
static RTC_DATA_ATTR uint64_t rtc_last_boot_time_ms = 0;

static const char *TAG = "main";

typedef enum
{
    I2C_BUS_SENSORS, // I2C_NUM_0 for main peripherals
    I2C_BUS_OLED,    // I2C_NUM_1 for the display
    I2C_BUS_ALL      // Both buses
} i2c_bus_target_t;

ds3231_t g_rtc;    // Global RTC device handle
bmp280_t g_bmp280; // Global BMP280 device handle
d6fph_t g_d6fph;   // Global D6F-PH device handle

TaskHandle_t g_datalogger_task_handle = NULL;
TaskHandle_t g_uiRender_task_handle = NULL;
i2c_master_bus_handle_t g_i2c_bus0_handle = NULL;
i2c_master_bus_handle_t g_i2c_bus1_handle = NULL;

const config_params_t *g_cfg = NULL;

static uint64_t last_activity_ms = 0;
static bool is_usb_connected_state = false;

static web_server_fsm_state_t s_web_server_state = WEB_SERVER_FSM_IDLE;

// Forward declaration to resolve implicit declaration warning
static void handle_battery_refresh(void)
{
    ESP_LOGI(TAG, "Refreshing battery status on USB state change.");
    if (g_datalogger_cmd_queue != NULL)
    {
        if (xQueueSend(g_datalogger_cmd_queue, &(datalogger_cmd_msg_t){.cmd = DATALOGGER_CMD_REFRESH_BATTERY}, 0) != pdPASS)
        {
            ESP_LOGE(TAG, "Failed to send battery-only refresh command to datalogger queue");
        }
    }
}

static bool handle_stop_web_server(void);

/**
 * @brief Callback for clockwise rotation from the rotary encoder.
 * this method is executed in the rotary econder task, hence it makes sense to send a message to main task here
 */
static void on_encoder_rotate_cw(void)
{
    uiRender_send_event(UI_EVENT_CW, NULL, 0);
    app_command_t cmd = {.cmd = APP_CMD_ACTIVITY_DETECTED};
    xQueueSend(g_app_cmd_queue, &cmd, 0);
}

/**
 * @brief Callback for counter-clockwise rotation from the rotary encoder.
 * this method is executed in the rotary econder task, hence it makes sense to send a message to main task here
 */
static void on_encoder_rotate_ccw(void)
{

    uiRender_send_event(UI_EVENT_CCW, NULL, 0);
    app_command_t cmd = {.cmd = APP_CMD_ACTIVITY_DETECTED};
    xQueueSend(g_app_cmd_queue, &cmd, 0);
}

/**
 * @brief Callback for button press from the rotary encoder.
 * this method is executed in the rotary econder task, hence it makes sense to send a message to main task here
 */
static void on_encoder_button_press(void)
{

    uiRender_send_event(UI_EVENT_BTN, NULL, 0);
    app_command_t cmd = {.cmd = APP_CMD_ACTIVITY_DETECTED};
    xQueueSend(g_app_cmd_queue, &cmd, 0);
}

/**
 * @brief Callback for button long press from the rotary encoder.
 * this method is executed in the rotary econder task, hence it makes sense to send a message to main task here
 */
static void on_encoder_button_long_press(void)
{
    uiRender_send_event(UI_EVENT_BTN_LONG, NULL, 0);
    app_command_t cmd = {.cmd = APP_CMD_ACTIVITY_DETECTED};
    xQueueSend(g_app_cmd_queue, &cmd, 0);
}

/**
 * @brief Sets the DS3231 RTC to the application's build time.
 *
 * This function is typically called when the RTC time is found to be invalid
 * at startup. It uses the `__DATE__` and `__TIME__` macros to get the compile
 * time, then sets the RTC and the system time accordingly. It also handles
 * I2C bus mutex and updates a global status variable for UI feedback.
 */
static void handle_set_rtc_to_build_time(void)
{

    esp_err_t ret = ESP_FAIL;
    if (xSemaphoreTake(g_i2c_bus_mutex, portMAX_DELAY))
    {
        ret = ds3231_set_time_to_build_time(&g_rtc);
        xSemaphoreGive(g_i2c_bus_mutex);
    }

    // Update the global command status
    if (xSemaphoreTake(g_command_status_mutex, portMAX_DELAY))
    {
        g_command_status = (ret == ESP_OK) ? CMD_STATUS_SUCCESS : CMD_STATUS_FAIL;
        xSemaphoreGive(g_command_status_mutex);
    }
}

/**
 * @brief Synchronizes the DS3231 RTC with an NTP server.
 *
 * This function retrieves Wi-Fi credentials from the configuration,
 * uses the ntp-client library to get the current UTC time, and then
 * sets the DS3231 RTC and the system time. It provides feedback
 * to the UI via the global command status.
 */
static void handle_sync_rtc_with_ntp(void)
{
    ESP_LOGI(TAG, "Starting NTP time synchronization...");

    // Check if Wi-Fi SSID is configured before attempting to connect.
    if (strlen(g_cfg->wifi_ssid) == 0)
    {
        ESP_LOGE(TAG, "Wi-Fi SSID is not configured. Cannot sync time with NTP.");
        if (xSemaphoreTake(g_command_status_mutex, portMAX_DELAY))
        {
            g_command_status = CMD_STATUS_FAIL;
            xSemaphoreGive(g_command_status_mutex);
        }
        return;
    }

    time_t current_utc_time;
    esp_err_t ret = ntp_client_get_utc_time(g_cfg->wifi_ssid, g_cfg->wifi_password, 30000, &current_utc_time);

    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "NTP sync successful. The current UTC time is: %lld", (long long)current_utc_time);

        // Now, set the DS3231 RTC with the obtained time
        if (xSemaphoreTake(g_i2c_bus_mutex, portMAX_DELAY))
        {
            ret = ds3231_set_time_t(&g_rtc, current_utc_time);
            xSemaphoreGive(g_i2c_bus_mutex);
        }

        if (ret == ESP_OK)
        {
            ESP_LOGI(TAG, "DS3231 RTC updated successfully.");
        }
        else
        {
            ESP_LOGE(TAG, "Failed to update DS3231 RTC.");
        }
    }

    // Update the global command status for UI feedback
    if (xSemaphoreTake(g_command_status_mutex, portMAX_DELAY))
    {
        g_command_status = (ret == ESP_OK) ? CMD_STATUS_SUCCESS : CMD_STATUS_FAIL;
        xSemaphoreGive(g_command_status_mutex);
    }
}

/**
 * @brief Powers off the OLED display and suspends the UI rendering task.
 *
 * This function sends a "prepare for sleep" event to the UI, waits for it
 * to be processed, then suspends the UI task and cuts power to the OLED.
 */
static void oled_power_off(void)
{
    // Only act if the UI task exists and is not already suspended.
    if (g_uiRender_task_handle != NULL && eTaskGetState(g_uiRender_task_handle) != eSuspended)
    {
        ESP_LOGI(TAG, "Powering off OLED.");
        vTaskSuspend(g_uiRender_task_handle);
        uiRender_send_event(UI_EVENT_PREPARE_SLEEP, NULL, 0);
        vTaskDelay(pdMS_TO_TICKS(200)); // Give UI time to draw sleep message
        // i2c_driver_delete(I2C_OLED_NUM);
        gpio_set_level(OLED_POWER_PIN, 0);
    }
}

/**
 * @brief Powers on the OLED display and resumes the UI rendering task.
 *
 * This function restores power to the OLED, re-initializes the UI driver,
 * resumes the UI task, and sends a "wake up" event. It's called after
 * waking from sleep due to user activity.
 */
static void oled_power_on(void)
{
    // Only act if the UI task exists and is currently suspended.
    if (g_uiRender_task_handle != NULL && eTaskGetState(g_uiRender_task_handle) == eSuspended)
    {
        ESP_LOGI(TAG, "Powering on OLED.");
        gpio_set_level(OLED_POWER_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(100)); // Wait for OLED to stabilize
        i2c_oled_send_init_commands(I2C_OLED_NUM);
        // Immediately clear the screen and show a "Waking up" message
        // to prevent showing stale "Zzzzz" screen content.
        i2c_oled_clear(I2C_OLED_NUM);
        i2c_oled_write_inverted_text(I2C_OLED_NUM, 0, 0, "System");
        i2c_oled_write_text(I2C_OLED_NUM, 2, 0, "Waking up...");
        i2c_oled_update_screen(I2C_OLED_NUM);
        vTaskResume(g_uiRender_task_handle);
        uiRender_send_event(UI_EVENT_WAKE_UP, NULL, 0);
    }
}

static void go_to_deep_sleep(void)
{
    ESP_LOGI(TAG, "Saving state and entering deep sleep.");

    // --- Persist State to RTC Memory ---
    // Save the current datalogger state just before sleeping.
    // This is done AFTER the datalogger is confirmed to be paused to ensure we save the very last state.
    if (xSemaphoreTake(g_sensor_buffer_mutex, pdMS_TO_TICKS(100)))
    {
        // The most critical value is the timestamp of the last successful write.
        // Saving this correctly prevents the race condition where the device wakes
        // and immediately tries to sleep again.
        rtc_last_successful_write_ts = g_sensor_buffer.last_successful_write_ts;
        rtc_datalogger_mode = g_sensor_buffer.datalogger_paused ? DATALOGGER_MODE_PAUSED : (g_sensor_buffer.high_freq_mode_enabled ? DATALOGGER_MODE_HF : DATALOGGER_MODE_NORMAL);
        rtc_last_write_status = g_sensor_buffer.writeStatus;
        xSemaphoreGive(g_sensor_buffer_mutex);

        uint64_t current_awake_time_s = (esp_timer_get_time() - rtc_last_boot_time_ms * 1000) / 1000000ULL;
        rtc_total_awake_time_s += current_awake_time_s;
        ESP_LOGI(TAG, "Saved last write time and status (%d) to RTC memory.", rtc_last_write_status);
    }
    else
    {
        ESP_LOGE(TAG, "Failed to acquire buffer mutex to save state. State will not be persisted.");
    }

    // Configure the rotary encoder button pin as the wakeup source.
    // This uses the RTC peripheral, which is required for deep sleep.
    rotaryencoder_enable_wakeup_source();

    sensor_buffer_t local_buffer;
    if (xSemaphoreTake(g_sensor_buffer_mutex, pdMS_TO_TICKS(50)))
    {
        local_buffer = g_sensor_buffer;
        xSemaphoreGive(g_sensor_buffer_mutex);
    }
    else
    {
        // In case of failure, assume default mode to be safe.
        local_buffer.high_freq_mode_enabled = false;
        local_buffer.last_successful_write_ts = 0;
    }

    // --- Smart Sleep Duration Calculation ---
    uint64_t configured_sleep_duration_ms = local_buffer.high_freq_mode_enabled ? g_cfg->hf_sleep_duration_ms : g_cfg->sleep_duration_ms;
    uint32_t log_interval_ms = local_buffer.high_freq_mode_enabled ? g_cfg->hf_log_interval_ms : g_cfg->log_interval_ms;

    time_t current_ts = time(NULL);
    time_t next_log_ts = local_buffer.last_successful_write_ts + (log_interval_ms / 1000);
    int64_t time_to_next_log_ms = (next_log_ts > current_ts) ? (next_log_ts - current_ts) * 1000 : 0;

    // Sleep for the configured duration, but no longer than the time remaining until the next log.
    // This prevents oversleeping and ensures timely writes. Add a small buffer (e.g., 100ms) to wake up just before.
    uint64_t sleep_duration = configured_sleep_duration_ms;
    if (time_to_next_log_ms > 100 && time_to_next_log_ms < configured_sleep_duration_ms)
    {
        sleep_duration = time_to_next_log_ms - 100;
    }
    esp_sleep_enable_timer_wakeup(sleep_duration * 1000);

    // IMPORTANT: Ensure the wakeup pin has a pull-up enabled to prevent floating.
    // This is critical to prevent immediate wake-up if the pin is not externally pulled up.
    // Use rtc_gpio functions as digital GPIO pads are powered down in deep sleep.
    rtc_gpio_pullup_en(ROTARY_ENCODER_BUTTON_PIN);
    rtc_gpio_pulldown_dis(ROTARY_ENCODER_BUTTON_PIN);

    // Disable pull-ups on non-RTC encoder pins to prevent current leakage.
    // These pins are not RTC-capable, so we use standard GPIO functions.
    gpio_pullup_dis(ROTARY_ENCODER_PIN_A);
    gpio_pullup_dis(ROTARY_ENCODER_PIN_B);

    // Hold the state of non-RTC GPIOs during deep sleep to prevent leakage.
    gpio_hold_en(ROTARY_ENCODER_PIN_A);
    gpio_hold_en(ROTARY_ENCODER_PIN_B);
    gpio_deep_sleep_hold_en();

    ESP_LOGI(TAG, "Entering deep sleep for %llu ms.", sleep_duration);
    esp_deep_sleep_start();
    // --- Execution stops here, device will restart on wakeup ---
}

static void handle_start_web_server(void)
{
    // Check if USB is connected as Mass Storage. If so, don't start the web server.
    if (spi_sdcard_is_usb_connected())
    {
        ESP_LOGW(TAG, "Cannot start web server while USB is connected as mass storage.");
        if (xSemaphoreTake(g_sensor_buffer_mutex, portMAX_DELAY))
        {
            g_sensor_buffer.web_server_status = WEB_SERVER_USB_CONNECTED;
            xSemaphoreGive(g_sensor_buffer_mutex);
        }
        return;
    }

    if (xSemaphoreTake(g_sensor_buffer_mutex, portMAX_DELAY))
    {
        g_sensor_buffer.web_server_status = WEB_SERVER_STARTING;
        xSemaphoreGive(g_sensor_buffer_mutex);
    }
    s_web_server_state = WEB_SERVER_FSM_CONNECTING;
    if (wifi_manager_connect(g_cfg->wifi_ssid, g_cfg->wifi_password) == ESP_OK)
    {
        // Wi-Fi connection process will be handled by the FSM in WEB_SERVER_FSM_CONNECTING
    }
    else
    {
        ESP_LOGE(TAG, "Failed to initiate Wi-Fi connection for web server.");
        handle_stop_web_server(); // Clean up on failure
    }
}

static bool handle_stop_web_server(void)
{
    s_web_server_state = WEB_SERVER_FSM_IDLE;
    ESP_LOGI(TAG, "Stopping web server and Wi-Fi.");
    stop_web_server();
    if (xSemaphoreTake(g_sensor_buffer_mutex, portMAX_DELAY))
    {
        g_sensor_buffer.web_server_status = WEB_SERVER_STOPPED;
        g_sensor_buffer.web_server_url[0] = '\0';
        xSemaphoreGive(g_sensor_buffer_mutex);
    }

    // Disconnect from Wi-Fi only if it's actually connected
    if (wifi_manager_is_connected())
    {
        wifi_manager_disconnect();
    }
    return true;
}

static void handle_web_server_fsm(void)
{
    // --- Web Server State Machine ---
    // This state is entered after the datalogger has successfully paused.
    if (s_web_server_state == WEB_SERVER_FSM_CONNECTING)
    {
        if (wifi_manager_is_connected())
        {
            ESP_LOGI(TAG, "Wi-Fi connected. Starting web server.");
            start_web_server();

            esp_netif_ip_info_t ip_info;
            esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
            if (netif && esp_netif_get_ip_info(netif, &ip_info) == ESP_OK && ip_info.ip.addr != 0)
            {
                if (xSemaphoreTake(g_sensor_buffer_mutex, portMAX_DELAY))
                {
                    snprintf(g_sensor_buffer.web_server_url, sizeof(g_sensor_buffer.web_server_url), "http://" IPSTR, IP2STR(&ip_info.ip));
                    g_sensor_buffer.web_server_status = WEB_SERVER_RUNNING;
                    xSemaphoreGive(g_sensor_buffer_mutex);
                }
                s_web_server_state = WEB_SERVER_FSM_IDLE; // Transition to idle, but server is running
            }
            else
            {
                ESP_LOGE(TAG, "Could not get IP address. Stopping web server.");
                handle_stop_web_server();
            }
        }
    }
}

/**
 * @brief i2c master initialization
 *
 * Configures and installs the I2C driver for the main peripheral bus (I2C_NUM_0).
 */
static esp_err_t i2c_bus_init(void)
{
    esp_err_t ret = ESP_OK;

    // --- Init I2C Bus 0 for Sensors (if not already initialized) ---
    if (g_i2c_bus0_handle == NULL)
    {
        i2c_master_bus_config_t i2c_mst0_config = {
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .i2c_port = I2C_MASTER_NUM,
            .sda_io_num = I2C_MASTER_SDA_IO,
            .scl_io_num = I2C_MASTER_SCL_IO,
            .glitch_ignore_cnt = 7,
            .flags.enable_internal_pullup = true,
        };
        ret = i2c_new_master_bus(&i2c_mst0_config, &g_i2c_bus0_handle);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to create I2C bus 0: %s", esp_err_to_name(ret));
            return ret;
        }
    }

    // --- Init I2C Bus 1 for OLED (if not already initialized) ---
    if (g_i2c_bus1_handle == NULL)
    {
        i2c_master_bus_config_t i2c_mst1_config = {
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .i2c_port = I2C_OLED_NUM,
            .sda_io_num = I2C_OLED_SDA_IO,
            .scl_io_num = I2C_OLED_SCL_IO,
            .glitch_ignore_cnt = 7,
            .flags.enable_internal_pullup = true,
        };
        ret = i2c_new_master_bus(&i2c_mst1_config, &g_i2c_bus1_handle);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to create I2C bus 1: %s", esp_err_to_name(ret));
            return ret;
        }
    }

    return ret;
}

/**
 * @brief De-initializes I2C buses and detaches all devices.
 *
 * This function safely detaches all known I2C devices from their buses
 * and then deletes the bus handles, freeing the resources.
 */
static void i2c_bus_deinit(i2c_bus_target_t bus_to_deinit)
{
    ESP_LOGI(TAG, "De-initializing I2C bus(es)...");

    if (bus_to_deinit == I2C_BUS_SENSORS || bus_to_deinit == I2C_BUS_ALL)
    {
        ESP_LOGD(TAG, "Detaching devices from bus 0...");
        // --- Detach devices from bus 0 ---
        if (g_rtc.i2c_dev_handle)
        {
            i2c_master_bus_rm_device(g_rtc.i2c_dev_handle);
            g_rtc.i2c_dev_handle = NULL;
        }
        if (g_bmp280.i2c_dev_handle)
        {
            i2c_master_bus_rm_device(g_bmp280.i2c_dev_handle);
            g_bmp280.i2c_dev_handle = NULL;
        }
        if (g_d6fph.i2c_dev_handle)
        {
            i2c_master_bus_rm_device(g_d6fph.i2c_dev_handle);
            g_d6fph.i2c_dev_handle = NULL;
        }
        // --- Delete bus 0 handle ---
        if (g_i2c_bus0_handle)
        {
            i2c_del_master_bus(g_i2c_bus0_handle);
            g_i2c_bus0_handle = NULL;
        }
    }

    if (bus_to_deinit == I2C_BUS_OLED || bus_to_deinit == I2C_BUS_ALL)
    {
        ESP_LOGD(TAG, "Detaching device from bus 1...");
        // --- Detach device from bus 1 ---
        i2c_oled_bus_deinit();
        // --- Delete bus 1 handle ---
        if (g_i2c_bus1_handle)
        {
            i2c_del_master_bus(g_i2c_bus1_handle);
            g_i2c_bus1_handle = NULL;
        }
    }
}

// --- Main Application Entry Point ---

void main_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Deep sleep configured. Inactivity timeout: %lums, Sleep duration: %llums", (unsigned long)g_cfg->inactivity_timeout_ms, g_cfg->sleep_duration_ms);

    while (1)
    {
        // --- Create a local, consistent snapshot of the shared buffer ---
        sensor_buffer_t local_buffer;
        if (xSemaphoreTake(g_sensor_buffer_mutex, pdMS_TO_TICKS(50)))
        {
            // Copy the global buffer to a local struct to work with consistent data.
            local_buffer = g_sensor_buffer;

            // --- Periodic Uptime Update ---
            // This is the best place to update uptime, as main_task is always running.
            // Calculate current session's awake time and add to total persistent uptime
            uint64_t current_session_awake_s = (esp_timer_get_time() - rtc_last_boot_time_ms * 1000) / 1000000ULL;
            g_sensor_buffer.uptime_seconds = rtc_total_awake_time_s + current_session_awake_s;

            xSemaphoreGive(g_sensor_buffer_mutex);
        }
        else
        {
            ESP_LOGE(TAG, "Failed to get sensor buffer snapshot. Loop will use stale data.");
        }

        // --- Process commands from other tasks ---
        app_command_t cmd;
        if (xQueueReceive(g_app_cmd_queue, &cmd, 0) == pdPASS)
        {
            // Any command from the UI means it's active
            switch (cmd.cmd)
            {
            case APP_CMD_SET_RTC_BUILD_TIME:
                ESP_LOGI(TAG, "CMD: Set RTC to build time");
                handle_set_rtc_to_build_time();
                break;
            case APP_CMD_GET_SD_FREE_SPACE:
                ESP_LOGI(TAG, "CMD: Get SD free space");
                spi_sdcard_get_free_space_mb();
                break;
            case APP_CMD_GET_SD_FILE_COUNT:
                ESP_LOGI(TAG, "CMD: Get SD file count");
                spi_sdcard_get_file_count();
                break;
            case APP_CMD_FORMAT_SD_CARD:
                ESP_LOGI(TAG, "CMD: Format SD card");
                // Pause the datalogger before formatting to prevent conflicts.
                ESP_LOGI(TAG, "Pausing datalogger for SD format...");
                xQueueSend(g_datalogger_cmd_queue, &(datalogger_cmd_msg_t){.cmd = DATALOGGER_CMD_PAUSE_WRITES}, 0);

                int wait_cycles = 0;
                const int max_wait_cycles = 150; // 3-second timeout
                while (g_datalogger_task_handle != NULL && eTaskGetState(g_datalogger_task_handle) != eSuspended && wait_cycles < max_wait_cycles)
                {
                    vTaskDelay(pdMS_TO_TICKS(20));
                    wait_cycles++;
                }

                if (wait_cycles >= max_wait_cycles)
                {
                    ESP_LOGE(TAG, "Timeout waiting for datalogger to pause for format. Aborting.");
                    if (xSemaphoreTake(g_command_status_mutex, portMAX_DELAY))
                    {
                        g_command_status = CMD_STATUS_FAIL;
                        xSemaphoreGive(g_command_status_mutex);
                    }
                }
                else
                {
                    ESP_LOGI(TAG, "Datalogger paused. Formatting SD card...");
                    spi_sdcard_format();
                }

                // Always resume the datalogger afterward.
                ESP_LOGI(TAG, "Resuming datalogger after format attempt.");
                if (g_datalogger_task_handle != NULL && eTaskGetState(g_datalogger_task_handle) == eSuspended)
                {
                    vTaskResume(g_datalogger_task_handle);
                }
                break;
            case APP_CMD_SYNC_RTC_NTP:
                ESP_LOGI(TAG, "CMD: Sync RTC with NTP");
                handle_sync_rtc_with_ntp();
                break;
            case APP_CMD_ACTIVITY_DETECTED:
                ESP_LOGD(TAG, "CMD: Activity Detected");
                if (g_uiRender_task_handle != NULL && eTaskGetState(g_uiRender_task_handle) == eSuspended)
                {
                    ESP_LOGI(TAG, "Activity detected, waking up UI.");
                }
                last_activity_ms = esp_timer_get_time() / 1000;
                if (g_uiRender_task_handle != NULL)
                {
                    // UI task exists, it's either running or suspended.
                    // oled_power_on() will handle resuming it if suspended.
                    oled_power_on();
                }
                else
                {
                    // UI task does not exist (e.g., after a timer wakeup).
                    // We need to create it for the first time.
                    ESP_LOGI(TAG, "UI task not running. Creating it now.");
                    gpio_set_level(OLED_POWER_PIN, 1); // Ensure OLED has power
                    vTaskDelay(pdMS_TO_TICKS(100));    // Wait for OLED to stabilize
                    i2c_oled_send_init_commands(I2C_OLED_NUM);
                    xTaskCreate(uiRender_task, "uiRender", 4096, NULL, 6, &g_uiRender_task_handle);
                }
                break;
            case APP_CMD_START_WEB_SERVER:
                ESP_LOGI(TAG, "CMD: Start web server");
                handle_start_web_server();
                break;
            case APP_CMD_STOP_WEB_SERVER:
                ESP_LOGI(TAG, "CMD: Stop web server");
                if (handle_stop_web_server())
                {
                    local_buffer.web_server_status = WEB_SERVER_STOPPED;
                }
                break;
            case APP_CMD_SET_DATALOGGER_MODE:
                ESP_LOGI(TAG, "CMD: Set Datalogger Mode to %d", cmd.mode);
                if (g_datalogger_cmd_queue != NULL)
                {
                    datalogger_cmd_msg_t logger_cmd = {
                        .cmd = DATALOGGER_CMD_SET_MODE,
                        .mode = cmd.mode};
                    xQueueSend(g_datalogger_cmd_queue, &logger_cmd, 0);

                    // Also force a file rotation on any mode change to ensure data is
                    // logged to a new file with the correct name (_hf or not).
                    datalogger_cmd_msg_t rotate_cmd = {.cmd = DATALOGGER_CMD_ROTATE_FILE};
                    xQueueSend(g_datalogger_cmd_queue, &rotate_cmd, 0);
                }
                break;
            case APP_CMD_LOG_COMPLETE_SLEEP_NOW:
                // checks here are partially redundant, I could just use the main loop to check all the "no sleep" conditions and update variables accordingly.
                // or I could just raise a flag, "sleep now requested" and have the main loop handle it. but since debugging sleep issues is a pain in the *ss,
                // I'll keep the detailed albeit redundant checks and logging here for now. :-P
                const char *sleep_reason_log = local_buffer.datalogger_paused ? "Logging paused" : "Log complete";
                ESP_LOGI(TAG, "CMD: %s, checking sleep conditions.", sleep_reason_log);

                bool web_server_active = (local_buffer.web_server_status == WEB_SERVER_RUNNING || local_buffer.web_server_status == WEB_SERVER_STARTING);
                uint64_t current_time_ms_for_sleep_check = esp_timer_get_time() / 1000ULL;
                // last_activity_ms== 0 means no command was received since boot
                bool ui_is_inactive = (current_time_ms_for_sleep_check - last_activity_ms > g_cfg->inactivity_timeout_ms || last_activity_ms == 0);

                if (ui_is_inactive && !web_server_active)
                {
                    ESP_LOGI(TAG, "UI is inactive and %s. Initiating sleep.", sleep_reason_log);
                    // in case it's already off, this does no harm
                    oled_power_off();

                    // --- Step 1: Tell the logger to pause---
                    xQueueSend(g_datalogger_cmd_queue, &(datalogger_cmd_msg_t){.cmd = DATALOGGER_CMD_PAUSE_WRITES}, 0);

                    // --- Step 2: Wait for the datalogger to suspend itself ---
                    ESP_LOGI(TAG, "Waiting for datalogger to pause...");
                    int wait_cycles = 0;
                    const int max_wait_cycles = 150; // 3 seconds timeout
                    while (g_datalogger_task_handle != NULL && eTaskGetState(g_datalogger_task_handle) != eSuspended && wait_cycles < max_wait_cycles)
                    {
                        vTaskDelay(pdMS_TO_TICKS(20));
                        wait_cycles++;
                    }

                    if (wait_cycles >= max_wait_cycles)
                    {
                        ESP_LOGE(TAG, "Timeout waiting for datalogger to pause. Aborting sleep.");
                    }
                    else
                    {
                        ESP_LOGI(TAG, "Datalogger paused. De-initializing peripherals and sleeping.");
                        spi_sdcard_deinit();
                        wifi_manager_deinit();
                        gpio_set_level(DEVICES_POWER_PIN, 0);
                        go_to_deep_sleep();
                    }
                }
                else
                {
                    // Detailed logging for why sleep was skipped.
                    if (!ui_is_inactive)
                    {
                        uint64_t current_time_ms_for_log = esp_timer_get_time() / 1000ULL;
                        uint64_t inactivity_duration = current_time_ms_for_log - last_activity_ms;
                        ESP_LOGW(TAG, "Sleep skipped: UI is active. Inactivity timer: %llu ms (timeout is %lu ms)",
                                 inactivity_duration, (unsigned long)g_cfg->inactivity_timeout_ms);
                    }
                    if (web_server_active)
                    {
                        ESP_LOGW(TAG, "Sleep skipped: Web server is active.");
                    }
                }
                break;
            default:
                ESP_LOGW(TAG, "Unknown command received: %d", cmd.cmd);
                break;
            }
        }

        handle_web_server_fsm();

        bool web_server_active = (local_buffer.web_server_status == WEB_SERVER_RUNNING || local_buffer.web_server_status == WEB_SERVER_STARTING);

        // Handle Wi-Fi disconnect when web server should be active
        if (web_server_active && !wifi_manager_is_connected())
        {
            ESP_LOGW(TAG, "Web server is active but Wi-Fi disconnected. Attempting to reconnect...");
            // Update UI status
            if (xSemaphoreTake(g_sensor_buffer_mutex, portMAX_DELAY))
            {
                g_sensor_buffer.web_server_status = WEB_SERVER_STARTING;
                xSemaphoreGive(g_sensor_buffer_mutex);
            }
            wifi_manager_connect(g_cfg->wifi_ssid, g_cfg->wifi_password);
        }

     

        // --- Main Sleep/Wake Logic ---
        // --- Power Saving & UI Inactivity Logic ---
        uint64_t current_time_ms = esp_timer_get_time() / 1000ULL;

        // IMPORTANT, this also affect s the handler of SLEEP NOW message
        // If USB is mounted by a host, treat it as continuous activity to prevent deep sleep.
        // This is a critical safeguard.
        if (spi_sdcard_is_usb_connected())
        {
            // Keep resetting the activity timer to prevent the device from entering deep sleep.
            last_activity_ms = current_time_ms;
            if (xSemaphoreTake(g_sensor_buffer_mutex, portMAX_DELAY))
            {
                g_sensor_buffer.usb_msc_connected = true;   
                xSemaphoreGive(g_sensor_buffer_mutex);
            }

        }
        else
        {
            if (xSemaphoreTake(g_sensor_buffer_mutex, portMAX_DELAY))
            {
                g_sensor_buffer.usb_msc_connected = false;   
                xSemaphoreGive(g_sensor_buffer_mutex);
            }
        }

        // The UI inactivity check for turning off the OLED should be separate.
        bool ui_is_inactive = (current_time_ms - last_activity_ms > g_cfg->inactivity_timeout_ms) && (last_activity_ms != 0);

        // --- USB Connection Logic ---
        // This block handles the transition between SD card only mode and USB Mass Storage mode.
        bool is_externally_powered = battery_is_externally_powered();

        if (is_externally_powered && !is_usb_connected_state)
        {
            // USB was just connected (or was connected at boot).
            // USB has absolute priority. If the web server is running, stop it.
            if (local_buffer.web_server_status == WEB_SERVER_RUNNING || local_buffer.web_server_status == WEB_SERVER_STARTING)
            {
                ESP_LOGI(TAG, "USB connected, stopping web server to grant exclusive access.");
                handle_stop_web_server();
            }

            // Switch to full USB MSC mode.
            ESP_LOGI(TAG, "USB power detected. Initializing for MSC.");
            // De-init SD card first to release resources for TinyUSB to use.
            spi_sdcard_deinit();
            // Now, do a full init which includes TinyUSB.
            spi_sdcard_full_init();
            is_usb_connected_state = true;
            handle_battery_refresh();
        }
        else if (!is_externally_powered && is_usb_connected_state)
        {
            // USB was just disconnected.
            ESP_LOGI(TAG, "USB disconnected. Re-initializing SD card for logging.");
            // De-init everything to be safe.
            spi_sdcard_deinit();
            // Re-init in SD-only mode for the datalogger.
            spi_sdcard_init_sd_only();
            is_usb_connected_state = false;
            handle_battery_refresh();
        }

        // --- UI Inactivity Logic ---
   
        // If UI is inactive but the screen is still on, turn it off.
        if (g_uiRender_task_handle != NULL && ui_is_inactive && !web_server_active)
        {
            // UI timeout occurred, but no new write yet. Just turn off the screen.
            oled_power_off();
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void log_wakeup_reason(esp_sleep_wakeup_cause_t cause)
{
    switch (cause)
    {
    case ESP_SLEEP_WAKEUP_UNDEFINED:
        ESP_LOGI(TAG, "Wakeup reason: Reset or power-on");
        break;
    case ESP_SLEEP_WAKEUP_TIMER:
        ESP_LOGI(TAG, "Wakeup reason: Timer");
        break;
    case ESP_SLEEP_WAKEUP_GPIO:
        ESP_LOGI(TAG, "Wakeup reason: GPIO");
        break;
    case ESP_SLEEP_WAKEUP_EXT0:
    case ESP_SLEEP_WAKEUP_EXT1:
        ESP_LOGI(TAG, "Wakeup reason: External GPIO");
        break;
    case ESP_SLEEP_WAKEUP_UART:
        ESP_LOGI(TAG, "Wakeup reason: UART");
        break;
    default:
        ESP_LOGI(TAG, "Wakeup reason: Other (%d)", cause);
        break;
    }
}

void app_main(void)
{
    // Get wakeup cause
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    log_wakeup_reason(cause);

    // Disable all wakeup sources after waking up to prevent immediate re-triggering.
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);

    // Create the initialization event group
    g_init_event_group = xEventGroupCreate();

    // --- Step 1: Initialize GPIO for power control ---
    gpio_config_t oled_pwr_pin_cfg = {
        .pin_bit_mask = (1ULL << OLED_POWER_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&oled_pwr_pin_cfg);

    gpio_config_t pwr_pin_cfg = {
        .pin_bit_mask = (1ULL << DEVICES_POWER_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&pwr_pin_cfg);

    // Turn on power for all peripherals.
    gpio_set_level(DEVICES_POWER_PIN, 1);

    // Initialize I2C buses first, as they are needed by peripherals.
    esp_err_t err = i2c_bus_init();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C master initialization failed, stopping.");
        return;
    }

    // --- Early UI Initialization to show boot message ---
    if (cause != ESP_SLEEP_WAKEUP_TIMER)
    {
        // Power on OLED
        gpio_set_level(OLED_POWER_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(50)); // Wait for OLED power to stabilize
    }
    else
    {
        // Ensure OLED is off for timer-based wakeups
        gpio_set_level(OLED_POWER_PIN, 0);
    }
    vTaskDelay(pdMS_TO_TICKS(50)); // Wait for power to stabilize

    // Initialize UI renderer which will handle its own I2C bus and OLED.
    uiRender_init(g_i2c_bus1_handle, OLED_I2C_ADDR);

    // The uiRender_init function already sends the init commands.
    // Now we clear the buffer, write our message, and update the screen.
    i2c_oled_clear(I2C_OLED_NUM);
    char display_str[21];
    const char *msg = (cause == ESP_SLEEP_WAKEUP_EXT0 || cause == ESP_SLEEP_WAKEUP_GPIO) ? "Waking up..." : "Booting...";
    snprintf(display_str, sizeof(display_str), "%-20s", msg);
    i2c_oled_write_inverted_text(I2C_OLED_NUM, 0, 0, "System");
    i2c_oled_write_text(I2C_OLED_NUM, 2, 0, display_str);
    i2c_oled_update_screen(I2C_OLED_NUM); // This is the crucial missing step

    // --- Step 2: Initialize RTOS objects (Mutexes and Queues) ---
    g_command_status_mutex = xSemaphoreCreateMutex();
    g_i2c_bus_mutex = xSemaphoreCreateMutex();
    g_sensor_buffer_mutex = xSemaphoreCreateMutex();
    g_app_cmd_queue = xQueueCreate(10, sizeof(app_command_t));
    g_datalogger_cmd_queue = xQueueCreate(5, sizeof(datalogger_cmd_msg_t));

    // --- Step 2.5: Restore State from RTC Memory ---
    if (cause == ESP_SLEEP_WAKEUP_UNDEFINED)
    {
        // This is a cold boot, not a wakeup from sleep. Initialize RTC variables.
        ESP_LOGI(TAG, "First boot, initializing RTC state.");
        rtc_last_write_status = WRITE_STATUS_UNKNOWN;
        rtc_last_successful_write_ts = 0;
        rtc_datalogger_mode = DATALOGGER_MODE_NORMAL; // Default to normal on cold boot
        rtc_total_awake_time_s = 0;
    }
    rtc_last_boot_time_ms = esp_timer_get_time() / 1000;

    // Restore the persisted state into the global buffer.
    if (xSemaphoreTake(g_sensor_buffer_mutex, portMAX_DELAY))
    {
        g_sensor_buffer.writeStatus = rtc_last_write_status;
        g_sensor_buffer.last_successful_write_ts = rtc_last_successful_write_ts;
        g_sensor_buffer.high_freq_mode_enabled = (rtc_datalogger_mode == DATALOGGER_MODE_HF);
        g_sensor_buffer.datalogger_paused = (rtc_datalogger_mode == DATALOGGER_MODE_PAUSED);
        // Uptime is calculated dynamically in main_task
        xSemaphoreGive(g_sensor_buffer_mutex);
    }

    // --- Step 3: Initialize SD card and Configuration ---
    // Initialize SD card without USB first. The main_task will handle
    // full USB initialization if external power is detected.
    spi_sdcard_init_sd_only();
    is_usb_connected_state = battery_is_externally_powered();

    // Initialize NVS first, as it's required by other components (like Wi-Fi)
    config_init();

    // Initialize Wi-Fi stack
    wifi_manager_init();

    const char *config_path = "/sdcard/config.ini";
    const char *new_config_path = "/sdcard/config_LOADED.ini";

    // Check if config.ini exists.
    struct stat st;
    if (stat(config_path, &st) == 0)
    {
        // File exists, so load it to flash.
        ESP_LOGI(TAG, "Found %s, loading to flash...", config_path);
        if (config_load_from_sdcard_to_flash(config_path) == ESP_OK)
        {
            ESP_LOGI(TAG, "Config loaded successfully. Renaming to %s", new_config_path);
            // Before renaming, check if the destination file exists and delete it.
            struct stat st_dest;
            if (stat(new_config_path, &st_dest) == 0)
            {
                ESP_LOGI(TAG, "Deleting old %s before renaming.", new_config_path);
                if (unlink(new_config_path) != 0)
                {
                    ESP_LOGE(TAG, "Failed to delete %s: %s", new_config_path, strerror(errno));
                }
            }
            // Rename the file to prevent it from being loaded again on next boot.
            if (rename(config_path, new_config_path) != 0)
            {
                ESP_LOGE(TAG, "Failed to rename config file: %s", strerror(errno));
            }
        }
        else
        {
            ESP_LOGE(TAG, "Failed to load config from SD card into flash.");
        }
    }
    else
    {
        // File not found, which is a normal case.
        // The application will use the configuration already stored in flash.
        ESP_LOGI(TAG, "%s not found. Using stored or default values.", config_path);
    }
    // --- Step 3.5: Read configuration from flash and apply it ---
    config_params_init();
    g_cfg = config_params_get();

    // --- Step 4: Initialize hardware drivers and peripherals ---
    // Initialize Battery Reader first, as it uses ADC which can conflict if I2C is initialized first
    battery_reader_init(BATTERY_ADC_PIN, BATTERY_PWR_PIN, g_cfg->battery_voltage_divider_ratio);

    // --- Step 5: Initialize and validate the external Real-Time Clock (RTC) ---
    bool local_ds3231_available = false;
    err = ds3231_init(&g_rtc, g_i2c_bus0_handle, DS3231_I2C_ADDR);
    struct tm timeinfo;
    time_t rtc_ts = -1;

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "DS3231 RTC initialization failed with error: %s", esp_err_to_name(err));
        i2c_bus_deinit(I2C_BUS_SENSORS);
        i2c_bus_init();
        g_rtc.i2c_dev_handle = NULL;
        local_ds3231_available = false;
    }
    else
    {
        local_ds3231_available = true;
        ESP_LOGI(TAG, "DS3231 RTC initialized successfully.");

        // On a cold boot (not a wakeup from deep sleep), apply the default configuration.
        if (cause == ESP_SLEEP_WAKEUP_UNDEFINED)
        {
            ESP_LOGI(TAG, "Cold boot detected. Applying DS3231 default configuration.");
            if (xSemaphoreTake(g_i2c_bus_mutex, portMAX_DELAY))
            {
                ds3231_set_default_config(&g_rtc);
                xSemaphoreGive(g_i2c_bus_mutex);
            }
        }

        // Attempt to read the time from the RTC.
        if (xSemaphoreTake(g_i2c_bus_mutex, portMAX_DELAY))
        {
            if (ds3231_get_time(&g_rtc, &timeinfo) == ESP_OK)
            {
                // Since system TZ is UTC, mktime works like timegm.
                rtc_ts = mktime(&timeinfo);
            }
            xSemaphoreGive(g_i2c_bus_mutex);
        }

        // If RTC read was successful, check if the time is valid.
        if (rtc_ts != -1)
        {
            struct tm build_time;
            sscanf(__DATE__, "%*s %*d %d", &build_time.tm_year);
            build_time.tm_year -= 1900;

            // If RTC year is before build year, set the time
            if (timeinfo.tm_year < build_time.tm_year)
            {
                ESP_LOGW(TAG, "RTC time is invalid. Setting to build time.");
                handle_set_rtc_to_build_time();
            }

            local_ds3231_available = true;
            ESP_LOGI(TAG, "RTC available, using DS3231 for timestamps.");
            // Set system timezone to UTC and synchronize system time with RTC.
            setenv("TZ", "UTC0", 1);
            tzset();
            struct timeval tv = {.tv_sec = rtc_ts};
            settimeofday(&tv, NULL);
            ESP_LOGI(TAG, "System time synchronized from DS3231 RTC to UTC.");
        }

        if (xSemaphoreTake(g_sensor_buffer_mutex, portMAX_DELAY))
        {
            g_sensor_buffer.ds3231_available = local_ds3231_available;
            xSemaphoreGive(g_sensor_buffer_mutex);
        }
        else
        {
            local_ds3231_available = false;
            ESP_LOGW(TAG, "RTC not available, using ESP32 system time.");
        }
    }
    bool local_d6fph_available = false;
    // Initialize D6F-PH Sensor.
    err = d6fph_init(&g_d6fph, g_i2c_bus0_handle, D6FPH_I2C_ADDR, g_cfg->d6fph_model);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "D6F-PH sensor initialization failed with error: %s", esp_err_to_name(err));
        i2c_bus_deinit(I2C_BUS_SENSORS); // Tear down the failed bus
        i2c_bus_init();                  // Re-create it
        g_d6fph.i2c_dev_handle = NULL;
        local_d6fph_available = false;
        if (local_ds3231_available)
        {
            ESP_LOGI(TAG, "D6F-PH sensor not available, but RTC is available. Using DS3231 for timestamps.");
            ds3231_init(&g_rtc, g_i2c_bus0_handle, DS3231_I2C_ADDR); // Re-attach RTC to the new bus
        }
    }
    else
    {
        local_d6fph_available = true;
        ESP_LOGI(TAG, "D6F-PH sensor initialized successfully.");

        if (xSemaphoreTake(g_sensor_buffer_mutex, portMAX_DELAY))
        {
            g_sensor_buffer.d6fph_available = local_d6fph_available;
            xSemaphoreGive(g_sensor_buffer_mutex);
        }
    }

    bool local_bmp280_available = false;
    // Initialize BMP280 Sensor.
    err = bmp280_init(&g_bmp280, g_i2c_bus0_handle, BMP280_I2C_ADDR);

    if (err != ESP_OK)
    {
        local_bmp280_available = false;
        ESP_LOGE(TAG, "BMP280 sensor initialization failed with error: %s", esp_err_to_name(err));
        i2c_bus_deinit(I2C_BUS_SENSORS); // Tear down the failed bus
        i2c_bus_init();                  // Re-create it
        g_bmp280.i2c_dev_handle = NULL;
        if (local_ds3231_available)
        {
            ESP_LOGI(TAG, "BMP280 sensor not available, but RTC is available. Using DS3231 for timestamps.");
            ds3231_init(&g_rtc, g_i2c_bus0_handle, DS3231_I2C_ADDR); // Re-attach RTC
        }
        if (local_d6fph_available)
        {
            ESP_LOGI(TAG, "BMP280 sensor not available, but D6F-PH is available. Using D6F-PH for pressure data.");
            d6fph_init(&g_d6fph, g_i2c_bus0_handle, D6FPH_I2C_ADDR, g_cfg->d6fph_model); // Re-attach D6FPH
        }
    }
    else
    {
        local_bmp280_available = true;
        ESP_LOGI(TAG, "BMP280 sensor initialized successfully.");
        if (xSemaphoreTake(g_sensor_buffer_mutex, portMAX_DELAY))
        {
            g_sensor_buffer.bmp280_available = local_bmp280_available;
            xSemaphoreGive(g_sensor_buffer_mutex);
        }
    }

    // --- Step 6: Initialize user input (Rotary Encoder) ---
    rotaryencoder_config_t encoder_cfg = {
        .pin_a = ROTARY_ENCODER_PIN_A,
        .pin_b = ROTARY_ENCODER_PIN_B,
        .button_pin = ROTARY_ENCODER_BUTTON_PIN,
        .button_debounce_ms = (uint32_t)BUTTON_DEBOUNCE_TIME_MS,
        .on_rotate_cw = on_encoder_rotate_cw,
        .on_rotate_ccw = on_encoder_rotate_ccw,
        .on_button_press = on_encoder_button_press,
        .on_button_long_press = on_encoder_button_long_press,
    };
    rotaryencoder_init(&encoder_cfg);

    // --- Step 7: Initialize UI (if needed) ---
    if (cause != ESP_SLEEP_WAKEUP_TIMER)
    {
        // UI is already initialized and showing a boot message.
    }
    else
    {
        ESP_LOGI(TAG, "Timer wakeup, skipping UI initialization.");
    }

    // --- Step 8: Start all application tasks ---
    ESP_LOGI(TAG, "Starting  sensor readings and Encoder monitoring...");
    rotaryencoder_start_task();

    // Prepare parameters for the datalogger task
    // Allocate parameters on the heap so they persist after app_main exits.
    datalogger_task_params_t *datalogger_params = malloc(sizeof(datalogger_task_params_t));
    if (datalogger_params == NULL)
    {
        ESP_LOGE(TAG, "Failed to allocate memory for datalogger params!");
        return; // Or handle error appropriately
    }
    datalogger_params->bmp280_dev = &g_bmp280;
    datalogger_params->d6fph_dev = &g_d6fph;
    datalogger_params->bmp280_available = local_bmp280_available;
    datalogger_params->d6fph_available = local_d6fph_available;
    datalogger_params->log_interval_ms = g_cfg->log_interval_ms;
    datalogger_params->hf_log_interval_ms = g_cfg->hf_log_interval_ms;

    if (cause != ESP_SLEEP_WAKEUP_TIMER)
    {
        xTaskCreate(uiRender_task, "uiRender", 4096, NULL, 6, &g_uiRender_task_handle);
    }
    else
    {
        g_uiRender_task_handle = NULL; // Ensure handle is null if task is not created
    }
    xTaskCreate(datalogger_task, "datalogger", 4096, datalogger_params, 5, &g_datalogger_task_handle); // Pass pointer to heap-allocated struct
    xTaskCreate(main_task, "main_task", 4096, NULL, 5, NULL);                                          // Main command processing task at priority 5

    // Signal that all initialization is done
    xEventGroupSetBits(g_init_event_group, INIT_DONE_BIT);
}
