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
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_rom_sys.h"
#include "esp_timer.h" // Required for esp_timer_get_time()
#include <time.h>
#include <sys/time.h>
#include "esp_sleep.h"

#include "../lib/i2c-bmp280/i2c-bmp280.h" // BMP280 sensor API
#include "../lib/rotaryencoder/rotaryencoder.h"
#include "../lib/spi-sdcard/spi-sdcard.h"
#include "../lib/i2c-oled/i2c-oled.h"
#include "ui_render.h"
#include "../lib/buffers.h"
#include "i2c-ds3231.h"
#include "datalogger_task.h"
#include "../lib/lipo-battery/lipo-battery.h"

// DO NOT use pins 19 and 20, they are used by the flash memory
// DO NOT use pins 5 and 6, they are used for LiPo battery measurements

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
#define ROTARY_ENCODER_BUTTON_PIN GPIO_NUM_7
#define BUTTON_DEBOUNCE_TIME_MS 50
#define INACTIVITY_TIMEOUT_MS 10000          // 10 seconds
#define SLEEP_DURATION_US (45 * 1000 * 1000) // 45 seconds

// --- Battery ADC Configuration ---
#define BATTERY_PWR_PIN GPIO_NUM_5
#define BATTERY_ADC_PIN GPIO_NUM_6
#define BATTERY_VOLTAGE_DIVIDER_RATIO 4.1428f // (470k + 150k) / 150k, then tuned experimentally

static const char *TAG = "main";

bool rtc_available = false;

ds3231_t g_rtc;    // Global RTC device handle
bmp280_t g_bmp280; // Global BMP280 device handle

TaskHandle_t g_uiRender_task_handle = NULL;

static uint64_t last_activity_ms = 0;

static void set_rtc_to_build_time(void)
{
    if (!rtc_available)
    {
        ESP_LOGE(TAG, "Cannot set time, RTC not available.");
    }

    // Set the timezone to correctly interpret the local build time
    setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 1);
    tzset();

    struct tm build_time;
    const char *date_str = __DATE__; // "Mmm dd yyyy"
    const char *time_str = __TIME__; // "hh:mm:ss"

    // Parse date and time
    sscanf(time_str, "%d:%d:%d", &build_time.tm_hour, &build_time.tm_min, &build_time.tm_sec);

    char month_str[4];
    int year;
    sscanf(date_str, "%s %d %d", month_str, &build_time.tm_mday, &year);
    build_time.tm_year = year - 1900;

    const char *months[] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
    for (int i = 0; i < 12; i++)
    {
        if (strcmp(month_str, months[i]) == 0)
        {
            build_time.tm_mon = i;
            break;
        }
    }

    // Convert local build time to a UTC timestamp, then back to a UTC-based struct tm
    time_t build_ts = mktime(&build_time);
    struct tm utc_build_time;
    gmtime_r(&build_ts, &utc_build_time);

    int ret = -1;
    if (xSemaphoreTake(g_i2c_bus_mutex, portMAX_DELAY))
    {
        ret = ds3231_set_time(&g_rtc, &utc_build_time);
        xSemaphoreGive(g_i2c_bus_mutex);
    }

    if (ret == 0)
    {
        ESP_LOGI(TAG, "RTC time set to build time: %s %s", date_str, time_str);
        // Also update system time immediately
        struct timeval tv = {.tv_sec = build_ts};
        settimeofday(&tv, NULL);
    }
    else
    {
        ESP_LOGE(TAG, "Failed to set RTC time.");
    }

    // Restore the system timezone to UTC
    setenv("TZ", "UTC0", 1);
    tzset();

    // Update the global command status
    if (xSemaphoreTake(g_command_status_mutex, portMAX_DELAY))
    {
        g_command_status = (ret == 0) ? CMD_STATUS_SUCCESS : CMD_STATUS_FAIL;
        xSemaphoreGive(g_command_status_mutex);
    }
}

static void peripherals_deinit(void)
{
    spi_sdcard_deinit();
    ESP_LOGI(TAG, "Peripherals de-initialized.");
}

static void resync_time_from_rtc(void)
{
    if (!rtc_available)
    {
        return;
    }

    struct tm timeinfo;
    time_t rtc_ts = -1;

    if (xSemaphoreTake(g_i2c_bus_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        if (ds3231_get_time(&g_rtc, &timeinfo) == 0)
        {
            // The RTC provides a struct tm in UTC.
            // Since system TZ is UTC, mktime works like timegm.
            rtc_ts = mktime(&timeinfo);
        }
        xSemaphoreGive(g_i2c_bus_mutex);
    }

    if (rtc_ts != -1)
    {
        struct timeval tv = {.tv_sec = rtc_ts};
        settimeofday(&tv, NULL);
        ESP_LOGI(TAG, "System time re-synced from DS3231 RTC.");
    }
}
/**
 * @brief i2c master initialization
 */

static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
}

// --- Global buffer and mutex definition ---
sensor_buffer_t g_sensor_buffer = {0};
SemaphoreHandle_t g_sensor_buffer_mutex = NULL;
QueueHandle_t g_app_cmd_queue = NULL;
SemaphoreHandle_t g_command_status_mutex = NULL;
SemaphoreHandle_t g_i2c_bus_mutex = NULL;
volatile command_status_t g_command_status = CMD_STATUS_IDLE;
QueueHandle_t g_datalogger_cmd_queue = NULL;

// --- Main Application Entry Point ---

void main_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Light sleep configured. Inactivity timeout: %dms, Sleep duration: %llus", INACTIVITY_TIMEOUT_MS, SLEEP_DURATION_US / 1000000ULL);
    time_t last_processed_write_ts = 0;

    while (1)
    {
        // --- Process commands from other tasks ---
        app_command_t cmd;
        if (xQueueReceive(g_app_cmd_queue, &cmd, 0) == pdPASS)
        {
            // Any command from the UI means it's active
            if (eTaskGetState(g_uiRender_task_handle) == eSuspended)
            {
                ESP_LOGI(TAG, "Activity detected, waking up UI.");
                // This will fall through to the activity detected command
            }
            switch (cmd)
            {
            case APP_CMD_SET_RTC_BUILD_TIME:
                ESP_LOGI(TAG, "Received command to set RTC to build time.");
                set_rtc_to_build_time();
                break;
            case APP_CMD_GET_SD_FREE_SPACE:
                ESP_LOGI(TAG, "Received command to get SD card free space.");
                spi_sdcard_get_free_space_mb();
                break;
            case APP_CMD_GET_SD_FILE_COUNT:
                ESP_LOGI(TAG, "Received command to get SD card file count.");
                spi_sdcard_get_file_count();
                break;
            case APP_CMD_FORMAT_SD_CARD:
                ESP_LOGI(TAG, "Received command to format SD card.");
                spi_sdcard_format();
                // Status is now handled inside spi_sdcard_format
                break;
            case APP_CMD_ACTIVITY_DETECTED:
                last_activity_ms = esp_timer_get_time() / 1000;
                if (eTaskGetState(g_uiRender_task_handle) == eSuspended)
                {
                    ESP_LOGI(TAG, "Waking up UI from suspended state.");
                    // Power on OLED and re-initialize
                    gpio_set_level(OLED_POWER_PIN, 1);
                    vTaskDelay(pdMS_TO_TICKS(50)); // Wait for OLED to stabilize
                    uiRender_init(I2C_OLED_NUM, I2C_OLED_SDA_IO, I2C_OLED_SCL_IO);

                    // Resume the UI task and send a wake-up event
                    vTaskResume(g_uiRender_task_handle);
                    uiRender_send_event(UI_EVENT_WAKE_UP, NULL, 0);
                }
                break;
            case APP_CMD_REFRESH_SENSOR_DATA:
                // ESP_LOGI(TAG, "Received command to refresh sensor data, forwarding to datalogger.");
                if (g_datalogger_cmd_queue != NULL)
                {
                    datalogger_command_t logger_cmd = DATALOGGER_CMD_FORCE_REFRESH;
                    xQueueSend(g_datalogger_cmd_queue, &logger_cmd, 0);
                }
                break;
            default:
                ESP_LOGW(TAG, "Unknown command received: %d", cmd);
                break;
            }
        }

        // --- Power Saving & UI Inactivity Logic ---
        // If USB is connected and mounted by the host, treat it as continuous activity to prevent sleep.
        if (spi_sdcard_is_usb_connected())
        {
            last_activity_ms = esp_timer_get_time() / 1000ULL;
        }

        uint64_t current_time_ms = esp_timer_get_time() / 1000ULL;
        bool ui_is_inactive = (current_time_ms - last_activity_ms > INACTIVITY_TIMEOUT_MS);

        // --- New Sleep Logic ---
        time_t current_write_ts = 0;
        if (xSemaphoreTake(g_sensor_buffer_mutex, pdMS_TO_TICKS(50)))
        {
            current_write_ts = g_sensor_buffer.last_successful_write_ts;
            xSemaphoreGive(g_sensor_buffer_mutex);
        }

        bool new_write_occurred = (current_write_ts > 0 && current_write_ts > last_processed_write_ts);

        if (ui_is_inactive && new_write_occurred)
        {
            ESP_LOGI(TAG, "UI inactive and new data logged. Initiating sleep.");
            last_processed_write_ts = current_write_ts; // Mark this write as processed

            // --- Prepare for sleep ---
            // 1. Suspend UI task and power down OLED
            uiRender_send_event(UI_EVENT_PREPARE_SLEEP, NULL, 0);
            vTaskDelay(pdMS_TO_TICKS(300)); // Give UI time to draw sleep message
            vTaskSuspend(g_uiRender_task_handle);
            gpio_set_level(OLED_POWER_PIN, 0);

            // 2. Tell the datalogger to pause writes
            ESP_LOGI(TAG, "Requesting datalogger to pause...");
            datalogger_command_t logger_cmd = DATALOGGER_CMD_PAUSE_WRITES;
            xQueueSend(g_datalogger_cmd_queue, &logger_cmd, 0);

            // 3. Wait for the datalogger to confirm it's paused
            int datalogger_status = 0;
            int wait_cycles = 0;
            do
            {
                vTaskDelay(pdMS_TO_TICKS(50));
                if (xSemaphoreTake(g_sensor_buffer_mutex, pdMS_TO_TICKS(50)))
                {
                    datalogger_status = g_sensor_buffer.datalogger_status;
                    xSemaphoreGive(g_sensor_buffer_mutex);
                }
                wait_cycles++;
            } while (datalogger_status != DATA_LOGGER_PAUSED && wait_cycles < 60); // Max 3 sec wait

            if (datalogger_status != DATA_LOGGER_PAUSED)
            {
                ESP_LOGE(TAG, "Timeout waiting for datalogger to pause. Aborting sleep.");
                // Reset activity timer and try again later
                last_activity_ms = esp_timer_get_time() / 1000;
                // Tell UI to wake up again
                uiRender_send_event(UI_EVENT_WAKE_UP, NULL, 0);
                continue;
            }

            ESP_LOGI(TAG, "Datalogger paused. Preparing to sleep.");

            ESP_LOGI(TAG, "De-initializing peripherals before sleep.");
            peripherals_deinit();

            ESP_LOGI(TAG, "Powering down external devices.");
            gpio_set_level(DEVICES_POWER_PIN, 0);

            ESP_LOGI(TAG, "Entering light sleep.");

            // Disable normal ISRs to prepare for sleep
            gpio_isr_handler_remove(ROTARY_ENCODER_PIN_A);
            gpio_isr_handler_remove(ROTARY_ENCODER_PIN_B);
            gpio_isr_handler_remove(ROTARY_ENCODER_BUTTON_PIN);

            // Configure and enable GPIO wakeup sources
            rotaryencoder_enable_wakeup_source();
            esp_sleep_enable_gpio_wakeup();
            esp_sleep_enable_timer_wakeup(SLEEP_DURATION_US);
            esp_light_sleep_start();

            // --- WAKE UP ---
            gpio_set_level(DEVICES_POWER_PIN, 1);
            vTaskDelay(pdMS_TO_TICKS(100));
            ESP_LOGI(TAG, "Powered up external devices. Re-initializing...");

            esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
            if (cause == ESP_SLEEP_WAKEUP_TIMER)
            {
                ESP_LOGI(TAG, "Woke up from light sleep due to timer.");
                // Keep OLED off and UI task suspended.
                // Only re-init SD card for datalogger.
                spi_sdcard_full_init();
            }
            else if (cause == ESP_SLEEP_WAKEUP_GPIO)
            {
                ESP_LOGI(TAG, "Woke up from light sleep due to GPIO.");
                // Power on OLED, re-init, and resume UI task
                gpio_set_level(OLED_POWER_PIN, 1);
                vTaskDelay(pdMS_TO_TICKS(50)); // Wait for OLED to stabilize
                uiRender_init(I2C_OLED_NUM, I2C_OLED_SDA_IO, I2C_OLED_SCL_IO);
                spi_sdcard_full_init(); // Also re-init SD card
                vTaskResume(g_uiRender_task_handle);
                uiRender_send_event(UI_EVENT_WAKE_UP, NULL, 0);

                // Debounce after GPIO wakeup to prevent the wake-up press from being processed as a command.
                vTaskDelay(pdMS_TO_TICKS(100));
                xQueueReset(rotaryencoder_get_queue_handle());
            }
            else
            {
                ESP_LOGI(TAG, "Woke up from light sleep due to other reason (%d)", cause);
            }

            // Tell the datalogger to resume writes
            logger_cmd = DATALOGGER_CMD_RESUME_WRITES;
            xQueueSend(g_datalogger_cmd_queue, &logger_cmd, 0);

            // Re-install normal ISRs for active operation
            gpio_isr_handler_add(ROTARY_ENCODER_PIN_A, gpio_isr_handler, (void *)ROTARY_ENCODER_PIN_A);
            gpio_isr_handler_add(ROTARY_ENCODER_PIN_B, gpio_isr_handler, (void *)ROTARY_ENCODER_PIN_B);
            gpio_isr_handler_add(ROTARY_ENCODER_BUTTON_PIN, gpio_isr_handler, (void *)ROTARY_ENCODER_BUTTON_PIN);

            gpio_wakeup_disable(ROTARY_ENCODER_BUTTON_PIN); // Disable wakeup to use normal ISR

            resync_time_from_rtc();                         // Re-sync system time with external RTC
            last_activity_ms = esp_timer_get_time() / 1000ULL; // Reset activity timer on wakeup
        }
        else if (ui_is_inactive && eTaskGetState(g_uiRender_task_handle) != eSuspended)
        {
            // UI timeout occurred, but no new write yet. Just turn off the screen.
            ESP_LOGI(TAG, "UI Inactivity timeout. Suspending UI task and turning off OLED.");
            vTaskSuspend(g_uiRender_task_handle);
            gpio_set_level(OLED_POWER_PIN, 0);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main(void) {
    // --- Initialize Devices Power Pin ---
    gpio_config_t oled_pwr_pin_cfg = {
        .pin_bit_mask = (1ULL << OLED_POWER_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&oled_pwr_pin_cfg);
   

    gpio_config_t pwr_pin_cfg ={
        .pin_bit_mask = (1ULL << DEVICES_POWER_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&pwr_pin_cfg);

    gpio_set_level(DEVICES_POWER_PIN, 1);
    gpio_set_level(OLED_POWER_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(50)); // Wait 50ms after powering up devices

    g_command_status_mutex = xSemaphoreCreateMutex();
    g_i2c_bus_mutex = xSemaphoreCreateMutex();

    g_sensor_buffer_mutex = xSemaphoreCreateMutex();
    g_app_cmd_queue = xQueueCreate(10, sizeof(app_command_t));
    g_datalogger_cmd_queue = xQueueCreate(5, sizeof(datalogger_command_t));

    // Initialize Battery Reader first, as it uses ADC which can conflict if I2C is initialized first
    battery_reader_init(BATTERY_ADC_PIN, BATTERY_PWR_PIN, BATTERY_VOLTAGE_DIVIDER_RATIO);

    // Initialize I2C
    esp_err_t err = i2c_master_init();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C master initialization failed, stopping.");
        return;
    }

    // Initialize BMP180 Sensor
    err = bmp280_init(&g_bmp280, I2C_MASTER_NUM, BMP280_SENSOR_ADDR);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "BMP280 sensor initialization failed, stopping.");
        return;
    }

    // TODO make 0x68 a define
    //  --- Initialize and check RTC ---
    ds3231_init(&g_rtc, I2C_NUM_0, 0x68);
    struct tm timeinfo;
    time_t rtc_ts = -1;
    if (xSemaphoreTake(g_i2c_bus_mutex, portMAX_DELAY))
    {
        if (ds3231_get_time(&g_rtc, &timeinfo) == 0)
        {
            // Since system TZ is UTC, mktime works like timegm.
            rtc_ts = mktime(&timeinfo);
        }
        xSemaphoreGive(g_i2c_bus_mutex);
    }

    if (rtc_ts != -1)
    {
        struct tm build_time;
        sscanf(__DATE__, "%*s %*d %d", &build_time.tm_year);
        build_time.tm_year -= 1900;

        // If RTC year is before build year, set the time
        if (timeinfo.tm_year < build_time.tm_year)
        {
            ESP_LOGW(TAG, "RTC time is invalid. Setting to build time.");
            set_rtc_to_build_time();
        }

        rtc_available = true;
        ESP_LOGI(TAG, "RTC available, using DS3231 for timestamps.");
        // Set system time to UTC
        setenv("TZ", "UTC0", 1);
        tzset();
        struct timeval tv = {.tv_sec = rtc_ts};
        settimeofday(&tv, NULL);
        ESP_LOGI(TAG, "System time synchronized from DS3231 RTC to UTC.");
    }
    else
    {
        rtc_available = false;
        ESP_LOGW(TAG, "RTC not available, using ESP32 system time.");
    }

    // Initialize Rotary Encoder
    rotaryencoder_config_t encoder_cfg = {
        .pin_a = ROTARY_ENCODER_PIN_A,
        .pin_b = ROTARY_ENCODER_PIN_B,
        .button_pin = ROTARY_ENCODER_BUTTON_PIN,
        .button_debounce_ms = BUTTON_DEBOUNCE_TIME_MS};
    rotaryencoder_init(&encoder_cfg);

    // Create the rotary encoder task
    rotaryencoder_start_task();

    // Initialize UI renderer which will handle its own I2C bus and OLED init
    uiRender_init(I2C_OLED_NUM, I2C_OLED_SDA_IO, I2C_OLED_SCL_IO);
    i2c_oled_clear(I2C_OLED_NUM);
    i2c_oled_write_text(I2C_OLED_NUM, 1, 0, "Booting...");

    spi_sdcard_full_init();
    ESP_LOGI(TAG, "Starting  sensor readings and Encoder monitoring...");

    xTaskCreate(uiRender_task, "uiRender", 4096, NULL, 5, &g_uiRender_task_handle);

    xTaskCreate(datalogger_task, "datalogger", 4096, NULL, 5, NULL); // Datalogger at priority 5

    xTaskCreate(main_task, "main_task", 4096, NULL, 6, NULL); // Main command processing task at priority 6
}
