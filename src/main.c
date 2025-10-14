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
#include "../lib/i2c-ds3231/i2c-ds3231.h"
#include "datalogger_task.h"
#include "../lib/lipo-battery/lipo-battery.h"

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
#define OLED_I2C_ADDR 0x3C

// --- Rotary Encoder Configuration ---
#define ROTARY_ENCODER_PIN_A GPIO_NUM_41
#define ROTARY_ENCODER_PIN_B GPIO_NUM_42
#define ROTARY_ENCODER_BUTTON_PIN GPIO_NUM_7
#define BUTTON_DEBOUNCE_TIME_MS 50
#define SELF_WAKEUP_TIMEOUT_MS 5000
#define INACTIVITY_TIMEOUT_MS 30000          // 10 seconds
#define SLEEP_DURATION_US (59 * 1000 * 1000) // 45 seconds

// --- Battery ADC Configuration ---
#define BATTERY_PWR_PIN GPIO_NUM_5
#define BATTERY_ADC_PIN GPIO_NUM_6
#define BATTERY_VOLTAGE_DIVIDER_RATIO 4.1428f // (470k + 150k) / 150k, then tuned experimentally

// --- Global buffer and mutex definition ---
sensor_buffer_t g_sensor_buffer = {.writeStatus = WRITE_STATUS_UNKNOWN};
SemaphoreHandle_t g_sensor_buffer_mutex = NULL;
QueueHandle_t g_app_cmd_queue = NULL;
SemaphoreHandle_t g_command_status_mutex = NULL;
SemaphoreHandle_t g_i2c_bus_mutex = NULL;
volatile command_status_t g_command_status = CMD_STATUS_IDLE;
QueueHandle_t g_datalogger_cmd_queue = NULL;

static const char *TAG = "main";

bool rtc_available = false;

ds3231_t g_rtc;    // Global RTC device handle
bmp280_t g_bmp280; // Global BMP280 device handle
d6fph_t g_d6fph;   // Global D6F-PH device handle

TaskHandle_t g_uiRender_task_handle = NULL;

static uint64_t last_activity_ms = 0;
static uint64_t last_self_wakeup_ms = 0;
static bool is_usb_connected_state = false;

/**
 * @brief Callback for clockwise rotation from the rotary encoder.
 */
static void on_encoder_rotate_cw(void)
{
    uiRender_send_event(UI_EVENT_CW, NULL, 0);
    uiRender_reset_activity_timer();
}

/**
 * @brief Callback for counter-clockwise rotation from the rotary encoder.
 */
static void on_encoder_rotate_ccw(void)
{
    uiRender_send_event(UI_EVENT_CCW, NULL, 0);
    uiRender_reset_activity_timer();
}

/**
 * @brief Callback for button press from the rotary encoder.
 */
static void on_encoder_button_press(void)
{
    uiRender_send_event(UI_EVENT_BTN, NULL, 0);
    uiRender_reset_activity_timer();
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
    if (!rtc_available)
    {
        ESP_LOGE(TAG, "Cannot set time, RTC not available.");
        g_command_status = CMD_STATUS_FAIL;
        return;
    }

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
 * @brief Re-initializes the BMP280 sensor.
 *
 * This function is called after waking from sleep to ensure the sensor
 * is responsive. It takes the I2C bus mutex before re-initializing the sensor.
 */
static void bmp280_reinit(void)
{
    ESP_LOGI(TAG, "Re-initializing BMP280 sensor.");
    if (xSemaphoreTake(g_i2c_bus_mutex, portMAX_DELAY))
    {
        esp_err_t err = bmp280_init(&g_bmp280, I2C_MASTER_NUM, BMP280_SENSOR_ADDR);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "BMP280 sensor re-initialization failed.");
        }
        xSemaphoreGive(g_i2c_bus_mutex);
    }
    else
    {
        ESP_LOGE(TAG, "Failed to acquire I2C mutex for BMP280 re-init.");
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
    if (eTaskGetState(g_uiRender_task_handle) != eSuspended)
    {
        ESP_LOGI(TAG, "Powering off OLED.");
        uiRender_send_event(UI_EVENT_PREPARE_SLEEP, NULL, 0);
        vTaskDelay(pdMS_TO_TICKS(200)); // Give UI time to draw sleep message
        vTaskSuspend(g_uiRender_task_handle);
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
    if (eTaskGetState(g_uiRender_task_handle) == eSuspended)
    {
        ESP_LOGI(TAG, "Powering on OLED.");
        gpio_set_level(OLED_POWER_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(100)); // Wait for OLED to stabilize
        i2c_oled_send_init_commands(I2C_OLED_NUM);
        vTaskResume(g_uiRender_task_handle);
        uiRender_send_event(UI_EVENT_WAKE_UP, NULL, 0);
    }
}

/**
 * @brief Reads the current time from the DS3231 RTC and updates the system time.
 *
 * This is used to synchronize the ESP32's internal system clock with the more
 * accurate external RTC, especially after waking from sleep.
 */
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
        if (ds3231_get_time(&g_rtc, &timeinfo) == ESP_OK)
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
 *
 * Configures and installs the I2C driver for the main peripheral bus (I2C_NUM_0).
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
                handle_set_rtc_to_build_time();
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
                oled_power_on();
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

        // --- Main Sleep/Wake Logic ---
        // --- Power Saving & UI Inactivity Logic ---
        uint64_t current_time_ms = esp_timer_get_time() / 1000ULL;
        bool ui_is_inactive = (current_time_ms - last_activity_ms > INACTIVITY_TIMEOUT_MS);

        // Check if a new data entry has been successfully written to the SD card.
        time_t current_write_ts = 0;
        if (xSemaphoreTake(g_sensor_buffer_mutex, pdMS_TO_TICKS(50)))
        {
            current_write_ts = g_sensor_buffer.last_successful_write_ts;
            xSemaphoreGive(g_sensor_buffer_mutex);
        }

        // --- USB Connection Logic ---
        bool is_externally_powered = battery_is_externally_powered();
        if (is_externally_powered && !is_usb_connected_state)
        {
            // USB was just connected
            ESP_LOGI(TAG, "USB connected. Re-initializing for MSC.");
            // De-init SD card first to release resources
            spi_sdcard_deinit();
            // Now, do a full init which includes TinyUSB
            spi_sdcard_full_init();
            is_usb_connected_state = true;
        }
        else if (!is_externally_powered && is_usb_connected_state)
        {
            // USB was just disconnected. We keep the TinyUSB stack running
            // in case it's plugged back in. It will be de-initialized before sleep.
            ESP_LOGI(TAG, "USB disconnected. TinyUSB stack remains active until sleep.");
            is_usb_connected_state = false;
        }

        // If USB is mounted by a host, treat it as continuous activity to prevent sleep.
        if (spi_sdcard_is_usb_connected())
        {
            last_activity_ms = current_time_ms;
        }

        // Determine if it's time to sleep. Conditions are:
        // 1. The UI has been inactive for the timeout period.
        // 2. EITHER a new data log has occurred OR a self-wakeup timeout has been reached
        //    (to ensure the device sleeps even if SD card writes are failing).
        bool new_write_occurred = (current_write_ts > 0 && current_write_ts > last_processed_write_ts);
        bool self_wakeup_timeout = (current_time_ms - last_self_wakeup_ms > SELF_WAKEUP_TIMEOUT_MS);

        // ESP_LOGI("main", "Inactivity: %llums, Since last self-wakeup: %llums, New write: %d, UI inactive: %d", current_time_ms - last_activity_ms, current_time_ms - last_self_wakeup_ms, new_write_occurred, ui_is_inactive);
        // --- Enter Sleep ---
        // If the UI is inactive and a new log has been written, initiate the sleep sequence.
        if (ui_is_inactive && (new_write_occurred || (self_wakeup_timeout && current_write_ts > 0)))
        {
            if (self_wakeup_timeout)
            {
                ESP_LOGI(TAG, "UI inactive and self-wakeup timeout reached. Initiating sleep.");
                last_self_wakeup_ms = current_time_ms; // Reset self-wakeup timer
            }
            else
            {
                ESP_LOGI(TAG, "UI inactive and new data logged. Initiating sleep.");
            }
            last_processed_write_ts = current_write_ts; // Mark this write as processed

            // --- Step 1: Prepare UI and Peripherals for Sleep ---
            // Suspend UI task and power down OLED to save power.
            oled_power_off(); // This will suspend the task and power down the OLED

            // Tell the datalogger task to pause writes to the SD card.
            ESP_LOGI(TAG, "Requesting datalogger to pause...");
            datalogger_command_t logger_cmd = DATALOGGER_CMD_PAUSE_WRITES;
            xQueueSend(g_datalogger_cmd_queue, &logger_cmd, 0);

            // 3. Wait for the datalogger to confirm it's paused
            int datalogger_status = 0;
            int wait_cycles = 0;
            // Wait for the datalogger to update its status to PAUSED.
            do
            {
                vTaskDelay(pdMS_TO_TICKS(20));
                if (xSemaphoreTake(g_sensor_buffer_mutex, pdMS_TO_TICKS(50)))
                {
                    datalogger_status = g_sensor_buffer.datalogger_status;
                    xSemaphoreGive(g_sensor_buffer_mutex);
                }
                wait_cycles++;
            } while (datalogger_status != DATA_LOGGER_PAUSED && wait_cycles < 120); // Max 3 sec wait

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

            // De-initialize peripherals that will be powered down.
            ESP_LOGI(TAG, "De-initializing peripherals before sleep.");
            spi_sdcard_deinit(); // Inlined from peripherals_deinit
            // OLED I2C driver is already deleted by oled_power_off()

            // Cut power to the main peripheral power rail.
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

            // --- WAKE UP SEQUENCE ---
            // Restore power to peripherals immediately after waking up.
            gpio_set_level(DEVICES_POWER_PIN, 1);
            vTaskDelay(pdMS_TO_TICKS(100));
            ESP_LOGI(TAG, "Powered up external devices. Re-initializing...");

            esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
            if (cause == ESP_SLEEP_WAKEUP_TIMER)
            // Woke up by timer, meaning no user interaction.
            {
                ESP_LOGI(TAG, "Woke up from light sleep due to timer.");
                last_self_wakeup_ms = esp_timer_get_time() / 1000ULL;
                // Keep OLED off and UI task suspended.
                // Only re-init SD card for datalogger.
                spi_sdcard_init_sd_only();
            }
            else if (cause == ESP_SLEEP_WAKEUP_GPIO)
            // Woke up by GPIO (user interaction).
            {
                ESP_LOGI(TAG, "Woke up from light sleep due to GPIO.");
                // Power on OLED, re-init SD card, and resume UI task.
                oled_power_on();
                spi_sdcard_init_sd_only(); // Re-init SD card only, USB will be handled by the main loop

                // Debounce after GPIO wakeup to prevent the wake-up press from being processed as a command.
                vTaskDelay(pdMS_TO_TICKS(100));
                xQueueReset(rotaryencoder_get_queue_handle());
                last_activity_ms = esp_timer_get_time() / 1000ULL;    // Reset activity timer on wakeup
                last_self_wakeup_ms = esp_timer_get_time() / 1000ULL; // Reset self-wakeup timer on wakeup
            }
            else
            {
                ESP_LOGI(TAG, "Woke up from light sleep due to other reason (%d)", cause);
            }

            // Re-initialize the BMP280 sensor as it may have lost its state.
            bmp280_reinit(); // Re-initialize BMP280 sensor
            // Tell the datalogger task to resume writing data to the SD card.
            logger_cmd = DATALOGGER_CMD_RESUME_WRITES;
            xQueueSend(g_datalogger_cmd_queue, &logger_cmd, 0);

            // Re-install normal ISRs for the rotary encoder for active operation.
            gpio_isr_handler_add(ROTARY_ENCODER_PIN_A, gpio_isr_handler, (void *)ROTARY_ENCODER_PIN_A);
            gpio_isr_handler_add(ROTARY_ENCODER_PIN_B, gpio_isr_handler, (void *)ROTARY_ENCODER_PIN_B);
            gpio_isr_handler_add(ROTARY_ENCODER_BUTTON_PIN, gpio_isr_handler, (void *)ROTARY_ENCODER_BUTTON_PIN);

            gpio_wakeup_disable(ROTARY_ENCODER_BUTTON_PIN); // Disable wakeup to use normal ISR

            // Re-sync system time with the external RTC.
            resync_time_from_rtc(); // Re-sync system time with external RTC
        }
        else if (ui_is_inactive && eTaskGetState(g_uiRender_task_handle) != eSuspended)
        {
            uiRender_send_event(UI_EVENT_PREPARE_SLEEP, NULL, 0);
            vTaskDelay(pdMS_TO_TICKS(300)); // Give UI time to draw sleep message
            // UI timeout occurred, but no new write yet. Just turn off the screen.
            oled_power_off();
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main(void)
{
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
    gpio_set_level(OLED_POWER_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(50)); // Wait 50ms after powering up devices

    // --- Step 2: Initialize RTOS objects (Mutexes and Queues) ---
    g_command_status_mutex = xSemaphoreCreateMutex();
    g_i2c_bus_mutex = xSemaphoreCreateMutex();
    g_sensor_buffer_mutex = xSemaphoreCreateMutex();
    g_app_cmd_queue = xQueueCreate(10, sizeof(app_command_t));
    g_datalogger_cmd_queue = xQueueCreate(5, sizeof(datalogger_command_t));

    // --- Step 3: Initialize hardware drivers and peripherals ---
    // Initialize Battery Reader first, as it uses ADC which can conflict if I2C is initialized first
    battery_reader_init(BATTERY_ADC_PIN, BATTERY_PWR_PIN, BATTERY_VOLTAGE_DIVIDER_RATIO);

    // Initialize main I2C bus for sensors.
    esp_err_t err = i2c_master_init();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C master initialization failed, stopping.");
        return;
    }

    // Initialize BMP280 Sensor.
    err = bmp280_init(&g_bmp280, I2C_MASTER_NUM, BMP280_SENSOR_ADDR);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "BMP280 sensor initialization failed, stopping.");
        return;
    }
/*
    // Initialize D6F-PH Sensor.
    err = d6fph_init(&g_d6fph, I2C_MASTER_NUM, D6FPH_I2C_ADDR_DEFAULT, D6FPH_MODEL_0025AD1);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "D6F-PH sensor initialization failed, stopping.");
    }*/

    // --- Step 4: Initialize and validate the external Real-Time Clock (RTC) ---
    // TODO make 0x68 a define
    ds3231_init(&g_rtc, I2C_NUM_0, 0x68);
    struct tm timeinfo;
    time_t rtc_ts = -1;

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

        rtc_available = true;
        ESP_LOGI(TAG, "RTC available, using DS3231 for timestamps.");
        // Set system timezone to UTC and synchronize system time with RTC.
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

    // --- Step 5: Initialize user input (Rotary Encoder) ---
    rotaryencoder_config_t encoder_cfg = {
        .pin_a = ROTARY_ENCODER_PIN_A,
        .pin_b = ROTARY_ENCODER_PIN_B,
        .button_pin = ROTARY_ENCODER_BUTTON_PIN,
        .button_debounce_ms = BUTTON_DEBOUNCE_TIME_MS,
        .on_rotate_cw = on_encoder_rotate_cw,
        .on_rotate_ccw = on_encoder_rotate_ccw,
        .on_button_press = on_encoder_button_press,
    };
    rotaryencoder_init(&encoder_cfg);

    // --- Step 6: Initialize UI and SD card ---
    // Initialize UI renderer which will handle its own I2C bus, OLED, and rotary encoder.
    uiRender_init(I2C_OLED_NUM, I2C_OLED_SDA_IO, I2C_OLED_SCL_IO, OLED_I2C_ADDR);

    i2c_oled_clear(I2C_OLED_NUM);
    i2c_oled_write_text(I2C_OLED_NUM, 1, 0, "Booting...");

    spi_sdcard_full_init();

    // --- Step 7: Start all application tasks ---
    ESP_LOGI(TAG, "Starting  sensor readings and Encoder monitoring...");
    rotaryencoder_start_task();

    // Prepare parameters for the datalogger task
    datalogger_task_params_t datalogger_params = {
        .bmp280_dev = &g_bmp280,
        .d6fph_dev = &g_d6fph
    };

    xTaskCreate(uiRender_task, "uiRender", 4096, NULL, 5, &g_uiRender_task_handle);
    xTaskCreate(datalogger_task, "datalogger", 4096, &datalogger_params, 5, NULL); // Pass params struct
    xTaskCreate(main_task, "main_task", 4096, NULL, 6, NULL);             // Main command processing task at priority 6
}
