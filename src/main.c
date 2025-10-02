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

#include "../lib/i2c-lcd/i2c-lcd.h"
#include "../lib/i2c-bmp280/i2c-bmp280.h" // BMP280 sensor API
#include "../lib/rotaryencoder/rotaryencoder.h"
#include "../lib/spi-sdcard/spi-sdcard.h"
#include "../lib/i2c-oled/i2c-oled.h"
#include "ui_render.h"
#include "../lib/buffers.h"
#include "i2c-ds3231.h"
#include "datalogger_task.h"

// DO NOT use pins 19 and 20, they are used by the flash memory

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
#define ROTARY_ENCODER_BUTTON_PIN GPIO_NUM_5

#define BUTTON_DEBOUNCE_TIME_MS 50

static const char *TAG = "DifferentialPressureSensor";


bool rtc_available = false;

ds3231_t g_rtc; // Global RTC device handle
bmp280_t g_bmp280; // Global BMP280 device handle

static int set_rtc_to_build_time(void) {
    if (!rtc_available) {
        ESP_LOGE(TAG, "Cannot set time, RTC not available.");
        return -1; // RTC not available
    }
     //TODO make this parametric to address different timezones
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
    for (int i = 0; i < 12; i++) {
        if (strcmp(month_str, months[i]) == 0) {
            build_time.tm_mon = i;
            break;
        }
    }

    // Convert local build time to a UTC timestamp, then back to a UTC-based struct tm
    time_t build_ts = mktime(&build_time);
    struct tm utc_build_time;
    gmtime_r(&build_ts, &utc_build_time);

    int ret = -1;
    if (xSemaphoreTake(g_i2c_bus_mutex, portMAX_DELAY)) {
        ret = ds3231_set_time(&g_rtc, &utc_build_time);
        xSemaphoreGive(g_i2c_bus_mutex);
    }

    if (ret == 0) {
        ESP_LOGI(TAG, "RTC time set to build time: %s %s", date_str, time_str);
        // Also update system time immediately
        struct timeval tv = {.tv_sec = build_ts};
        settimeofday(&tv, NULL);
        return 0;
    } else {
        ESP_LOGE(TAG, "Failed to set RTC time.");
        return -1;
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

// --- Main Application Entry Point ---

void main_task(void *pvParameters)
{
    while (1)
    {
        // --- Process commands from other tasks ---
        app_command_t cmd;
        if (xQueueReceive(g_app_cmd_queue, &cmd, 0) == pdPASS) {
            switch (cmd) {
                case APP_CMD_SET_RTC_BUILD_TIME:
                    ESP_LOGI(TAG, "Received command to set RTC to build time.");
                    if (xSemaphoreTake(g_command_status_mutex, portMAX_DELAY)) {
                        g_command_status = CMD_STATUS_PENDING;
                        xSemaphoreGive(g_command_status_mutex);
                    }
                    if (set_rtc_to_build_time() == 0) {
                        if (xSemaphoreTake(g_command_status_mutex, portMAX_DELAY)) {
                            g_command_status = CMD_STATUS_SUCCESS;
                            xSemaphoreGive(g_command_status_mutex);
                        }
                    } else {
                        if (xSemaphoreTake(g_command_status_mutex, portMAX_DELAY)) {
                            g_command_status = CMD_STATUS_FAIL;
                            xSemaphoreGive(g_command_status_mutex);
                        }
                    }
                    break;
                default:
                    ESP_LOGW(TAG, "Unknown command received: %d", cmd);
                    break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
void app_main(void)
{
    g_command_status_mutex = xSemaphoreCreateMutex();
    g_i2c_bus_mutex = xSemaphoreCreateMutex();

    g_sensor_buffer_mutex = xSemaphoreCreateMutex();
    g_app_cmd_queue = xQueueCreate(10, sizeof(app_command_t));

    // Initialize I2C
    esp_err_t err = i2c_master_init();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C master initialization failed, stopping.");
        return;
    }

    // Initialize BMP180 Sensor
    /*err = bmp280_init(&g_bmp280, I2C_MASTER_NUM, BMP280_SENSOR_ADDR);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "BMP280 sensor initialization failed, stopping.");
        return;
    }*/

    // --- Initialize and check RTC ---
    ds3231_init(&g_rtc, I2C_NUM_0, 0x68);
    struct tm timeinfo;
    time_t rtc_ts = -1;
    if (xSemaphoreTake(g_i2c_bus_mutex, portMAX_DELAY)) {
        if (ds3231_get_time(&g_rtc, &timeinfo) == 0) {
            rtc_ts = mktime(&timeinfo);
        }
        xSemaphoreGive(g_i2c_bus_mutex);
    }

    if (rtc_ts != -1) {
        struct tm build_time;
        sscanf(__DATE__, "%*s %*d %d", &build_time.tm_year);
        build_time.tm_year -= 1900;

        // If RTC year is before build year, set the time
       if (timeinfo.tm_year < build_time.tm_year) {
            ESP_LOGW(TAG, "RTC time is invalid. Setting to build time.");
            set_rtc_to_build_time();
        }

        rtc_available = true;
        ESP_LOGI(TAG, "RTC available, using DS3231 for timestamps.");
        setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 1);
        tzset();
        struct timeval tv = {.tv_sec = rtc_ts};
        settimeofday(&tv, NULL);
        ESP_LOGI(TAG, "System time synchronized from DS3231 RTC.");
    } else {
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

    spi_sdcard_full_init(); // Call once at startup

    i2c_oled_clear(I2C_OLED_NUM);
    ESP_LOGI(TAG, "Starting BMP280 sensor readings and Encoder monitoring...");

    xTaskCreate(uiRender_task, "uiRender", 4096, NULL, 5, NULL);

    xTaskCreate(datalogger_task, "datalogger", 4096, NULL, 5, NULL); // Datalogger at priority 5

    xTaskCreate(main_task, "main_task", 4096, NULL, 6, NULL); // Main command processing task at priority 6
}
