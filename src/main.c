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

#include "../lib/i2c-lcd/i2c-lcd.h"
#include "../lib/i2c-bmp280/i2c-bmp280.h" // BMP280 sensor API
#include "../lib/rotaryencoder/rotaryencoder.h"
#include "../lib/spi-sdcard/spi-sdcard.h"
#include "../lib/i2c-oled/i2c-oled.h"
#include "ui_render.h"
#include "../lib/buffers.h"


//DO NOT use pins 19 and 20, they are used by the flash memory

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


char buffer[10];
float num = 12.34;

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

// --- Main Application Entry Point ---
void app_main(void)
{

    g_sensor_buffer_mutex = xSemaphoreCreateMutex();
   
    // Initialize I2C
    esp_err_t err = i2c_master_init();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C master initialization failed, stopping.");
        return;
    }

    // Initialize BMP180 Sensor
    err = bmp280_init();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "BMP280 sensor initialization failed, stopping.");
        return;
    }

 

    volatile int dummy = 0; // This variable cannot be optimized away




    // Initialize Rotary Encoder
    rotaryencoder_config_t encoder_cfg = {
        .pin_a = ROTARY_ENCODER_PIN_A,
        .pin_b = ROTARY_ENCODER_PIN_B,
        .button_pin = ROTARY_ENCODER_BUTTON_PIN,
        .button_debounce_ms = BUTTON_DEBOUNCE_TIME_MS};
    rotaryencoder_init(&encoder_cfg);

    // Create the rotary encoder task
    rotaryencoder_start_task();

    // --- Initialize second I2C bus for OLED ---
    i2c_config_t oled_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_OLED_SDA_IO, // e.g. GPIO_NUM_19
        .scl_io_num = I2C_OLED_SCL_IO, // e.g. GPIO_NUM_18
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000
    };
    i2c_param_config(I2C_OLED_NUM, &oled_conf);
    i2c_driver_install(I2C_OLED_NUM, oled_conf.mode, 0, 0, 0);

    // --- OLED test ---

    i2c_oled_init(I2C_OLED_NUM, I2C_OLED_SDA_IO, I2C_OLED_SCL_IO);
    i2c_oled_clear(I2C_OLED_NUM); 
    i2c_oled_write_text(I2C_OLED_NUM, 1, 0, "Booting...");

    spi_sdcard_full_init(); // Call once at startup
   
  
    
    i2c_oled_clear(I2C_OLED_NUM); 
    ESP_LOGI(TAG, "Starting BMP280 sensor readings and Encoder monitoring...");

    // Initialize UI renderer with OLED config
    uiRender_init(I2C_OLED_NUM, I2C_OLED_SDA_IO, I2C_OLED_SCL_IO);

    xTaskCreate(uiRender_task, "uiRender", 4096, NULL, 5, NULL);

   

    while (1)
    {
          

        // --- BMP280 Readings ---
        int32_t uncomp_temp = bmp280_read_raw_temp();
        float temperature_c = bmp280_compensate_temperature(uncomp_temp);

        int32_t uncomp_press = bmp280_read_raw_pressure();
        float pressure_pa = bmp280_compensate_pressure(uncomp_press);

        time_t now;
        char strftime_buf[64];
        struct tm timeinfo;

        time(&now);
        // Set timezone to China Standard Time
        setenv("TZ", "CEST", 1);
        tzset();

        localtime_r(&now, &timeinfo);
        strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
    
        // --- Write to SD card in CSV format ---
        int writeStatus = -1;
        writeStatus = spi_sdcard_write_csv("dati.csv", strftime_buf, temperature_c, pressure_pa);

        ESP_LOGI(TAG, "Temperature: %.2f C, Pressure: %.2f Pa, write status %d", temperature_c, pressure_pa, writeStatus);

        // --- Copy to shared buffer under mutex ---
        if (g_sensor_buffer_mutex && xSemaphoreTake(g_sensor_buffer_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            g_sensor_buffer.temperature_c = temperature_c;
            g_sensor_buffer.pressure_pa = pressure_pa;
            g_sensor_buffer.writeStatus = writeStatus;
            xSemaphoreGive(g_sensor_buffer_mutex);
        }

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
