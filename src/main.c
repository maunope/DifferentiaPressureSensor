// ESP-IDF script for reading I2C barometric sensor (BMP180) and rotary encoder,
// printing data to serial and a 2-line I2C LCD using a self-contained driver.

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
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
#include "../lib/i2c-bmp180/i2c-bmp180.h" // BMP180 sensor API
#include "../lib/rotaryencoder/rotaryencoder.h"
#include "../lib/spi-sdcard/spi-sdcard.h"
#include "../lib/i2c-oled/i2c-oled.h"

// --- I2C Configuration for peripherals ---
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_SCL_IO GPIO_NUM_21
#define I2C_MASTER_SDA_IO GPIO_NUM_20
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

// --- Main Application Entry Point ---
void app_main(void)
{

   
    // Initialize I2C
    esp_err_t err = i2c_master_init();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C master initialization failed, stopping.");
        return;
    }

    // Initialize BMP180 Sensor
    err = bmp180_init();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "BMP180 sensor initialization failed, stopping.");
        return;
    }

 

    volatile int dummy = 0; // This variable cannot be optimized away


    spi_sdcard_full_init(); // Call once at startup

    // Example usage of BMP180 API
    int32_t UT = bmp180_read_raw_temp();
    float temperature = bmp180_compensate_temperature(UT);

    int32_t UP = bmp180_read_raw_pressure();
    long pressure = bmp180_compensate_pressure(UP);

    ESP_LOGI(TAG, "Temperature: %.2f C, Pressure: %ld Pa", temperature, pressure);

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
   


    
   
    ESP_LOGI(TAG, "Starting BMP180 sensor readings and Encoder monitoring...");

   // lcd_init();
   // lcd_clear();
    //spi_sdcard_full_init(); // Call once at startup

    ESP_LOGI(TAG, "Starting BMP180 sensor readings and Encoder monitoring...");

    i2c_oled_clear(I2C_OLED_NUM); 

    while (1)
    {
          

        // --- BMP180 Readings ---
        int32_t uncomp_temp = bmp180_read_raw_temp();
        float temperature_c = bmp180_compensate_temperature(uncomp_temp);

        int32_t uncomp_press = bmp180_read_raw_pressure();
        long pressure_pa = bmp180_compensate_pressure(uncomp_press);

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
        int writeStatus = spi_sdcard_write_csv("dati.csv", strftime_buf, temperature_c, pressure_pa);

        ESP_LOGI(TAG, "Temperature: %.2f C, Pressure: %ld Pa", temperature_c, pressure_pa);

        // --- Rotary Encoder State ---
      
        // Display on Olde
        
        i2c_oled_write_text(I2C_OLED_NUM, 0, 0, "Stato letture");
         
      
        char oledOut[16];
  
        snprintf(oledOut, sizeof(oledOut), "Temp: %.2f C", temperature_c);
        i2c_oled_write_text(I2C_OLED_NUM, 1, 0, oledOut);

      
        snprintf(oledOut, sizeof(oledOut), "Pres: %ld Pa", pressure_pa);
        i2c_oled_write_text(I2C_OLED_NUM, 2, 0, oledOut);
        
        snprintf(oledOut, sizeof(oledOut), "File write: %s", writeStatus==SPI_CARD_OK?"OK":"KO");
        i2c_oled_write_text(I2C_OLED_NUM, 3, 0, oledOut);
        

        vTaskDelay(pdMS_TO_TICKS(2000)); // Read sensors and update LCD every 2 seconds
    }
}
