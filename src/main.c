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

#include "../lib/i2c-lcd/i2c-lcd.h"
#include "../lib/i2c-bmp180/i2c-bmp180.h" // BMP180 sensor API
#include "../lib/rotaryencoder/rotaryencoder.h"

// --- I2C Configuration ---
#define I2C_MASTER_SCL_IO GPIO_NUM_21
#define I2C_MASTER_SDA_IO GPIO_NUM_20
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0
#define I2C_MASTER_TIMEOUT_MS 1000

// --- Rotary Encoder Configuration ---
#define ROTARY_ENCODER_PIN_A GPIO_NUM_41
#define ROTARY_ENCODER_PIN_B GPIO_NUM_42
#define ROTARY_ENCODER_BUTTON_PIN GPIO_NUM_5

#define BUTTON_DEBOUNCE_TIME_MS 50

static const char *TAG = "i2c-simple-example";
static const char *LCD_TAG = "LCD_DISPLAY";

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

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
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
        .button_debounce_ms = BUTTON_DEBOUNCE_TIME_MS
    };
    rotaryencoder_init(&encoder_cfg);

    // Create the rotary encoder task
    rotaryencoder_start_task();

    // --- Initialize LCD ---
    /*x lcd_init();*/

    // --- Main Loop for BMP180 Readings and LCD Updates ---
    int32_t uncomp_temp = 0, uncomp_press = 0; // Initialized to zero
    float temperature_c = 0.0f;                // Initialized to zero
    long pressure_pa = 0L;                     // Initialized to zero

    /*char line1_buf[LCD_COLUMNS + 1]; // Buffer for first LCD line
    char line2_buf[LCD_COLUMNS + 1]; // Buffer for second LCD line*/
    const char *dir_str = "";
    const char *btn_str_short = "";

    ESP_LOGI(TAG, "Starting BMP180 sensor readings and Encoder monitoring...");

    lcd_init();
    lcd_clear();

    ESP_LOGI(TAG, "Starting BMP180 sensor readings and Encoder monitoring...");

    while (1)
    {
        // --- BMP180 Readings ---
        int32_t uncomp_temp = bmp180_read_raw_temp();
        float temperature_c = bmp180_compensate_temperature(uncomp_temp);

        int32_t uncomp_press = bmp180_read_raw_pressure();
        long pressure_pa = bmp180_compensate_pressure(uncomp_press);

        ESP_LOGI(TAG, "Temperature: %.2f C, Pressure: %ld Pa", temperature_c, pressure_pa);

        // --- Rotary Encoder State ---
        encoder_direction_t direction = rotaryencoder_get_direction();
        encoder_button_state_t button_state = rotaryencoder_get_button_state();

        // Display on LCD
        char LCDout[16];
        lcd_put_cur(0, 0);
        snprintf(LCDout, sizeof(LCDout), "Temp: %.2f C", temperature_c);
        lcd_send_string(LCDout);

        lcd_put_cur(1, 0);
        snprintf(LCDout, sizeof(LCDout), "Pres: %ld Pa", pressure_pa);
        lcd_send_string(LCDout);

   

        vTaskDelay(pdMS_TO_TICKS(2000)); // Read sensors and update LCD every 5 seconds
    }
}
