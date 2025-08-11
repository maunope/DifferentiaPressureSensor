// main.c
// ESP-IDF script for reading I2C barometric sensor (BMP180) and rotary encoder,
// printing data to serial and a 2-line I2C LCD.

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h" // Include FreeRTOS Queue
#include "esp_system.h"
#include "esp_log.h"
#include "driver/i2c.h" // Include I2C driver
#include "driver/gpio.h" // Include GPIO driver for encoder
#include "sdkconfig.h"  // For CONFIG_I2C_MASTER_SCL and CONFIG_I2C_MASTER_SDA
#include "esp_rom_sys.h" // Required for esp_rom_delay_us

// --- I2C Configuration ---
#define I2C_MASTER_SCL_IO           GPIO_NUM_21    // GPIO pin for I2C SCL
#define I2C_MASTER_SDA_IO           GPIO_NUM_20    // GPIO pin for I2C SDA
#define I2C_MASTER_NUM              I2C_NUM_0      // I2C port number
#define I2C_MASTER_FREQ_HZ          100000         // I2C clock frequency
#define I2C_MASTER_TX_BUF_DISABLE   0              // I2C master does not need buffer
#define I2C_MASTER_RX_BUF_DISABLE   0              // I2C master does not need buffer
#define I2C_MASTER_TIMEOUT_MS       1000           // I2C transaction timeout

// --- BMP180 Sensor Specifics ---
#define BMP180_SENSOR_ADDR          0x77           // I2C address of BMP180 (SDA HIGH)
#define BMP180_CHIP_ID_REG          0xD0           // Chip ID register (should be 0x55)
#define BMP180_CTRL_MEAS_REG        0xF4           // Control Measurement register
#define BMP180_TEMP_CAL_MSB_REG     0xF6           // Uncompensated Temperature MSB
#define BMP180_PRESS_CAL_MSB_REG    0xF6           // Uncompensated Pressure MSB (same register but different data length)
#define BMP180_CALIB_DATA_START     0xAA           // Calibration data start address

// Control register values for measurement
#define BMP180_READ_TEMP_CMD        0x2E
#define BMP180_READ_PRESSURE_0_CMD  0x34 // OSS = 0, ultra low power
#define BMP180_READ_PRESSURE_1_CMD  0x74 // OSS = 1, standard
#define BMP180_READ_PRESSURE_2_CMD  0xB4 // OSS = 2, high res
#define BMP180_READ_PRESSURE_3_CMD  0xF4 // OSS = 3, ultra high res

// Oversampling Setting (OSS) - Adjust as needed
// 0: ultra low power, 1: standard, 2: high resolution, 3: ultra high resolution
#define BMP180_OSS                  0 // For simplicity, using ultra low power (0)

// --- Rotary Encoder Configuration ---
#define ROTARY_ENCODER_PIN_A    GPIO_NUM_41 // GPIO for Encoder Phase A
#define ROTARY_ENCODER_PIN_B    GPIO_NUM_42 // GPIO for Encoder Phase B
#define ROTARY_ENCODER_BUTTON_PIN GPIO_NUM_5 // GPIO for Encoder Button

// Debounce delay for button (milliseconds)
#define BUTTON_DEBOUNCE_TIME_MS 50

// --- LCD I2C Configuration (PC8574 Backpack) ---
#define LCD_I2C_ADDR        0x27 // Common LCD I2C address, check your module (can be 0x3F)
#define LCD_LINE_MAX_CHARS  16   // Maximum characters per LCD line (CHANGED TO 16)

// PCF8574 Pin Map to LCD HD44780
#define RS_BIT      (1 << 0) // Register select bit
#define RW_BIT      (1 << 1) // Read/Write bit (always low for writing to LCD)
#define EN_BIT      (1 << 2) // Enable bit
#define BL_BIT      (1 << 3) // Backlight bit (often P3 or P7 on PCF8574, can vary)
#define D4_BIT      (1 << 4) // Data 4 bit
#define D5_BIT      (1 << 5) // Data 5 bit
#define D6_BIT      (1 << 6) // Data 6 bit
#define D7_BIT      (1 << 7) // Data 7 bit

// LCD HD44780 Commands
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_FUNCTIONSET 0x20
#define LCD_SETDDRAMADDR 0x80 // Set DDRAM address

// Flags for display on/off control
#define LCD_DISPLAYON 0x04
#define LCD_CURSOROFF 0x00
#define LCD_BLINKOFF 0x00

// Flags for function set
#define LCD_4BITMODE 0x00
#define LCD_2LINE 0x08
#define LCD_5x8DOTS 0x00

static const char *TAG = "BMP180_SENSOR"; // Tag for ESP_LOG messages
static const char *ENCODER_TAG = "ROTARY_ENCODER"; // Tag for Encoder LOG messages
static const char *LCD_TAG = "LCD_DISPLAY"; // Tag for LCD LOG messages

// --- BMP180 Calibration Data Structure ---
typedef struct {
    int16_t AC1;
    int16_t AC2;
    int16_t AC3;
    uint16_t AC4;
    uint16_t AC5;
    uint16_t AC6;
    int16_t B1;
    int16_t B2;
    int16_t MB;
    int16_t MC;
    int16_t MD;
} bmp180_calib_data_t;

static bmp180_calib_data_t calib_data; // Global variable to store calibration data

// --- FreeRTOS Queue for Encoder Events ---
static QueueHandle_t gpio_evt_queue = NULL;

// --- Global Encoder State ---
typedef enum {
    ENC_DIR_NONE,
    ENC_DIR_CW,
    ENC_DIR_CCW
} encoder_direction_t;

typedef enum {
    ENC_BTN_IDLE,
    ENC_BTN_PRESSED
} encoder_button_state_t;

volatile encoder_direction_t g_encoder_direction = ENC_DIR_NONE;
volatile encoder_button_state_t g_encoder_button_state = ENC_BTN_IDLE;


// --- ISR Handler for GPIO events ---
static void IRAM_ATTR gpio_isr_handler(void* arg) {
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

// --- I2C Master Initialization ---
static esp_err_t i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C parameter config failed: %s", esp_err_to_name(err));
        return err;
    }
    err = i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(err));
    }
    return err;
}

// --- Custom I2C Master Write Byte (Renamed to avoid conflict) ---
static esp_err_t bmp180_i2c_write_byte(i2c_port_t i2c_num, uint8_t reg_addr, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMP180_SENSOR_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BMP180 I2C write byte failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

// --- Custom I2C Master Read Bytes (Renamed to avoid conflict) ---
static esp_err_t bmp180_i2c_read_bytes(i2c_port_t i2c_num, uint8_t reg_addr, uint8_t *data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMP180_SENSOR_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd); // Repeated start
    i2c_master_write_byte(cmd, (BMP180_SENSOR_ADDR << 1) | I2C_MASTER_READ, true);
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BMP180 I2C read bytes failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

// --- Read a 16-bit signed calibration value ---
static int16_t read_s16(uint8_t reg_addr) {
    uint8_t raw_data[2];
    bmp180_i2c_read_bytes(I2C_MASTER_NUM, reg_addr, raw_data, 2);
    return (int16_t)((raw_data[0] << 8) | raw_data[1]);
}

// --- Read a 16-bit unsigned calibration value ---
static uint16_t read_u16(uint8_t reg_addr) {
    uint8_t raw_data[2];
    bmp180_i2c_read_bytes(I2C_MASTER_NUM, reg_addr, raw_data, 2);
    return (uint16_t)((raw_data[0] << 8) | raw_data[1]);
}

// --- Read BMP180 Calibration Data ---
static esp_err_t bmp180_read_calibration_data(void) {
    calib_data.AC1 = read_s16(0xAA);
    calib_data.AC2 = read_s16(0xAC);
    calib_data.AC3 = read_s16(0xAE);
    calib_data.AC4 = read_u16(0xB0);
    calib_data.AC5 = read_u16(0xB2);
    calib_data.AC6 = read_u16(0xB4);
    calib_data.B1 = read_s16(0xB6);
    calib_data.B2 = read_s16(0xB8);
    calib_data.MB = read_s16(0xBA);
    calib_data.MC = read_s16(0xBC);
    calib_data.MD = read_s16(0xBE);

    ESP_LOGI(TAG, "Calibration Data Read:");
    ESP_LOGI(TAG, "AC1: %d, AC2: %d, AC3: %d", calib_data.AC1, calib_data.AC2, calib_data.AC3);
    ESP_LOGI(TAG, "AC4: %u, AC5: %u, AC6: %u", calib_data.AC4, calib_data.AC5, calib_data.AC6);
    ESP_LOGI(TAG, "B1: %d, B2: %d", calib_data.B1, calib_data.B2);
    ESP_LOGI(TAG, "MB: %d, MC: %d, MD: %d", calib_data.MB, calib_data.MC, calib_data.MD);

    return ESP_OK;
}

// --- BMP180 Initialization ---
static esp_err_t bmp180_init(void) {
    esp_err_t ret;
    uint8_t chip_id;

    // Read chip ID to confirm sensor presence
    ret = bmp180_i2c_read_bytes(I2C_MASTER_NUM, BMP180_CHIP_ID_REG, &chip_id, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read chip ID: %s", esp_err_to_name(ret));
        return ret;
    }

    if (chip_id != 0x55) { // BMP180 chip ID is 0x55
        ESP_LOGE(TAG, "BMP180 not found, chip ID: 0x%02X (Expected 0x55)", chip_id);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "BMP180 found, chip ID: 0x%02X", chip_id);

    // Read calibration data
    ret = bmp180_read_calibration_data();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read BMP180 calibration data.");
        return ret;
    }

    return ESP_OK;
}

// --- Read Uncompensated Temperature (UT) ---
static int32_t bmp180_read_raw_temp(void) {
    bmp180_i2c_write_byte(I2C_MASTER_NUM, BMP180_CTRL_MEAS_REG, BMP180_READ_TEMP_CMD);
    vTaskDelay(pdMS_TO_TICKS(5)); // Wait for 4.5ms conversion time for temperature
    uint8_t raw_data[2];
    bmp180_i2c_read_bytes(I2C_MASTER_NUM, BMP180_TEMP_CAL_MSB_REG, raw_data, 2);
    return (int32_t)((raw_data[0] << 8) | raw_data[1]);
}

// --- Read Uncompensated Pressure (UP) ---
static int32_t bmp180_read_raw_pressure(void) {
    uint8_t pressure_cmd = BMP180_READ_PRESSURE_0_CMD + (BMP180_OSS << 6); // Combine command with OSS
    bmp180_i2c_write_byte(I2C_MASTER_NUM, BMP180_CTRL_MEAS_REG, pressure_cmd);

    // Delay based on oversampling setting (OSS)
    int delay_ms;
    switch (BMP180_OSS) {
        case 0: delay_ms = 5; break;  // 4.5ms
        case 1: delay_ms = 8; break;  // 7.5ms
        case 2: delay_ms = 14; break; // 13.5ms
        case 3: delay_ms = 26; break; // 25.5ms
        default: delay_ms = 5; break;
    }
    vTaskDelay(pdMS_TO_TICKS(delay_ms));

    uint8_t raw_data[3];
    bmp180_i2c_read_bytes(I2C_MASTER_NUM, BMP180_PRESS_CAL_MSB_REG, raw_data, 3);
    // BMP180 raw pressure is 19-bit (MSB, LSB, XLSB first 3 bits)
    return (int32_t)(((raw_data[0] << 16) | (raw_data[1] << 8) | raw_data[2]) >> (8 - BMP180_OSS));
}

// Global variable for temperature compensation
long B5;

// --- Compensate Temperature (returns temperature in 0.1 deg C) ---
float bmp180_compensate_temperature(int32_t UT) {
    long X1 = (UT - (long)calib_data.AC6) * calib_data.AC5 / 32768;
    long X2 = ((long)calib_data.MC * 2048) / (X1 + calib_data.MD);
    B5 = X1 + X2;
    float T = (B5 + 8) / 16.0; // Temperature in 0.1 deg C, convert to deg C
    return T / 10.0;
}

// --- Compensate Pressure (returns pressure in Pa) ---
long bmp180_compensate_pressure(int32_t UP) {
    long B6 = B5 - 4000;
    long X1 = (calib_data.B2 * (B6 * B6 / 4096)) / 2048;
    long X2 = (calib_data.AC2 * B6) / 2048;
    long X3 = X1 + X2;
    long B3 = (((long)calib_data.AC1 * 4 + X3) << BMP180_OSS) / 4;

    X1 = (calib_data.AC3 * B6) / 8192;
    X2 = (calib_data.B1 * ((B6 * B6) / 2048)) / 16384;
    X3 = ((X1 + X2) + 2) / 4;
    uint32_t B4 = (calib_data.AC4 * (uint32_t)(X3 + 32768)) / 1000;
    uint32_t B7 = ((uint32_t)UP - B3) * (50000 >> BMP180_OSS);

    long p;
    if (B7 < 0x80000000) {
        p = (B7 * 2) / B4;
    } else {
        p = (B7 / B4) * 2;
    }

    X1 = (p / 256) * (p / 256);
    X1 = (X1 * 3038) / 65536;
    X2 = (-7357 * p) / 65536;
    p = p + ((X1 + X2 + 3791) / 16);

    return p; // Pressure in Pa
}

// --- LCD Driver Functions (Minimal PCF8574 HD44780) ---

// Helper function to send byte to PCF8574
static void i2c_write_byte_lcd(uint8_t val) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LCD_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, val, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        ESP_LOGE(LCD_TAG, "LCD I2C write failed: %s", esp_err_to_name(ret));
    }
}

// Send nibble (4 bits) to LCD
static void lcd_send_nibble(uint8_t nibble, uint8_t mode) {
    uint8_t data_to_send = (nibble & 0xF0) | mode | BL_BIT; // Combine data, RS, and Backlight
    
    // Pulse Enable High
    data_to_send |= EN_BIT;
    i2c_write_byte_lcd(data_to_send);
    esp_rom_delay_us(1); // Hold enable high briefly (microseconds, min 0.45us)
    
    // Pulse Enable Low
    data_to_send &= ~EN_BIT;
    i2c_write_byte_lcd(data_to_send);
    esp_rom_delay_us(50); // Small delay after pulse to ensure LCD processes (microseconds, min 37us)
}

// Send command to LCD
static void lcd_send_cmd(uint8_t cmd) {
    lcd_send_nibble(cmd & 0xF0, 0); // High nibble, RS = 0 (command)
    lcd_send_nibble(cmd << 4, 0);   // Low nibble, RS = 0 (command)
}

// Send data (character) to LCD
static void lcd_send_data(uint8_t data) {
    lcd_send_nibble(data & 0xF0, RS_BIT); // High nibble, RS = 1 (data)
    lcd_send_nibble(data << 4, RS_BIT);   // Low nibble, RS = 1 (data)
}

// Initialize LCD
static void lcd_init(void) {
    // According to HD44780 datasheet, initial power-up wait of >40ms
    vTaskDelay(pdMS_TO_TICKS(100)); // Wait for LCD power-up (increased for robustness)

    // 4-bit initialization sequence (three 8-bit mode commands for robustness)
    i2c_write_byte_lcd(0x30 | BL_BIT); vTaskDelay(pdMS_TO_TICKS(10)); // Function set 8-bit mode
    i2c_write_byte_lcd(0x30 | BL_BIT); vTaskDelay(pdMS_TO_TICKS(1));  // Function set 8-bit mode (again)
    i2c_write_byte_lcd(0x30 | BL_BIT); vTaskDelay(pdMS_TO_TICKS(1));  // Function set 8-bit mode (third time)
    i2c_write_byte_lcd(0x20 | BL_BIT); vTaskDelay(pdMS_TO_TICKS(1));  // Set to 4-bit mode (actual 4-bit command)

    // Function Set: 4-bit, 2-line, 5x8 dots
    lcd_send_cmd(LCD_FUNCTIONSET | LCD_4BITMODE | LCD_2LINE | LCD_5x8DOTS);
    vTaskDelay(pdMS_TO_TICKS(1)); // Short delay after function set

    // Display Control: Display ON, Cursor OFF, Blink OFF
    lcd_send_cmd(LCD_DISPLAYCONTROL | LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF);
    vTaskDelay(pdMS_TO_TICKS(1)); // Short delay after display control

    lcd_send_cmd(LCD_CLEARDISPLAY); // Clear display
    vTaskDelay(pdMS_TO_TICKS(3)); // Clear display needs longer delay (min 1.52ms)

    // Entry Mode Set: Increment cursor, no display shift
    lcd_send_cmd(LCD_ENTRYMODESET | 0x02); // 0x02 for cursor increment
    vTaskDelay(pdMS_TO_TICKS(1)); // Short delay after entry mode set

    lcd_send_cmd(LCD_RETURNHOME); // Return home
    vTaskDelay(pdMS_TO_TICKS(3)); // Return home needs longer delay (min 1.52ms)
    
    ESP_LOGI(LCD_TAG, "LCD Initialized.");
}

// Set LCD cursor position
static void lcd_set_cursor(uint8_t col, uint8_t row) {
    uint8_t row_offsets[] = {0x00, 0x40}; // For 2x16 LCD
    lcd_send_cmd(LCD_SETDDRAMADDR | (col + row_offsets[row]));
}

// Print string to LCD
static void lcd_print_string(const char *str) {
    while (*str) {
        lcd_send_data(*str++);
    }
}


// --- Rotary Encoder Task ---
static void rotary_encoder_task(void* arg) {
    uint32_t gpio_num;
    int last_state_A = gpio_get_level(ROTARY_ENCODER_PIN_A);
    int last_state_B = gpio_get_level(ROTARY_ENCODER_PIN_B);
    int last_button_state = gpio_get_level(ROTARY_ENCODER_BUTTON_PIN);
    TickType_t last_button_press_time = 0;

    while (1) {
        if (xQueueReceive(gpio_evt_queue, &gpio_num, portMAX_DELAY)) {
            // Give a small delay to allow for mechanical debounce
            vTaskDelay(pdMS_TO_TICKS(10));

            if (gpio_num == ROTARY_ENCODER_BUTTON_PIN) {
                int current_button_state = gpio_get_level(ROTARY_ENCODER_BUTTON_PIN);
                TickType_t current_time = xTaskGetTickCount();

                // Simple debounce logic for button
                if (current_button_state != last_button_state && (current_time - last_button_press_time) * portTICK_PERIOD_MS > BUTTON_DEBOUNCE_TIME_MS) {
                    if (current_button_state == 0) { // Assuming pull-up, so 0 means pressed
                        ESP_LOGI(ENCODER_TAG, "Rotary Encoder Button Pressed!");
                        g_encoder_button_state = ENC_BTN_PRESSED;
                    } else {
                        // ESP_LOGI(ENCODER_TAG, "Rotary Encoder Button Released!"); // Uncomment if you want release events
                        g_encoder_button_state = ENC_BTN_IDLE;
                    }
                    last_button_state = current_button_state;
                    last_button_press_time = current_time;
                }
            } else if (gpio_num == ROTARY_ENCODER_PIN_A || gpio_num == ROTARY_ENCODER_PIN_B) {
                int current_state_A = gpio_get_level(ROTARY_ENCODER_PIN_A);
                int current_state_B = gpio_get_level(ROTARY_ENCODER_PIN_B);

                // Basic rotary encoder logic
                if (current_state_A != last_state_A) { // Only evaluate if A changes
                    if (current_state_A == 1) { // Rising edge of A
                        if (current_state_B == 0) { // A goes high, B is low = Clockwise
                            ESP_LOGI(ENCODER_TAG, "Rotary Encoder: Clockwise");
                            g_encoder_direction = ENC_DIR_CW;
                        } else { // A goes high, B is high = Counter-Clockwise (depends on wiring/encoder)
                            ESP_LOGI(ENCODER_TAG, "Rotary Encoder: Counter-Clockwise");
                            g_encoder_direction = ENC_DIR_CCW;
                        }
                    } else { // Falling edge of A
                        if (current_state_B == 1) { // A goes low, B is high = Clockwise
                             ESP_LOGI(ENCODER_TAG, "Rotary Encoder: Clockwise");
                             g_encoder_direction = ENC_DIR_CW;
                        } else { // A goes low, B is low = Counter-Clockwise
                             ESP_LOGI(ENCODER_TAG, "Rotary Encoder: Counter-Clockwise");
                             g_encoder_direction = ENC_DIR_CCW;
                        }
                    }
                } else if (current_state_B != last_state_B) { // Only evaluate if B changes (and A didn't just change)
                    // This block handles cases where B might change first,
                    // or for robustness, but primary logic is usually A-triggered.
                    // For typical encoders, only one pin (A or B) is interrupted,
                    // and the other is read to determine direction. We interrupt both here.
                    if (current_state_B == 1) { // Rising edge of B
                        if (current_state_A == 1) { // B goes high, A is high = Clockwise
                            ESP_LOGI(ENCODER_TAG, "Rotary Encoder: Clockwise");
                            g_encoder_direction = ENC_DIR_CW;
                        } else { // B goes high, A is low = Counter-Clockwise
                            ESP_LOGI(ENCODER_TAG, "Rotary Encoder: Counter-Clockwise");
                            g_encoder_direction = ENC_DIR_CCW;
                        }
                    } else { // Falling edge of B
                        if (current_state_A == 0) { // B goes low, A is low = Clockwise
                            ESP_LOGI(ENCODER_TAG, "Rotary Encoder: Clockwise");
                            g_encoder_direction = ENC_DIR_CW;
                        } else { // B goes low, A is high = Counter-Clockwise
                            ESP_LOGI(ENCODER_TAG, "Rotary Encoder: Counter-Clockwise");
                            g_encoder_direction = ENC_DIR_CCW;
                        }
                    }
                }

                last_state_A = current_state_A;
                last_state_B = current_state_B;
            }
            // Reset direction after a short period if no further movement (optional, for "transient" display)
            // vTaskDelay(pdMS_TO_TICKS(100)); // Short delay to show movement, then reset
            // g_encoder_direction = ENC_DIR_NONE;
        }
    }
}

// --- Main Application Entry Point ---
void app_main(void) {
    // --- Initialize I2C ---
    esp_err_t err = i2c_master_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C master initialization failed, stopping.");
        return;
    }

    // --- Initialize BMP180 Sensor ---
    err = bmp180_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "BMP180 sensor initialization failed, stopping.");
        return;
    }

    // --- Initialize Rotary Encoder GPIOs and ISR ---
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_ANYEDGE; // Interrupt on both rising and falling edges for A & B
    io_conf.pin_bit_mask = (1ULL << ROTARY_ENCODER_PIN_A) | (1ULL << ROTARY_ENCODER_PIN_B);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    io_conf.intr_type = GPIO_INTR_NEGEDGE; // Interrupt on falling edge for button (assuming pull-up)
    io_conf.pin_bit_mask = (1ULL << ROTARY_ENCODER_BUTTON_PIN);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t)); // Create a queue to handle GPIO events

    // Install GPIO ISR service
    gpio_install_isr_service(0);
    // Hook ISR handler for specific GPIOs
    gpio_isr_handler_add(ROTARY_ENCODER_PIN_A, gpio_isr_handler, (void*) ROTARY_ENCODER_PIN_A);
    gpio_isr_handler_add(ROTARY_ENCODER_PIN_B, gpio_isr_handler, (void*) ROTARY_ENCODER_PIN_B);
    gpio_isr_handler_add(ROTARY_ENCODER_BUTTON_PIN, gpio_isr_handler, (void*) ROTARY_ENCODER_BUTTON_PIN);

    // Create the rotary encoder task
    xTaskCreate(&rotary_encoder_task, "rotary_encoder_task", 2048, NULL, 10, NULL);

    // --- Initialize LCD ---
    lcd_init();
    lcd_send_cmd(LCD_CLEARDISPLAY);
    vTaskDelay(pdMS_TO_TICKS(10)); // Give some time after clear

    // --- Main Loop for BMP180 Readings and LCD Updates ---
    int32_t uncomp_temp = 0, uncomp_press = 0; // Initialized to zero
    float temperature_c = 0.0f; // Initialized to zero
    long pressure_pa = 0L; // Initialized to zero
    char line1_buf[LCD_LINE_MAX_CHARS + 1]; // Buffer for first LCD line
    char line2_buf[LCD_LINE_MAX_CHARS + 1]; // Buffer for second LCD line
    const char *dir_str = "";
    const char *btn_str_short = "";

    ESP_LOGI(TAG, "Starting BMP180 sensor readings and Encoder monitoring...");

    while (1) {
        // --- BMP180 Readings ---
        uncomp_temp = bmp180_read_raw_temp();
        if (uncomp_temp == -1) {
            ESP_LOGE(TAG, "Failed to read uncompensated temperature.");
        } else {
            temperature_c = bmp180_compensate_temperature(uncomp_temp);
        }

        uncomp_press = bmp180_read_raw_pressure();
        if (uncomp_press == -1) {
            ESP_LOGE(TAG, "Failed to read uncompensated pressure.");
        } else {
            pressure_pa = bmp180_compensate_pressure(uncomp_press);
        }

        // --- Prepare LCD Line 1 (Temperature & Pressure) ---
        // Clear buffer before writing to ensure no old characters
        memset(line1_buf, ' ', LCD_LINE_MAX_CHARS);
        line1_buf[LCD_LINE_MAX_CHARS] = '\0'; // Null-terminate the buffer
        // Format: T:XX.X P:YYYYYY (Temp in C, Pressure in Pa)
        // e.g., "T:25.1 P:101325" (16 chars)
        snprintf(line1_buf, LCD_LINE_MAX_CHARS + 1, "T:%.1f P:%ld", temperature_c, pressure_pa);


        // --- Prepare LCD Line 2 (Encoder Output) ---
        // Determine encoder direction string
        if (g_encoder_direction == ENC_DIR_CW) {
            dir_str = "CW";
        } else if (g_encoder_direction == ENC_DIR_CCW) {
            dir_str = "CCW";
        } else {
            dir_str = "NONE"; // No movement
        }

        // Determine encoder button string
        if (g_encoder_button_state == ENC_BTN_PRESSED) {
            btn_str_short = "PRESSED"; // Use "PRESSED" for pressed
        } else {
            btn_str_short = "IDLE"; // Use "IDLE" for not pressed
        }
        
        // Clear buffer before writing to ensure no old characters
        memset(line2_buf, ' ', LCD_LINE_MAX_CHARS);
        line2_buf[LCD_LINE_MAX_CHARS] = '\0'; // Null-terminate the buffer
        // Format: E:DIR B:BTN (e.g., "E:CW B:PRESSED" is 14 chars, "E:NONE B:IDLE" is 13 chars)
        snprintf(line2_buf, LCD_LINE_MAX_CHARS + 1, "E:%s B:%s", dir_str, btn_str_short);
        
        // Reset encoder states after reading to show "transient" events
        g_encoder_direction = ENC_DIR_NONE;
        g_encoder_button_state = ENC_BTN_IDLE;


        // --- Update LCD Display ---
        lcd_set_cursor(0, 0); // Set cursor to start of first line
        lcd_print_string(line1_buf);
        
        // Pad with spaces to clear any leftover characters from previous longer strings
        for(int i = strlen(line1_buf); i < LCD_LINE_MAX_CHARS; i++) {
            lcd_send_data(' ');
        }

        lcd_set_cursor(0, 1); // Set cursor to start of second line
        lcd_print_string(line2_buf);

        // Pad with spaces for second line too
        for(int i = strlen(line2_buf); i < LCD_LINE_MAX_CHARS; i++) {
            lcd_send_data(' ');
        }

        ESP_LOGI(TAG, "Temperature: %.2f C, Pressure: %ld Pa", temperature_c, pressure_pa);

        vTaskDelay(pdMS_TO_TICKS(5000)); // Read sensors and update LCD every 5 seconds
    }
}
