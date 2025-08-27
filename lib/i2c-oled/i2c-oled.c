#include "i2c-oled.h"
#include <string.h>
#include "esp_log.h"

// Use font5x7 from header
extern const uint8_t font5x7[96][5];

// Send command to OLED
static esp_err_t oled_cmd(i2c_port_t i2c_num, uint8_t cmd) {
    uint8_t buf[2] = {0x00, cmd};
    return i2c_master_write_to_device(i2c_num, OLED_I2C_ADDR, buf, 2, 1000 / portTICK_PERIOD_MS);
}

// Send data to OLED
static esp_err_t oled_data(i2c_port_t i2c_num, const uint8_t *data, size_t len) {
    uint8_t buf[len + 1];
    buf[0] = 0x40;
    memcpy(&buf[1], data, len);
    return i2c_master_write_to_device(i2c_num, OLED_I2C_ADDR, buf, len + 1, 1000 / portTICK_PERIOD_MS);
}

void i2c_oled_init(i2c_port_t i2c_num, gpio_num_t sda, gpio_num_t scl) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda,
        .scl_io_num = scl,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000
    };
    i2c_param_config(i2c_num, &conf);
    i2c_driver_install(i2c_num, conf.mode, 0, 0, 0);

    // SSD1306 init sequence
    oled_cmd(i2c_num, 0xAE); // Display off
    oled_cmd(i2c_num, 0x20); oled_cmd(i2c_num, 0x00); // Horizontal addressing
    oled_cmd(i2c_num, 0xB0); // Page 0
    oled_cmd(i2c_num, 0xC8); // COM scan direction
    oled_cmd(i2c_num, 0x00); // Low column
    oled_cmd(i2c_num, 0x10); // High column
    oled_cmd(i2c_num, 0x40); // Start line
    oled_cmd(i2c_num, 0x81); oled_cmd(i2c_num, 0xFF); // Contrast
    oled_cmd(i2c_num, 0xA1); // Segment remap
    oled_cmd(i2c_num, 0xA6); // Normal display
    oled_cmd(i2c_num, 0xA8); oled_cmd(i2c_num, 0x3F); // Multiplex ratio
    oled_cmd(i2c_num, 0xA4); // Output follows RAM
    oled_cmd(i2c_num, 0xD3); oled_cmd(i2c_num, 0x00); // Display offset
    oled_cmd(i2c_num, 0xD5); oled_cmd(i2c_num, 0xF0); // Clock divide
    oled_cmd(i2c_num, 0xD9); oled_cmd(i2c_num, 0x22); // Pre-charge
    oled_cmd(i2c_num, 0xDA); oled_cmd(i2c_num, 0x12); // COM pins
    oled_cmd(i2c_num, 0xDB); oled_cmd(i2c_num, 0x20); // VCOMH
    oled_cmd(i2c_num, 0x8D); oled_cmd(i2c_num, 0x14); // Charge pump
    oled_cmd(i2c_num, 0xAF); // Display on
}

void i2c_oled_clear(i2c_port_t i2c_num) {
    for (uint8_t page = 0; page < 8; page++) {
        oled_cmd(i2c_num, 0xB0 | page);
        oled_cmd(i2c_num, 0x00);
        oled_cmd(i2c_num, 0x10);
        uint8_t zeros[128] = {0};
        oled_data(i2c_num, zeros, 128);
    }
}

void i2c_oled_write_text(i2c_port_t i2c_num, uint8_t row, uint8_t col, const char *text) {
    oled_cmd(i2c_num, 0xB0 | row); // Set page address
    oled_cmd(i2c_num, 0x00 | (col * 6 & 0x0F)); // Set lower column
    oled_cmd(i2c_num, 0x10 | ((col * 6 >> 4) & 0x0F)); // Set higher column

    while (*text) {
        if (*text < 32 || *text > 127) {
            text++;
            continue;
        }
        oled_data(i2c_num, font5x7[*text - 32], 5);
        uint8_t space = 0x00;
        oled_data(i2c_num, &space, 1);
        text++;
    }
}