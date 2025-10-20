#include "i2c-oled.h"
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

// Use font5x7 from header
extern const uint8_t font5x7[96][5];

// Store the address for the single OLED device this library manages.
static uint8_t s_oled_addr = 0;

// Screen buffer for 128x64 OLED
#define OLED_WIDTH 128
#define OLED_HEIGHT 64
static uint8_t s_screen_buffer[OLED_WIDTH * OLED_HEIGHT / 8];

// Forward declaration for the static helper function
static esp_err_t oled_data(i2c_port_t i2c_num, const uint8_t *data, size_t len);

/**
 * @brief Sends a command byte to the OLED controller.
 *
 * @param i2c_num The I2C port number.
 * @param cmd The command byte to send.
 * @return esp_err_t `ESP_OK` on success, or an I2C error code on failure.
 */
static esp_err_t oled_cmd(i2c_port_t i2c_num, uint8_t cmd) {
    uint8_t buf[2] = {0x00, cmd};
    return i2c_master_write_to_device(i2c_num, s_oled_addr, buf, 2, 1000 / portTICK_PERIOD_MS);
}

/**
 * @brief Draws a single pixel into the screen buffer.
 */
void i2c_oled_draw_pixel(i2c_port_t i2c_num, int x, int y, bool on) {
    if (x < 0 || x >= OLED_WIDTH || y < 0 || y >= OLED_HEIGHT) return;

    if (on) {
        // Set the bit in the buffer
        s_screen_buffer[x + (y / 8) * OLED_WIDTH] |= (1 << (y % 8));
    } else {
        // Clear the bit in the buffer
        s_screen_buffer[x + (y / 8) * OLED_WIDTH] &= ~(1 << (y % 8));
    }
}

void i2c_oled_draw_rect(i2c_port_t i2c_num, int x, int y, int width, int height, bool on) {
    // Draw horizontal lines
    for (int i = 0; i < width; i++) {
        i2c_oled_draw_pixel(i2c_num, x + i, y, on);
        i2c_oled_draw_pixel(i2c_num, x + i, y + height - 1, on);
    }
    // Draw vertical lines
    for (int i = 0; i < height; i++) {
        i2c_oled_draw_pixel(i2c_num, x, y + i, on);
        i2c_oled_draw_pixel(i2c_num, x + width - 1, y + i, on);
    }
}

void i2c_oled_write_char(i2c_port_t i2c_num, uint8_t row, uint8_t col, char c) {
    char str[2] = {c, '\0'};
    i2c_oled_write_text(i2c_num, row, col, str);
}


void i2c_oled_fill_rect(i2c_port_t i2c_num, int x, int y, int width, int height, bool on) {
    for (int i = 0; i < width; i++) {
        for (int j = 0; j < height; j++) {
            i2c_oled_draw_pixel(i2c_num, x + i, y + j, on);
        }
    }
}
/**
 * @brief Sends a buffer of data bytes to the OLED GDDRAM.
 *
 * @param i2c_num The I2C port number.
 * @param data Pointer to the data buffer to send.
 * @param len The number of bytes to send.
 * @return esp_err_t `ESP_OK` on success, or an I2C error code on failure.
 */
static esp_err_t oled_data(i2c_port_t i2c_num, const uint8_t *data, size_t len) {
    uint8_t buf[len + 1];
    buf[0] = 0x40;
    memcpy(&buf[1], data, len);
    return i2c_master_write_to_device(i2c_num, s_oled_addr, buf, len + 1, 1000 / portTICK_PERIOD_MS);
}

/**
 * @brief Initializes the I2C bus for the OLED display.
 *
 * Configures and installs the I2C driver for the specified port and pins,
 * then sends the initialization sequence to the OLED controller.
 * @param i2c_num The I2C port number to use.
 * @param sda The GPIO number for the SDA line.
 * @param scl The GPIO number for the SCL line.
 * @param i2c_addr The I2C address of the OLED display.
 */
void i2c_oled_bus_init(i2c_port_t i2c_num, gpio_num_t sda, gpio_num_t scl, uint8_t i2c_addr) {
    if (i2c_addr == 0) {
        s_oled_addr = OLED_I2C_ADDR; // Use default if 0 is passed
    } else {
        s_oled_addr = i2c_addr;
    }
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
    i2c_oled_send_init_commands(i2c_num);
}

/**
 * @brief Sends the initialization command sequence to the SSD1306 controller.
 *
 * @param i2c_num The I2C port number.
 */
void i2c_oled_send_init_commands(i2c_port_t i2c_num) {
    // SSD1306 init sequence
    oled_cmd(i2c_num, 0xAE); // Display off
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

/**
 * @brief Clears the entire OLED display RAM.
 *
 * @param i2c_num The I2C port number.
 */
void i2c_oled_clear(i2c_port_t i2c_num)
{
    memset(s_screen_buffer, 0, sizeof(s_screen_buffer));
}

/**
 * @brief Sends the entire screen buffer to the OLED display.
 *
 * @param i2c_num The I2C port number.
 */
void i2c_oled_update_screen(i2c_port_t i2c_num)
{
    for (uint8_t page = 0; page < (OLED_HEIGHT / 8); page++)
    {
        oled_cmd(i2c_num, 0xB0 | page); // Set page
        oled_cmd(i2c_num, 0x00);        // Set lower column address
        oled_cmd(i2c_num, 0x10);        // Set higher column address

        // Send 128 bytes of page data
        uint8_t *page_data = &s_screen_buffer[page * OLED_WIDTH];
        oled_data(i2c_num, page_data, OLED_WIDTH);
    }
}

/**
 * @brief Writes a string of text to the OLED display.
 *
 * @param i2c_num The I2C port number.
 * @param row The row (page) to write to (0-7).
 * @param col The column to start writing at (0-20).
 * @param text The null-terminated string to write.
 */
void i2c_oled_write_text(i2c_port_t i2c_num, uint8_t row, uint8_t col, const char *text) {
    uint16_t x = col * 6;
    uint16_t y = row * 8;
    while (*text) {
        if (*text < 32 || *text > 127) {
            text++;
            continue;
        }
        for (int i = 0; i < 5; i++) {
            s_screen_buffer[x + i + y * OLED_WIDTH / 8] = font5x7[*text - 32][i];
        }
        x += 6; // 5 for char, 1 for space
        text++;
    }
}

/**
 * @brief Sets the display to normal or inverted mode.
 *
 * @param i2c_num The I2C port number.
 * @param invert `true` for inverted display (white on black), `false` for normal (black on white).
 */
void i2c_oled_set_invert(i2c_port_t i2c_num, bool invert) {
    // 0xA6 is normal, 0xA7 is inverted
    oled_cmd(i2c_num, invert ? 0xA7 : 0xA6);
}

/**
 * @brief Writes a full line of inverted text (e.g., for a title bar).
 *
 * This function creates a full-width inverted bar and "cuts out" the text characters.
 * @param i2c_num The I2C port number.
 * @param row The row (page) to write to (0-7).
 * @param col The starting column (typically 0).
 * @param text The null-terminated string to write.
 */
void i2c_oled_write_inverted_text(i2c_port_t i2c_num, uint8_t row, uint8_t col, const char *text) {
    uint8_t line_buffer[128];
    memset(line_buffer, 0xFF, sizeof(line_buffer)); // Fill with 1s for an inverted background

    uint8_t current_col = col * 6;

    while (*text && current_col < 128) {
        if (*text >= 32 && *text <= 127) {
            const uint8_t* font_char = font5x7[*text - 32];
            for (int i = 0; i < 5 && (current_col + i) < OLED_WIDTH; i++) {
                line_buffer[current_col + i] = ~font_char[i]; // Invert font bits
            }
            current_col += 6; // 5 for char, 1 for space
        }
        text++;
    }
    memcpy(&s_screen_buffer[row * OLED_WIDTH], line_buffer, sizeof(line_buffer));
}

void i2c_oled_get_buffer(uint8_t* out_buffer, size_t len) {
    if (out_buffer && len >= sizeof(s_screen_buffer)) {
        memcpy(out_buffer, s_screen_buffer, sizeof(s_screen_buffer));
    }
}

void i2c_oled_load_buffer(const uint8_t* in_buffer, size_t len) {
    if (in_buffer && len >= sizeof(s_screen_buffer)) {
        memcpy(s_screen_buffer, in_buffer, sizeof(s_screen_buffer));
    }
}