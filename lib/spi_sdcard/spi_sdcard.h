#pragma once

#include <stdint.h>
#include "esp_err.h"
#include <stdbool.h>

#define SPI_CARD_ERROR_NOT_MOUNTED -1
#define SPI_CARD_ERROR_FILE_OPEN   -2
#define SPI_CARD_OK                 0

/**
 * @brief Configuration structure for SPI SD card pins.
 */
typedef struct {
    int mosi_io_num;
    int miso_io_num;
    int sclk_io_num;
    int cs_io_num;
} spi_sdcard_config_t;

void spi_sdcard_full_init(const spi_sdcard_config_t *config);
void spi_sdcard_init_sd_only(const spi_sdcard_config_t *config);
void spi_sdcard_deinit(void);
void spi_sdcard_rotate_file(void);


/**
 * @brief Writes a line of text to the current log file on the SD card.
 *
 * Handles file creation and rotation automatically.
 * @param line The null-terminated string to write to the file.
 * @param is_hf_mode True if in high-frequency mode, which affects filename.
 */
esp_err_t spi_sdcard_write_line(const char* line, bool is_hf_mode);

/**
 * @brief Counts the number of files on the SD card and updates the shared buffer.
 */
void spi_sdcard_get_file_count(void);

/**
 * @brief Gets the free space on the SD card in MB and updates the shared buffer.
 */
void spi_sdcard_get_free_space_mb(void);

/**
 * @brief Checks if the USB Mass Storage is connected and ready.
 * @return true if USB is connected and mounted by the host, false otherwise.
 */
bool spi_sdcard_is_usb_connected(void);

/**
 * @brief Sets the header line to be written to new CSV files.
 *
 * @param header The null-terminated string to use as the CSV header.
 */
void spi_sdcard_set_csv_header(const char* header);

/**
 * @brief Deletes a file from the SD card.
 *
 * @param path Full path of the file to delete.
 * @return esp_err_t ESP_OK on success, or an error code on failure.
 */
esp_err_t spi_sdcard_delete_file(const char *path);

/**
 * @brief Formats the SD card.
 * @return esp_err_t ESP_OK on success, or an error code on failure.
 */
esp_err_t spi_sdcard_format(void);
