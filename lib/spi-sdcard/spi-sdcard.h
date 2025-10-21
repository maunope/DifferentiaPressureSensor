#pragma once

#include <stdint.h>
#include "esp_err.h"
#include <stdbool.h>

#define SPI_CARD_ERROR_NOT_MOUNTED -1
#define SPI_CARD_ERROR_FILE_OPEN   -2
#define SPI_CARD_OK                 0

void spi_sdcard_full_init(void);
void spi_sdcard_init_sd_only(void);
void spi_sdcard_deinit(void);
void spi_sdcard_format(void);
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
