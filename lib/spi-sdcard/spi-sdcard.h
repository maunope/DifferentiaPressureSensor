#pragma once

#include <stdint.h>
#include <stdbool.h>


#define SPI_CARD_ERROR_NOT_MOUNTED -1
#define SPI_CARD_ERROR_FILE_OPEN   -2
#define SPI_CARD_OK                 0


void spi_sdcard_full_init();
void spi_sdcard_deinit(void);
void spi_sdcard_write_csv();
void spi_sdcard_format(void);

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