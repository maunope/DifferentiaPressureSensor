#pragma once

#include <stdint.h>


#define SPI_CARD_ERROR_NOT_MOUNTED -1
#define SPI_CARD_ERROR_FILE_OPEN   -2
#define SPI_CARD_OK                 0


void spi_sdcard_full_init();
void spi_sdcard_write_csv(const char *filename);
void spi_sdcard_format(void);