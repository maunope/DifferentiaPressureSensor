#include "spi-sdcard.h"
#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "sdmmc_cmd.h"

static const char *TAG = "SPI_SDCARD";

// Configure CS pin with pull-up
static void init_gpio_cs()
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GPIO_NUM_10),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
}

static bool mounted = false;

void spi_sdcard_init()
{

    ESP_LOGI(TAG, "Starting SD Card Initialization");
    if (mounted) 
    {
        ESP_LOGW(TAG, "SD Card already mounted, skipping initialization.");
        return;
    }

    init_gpio_cs();

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SPI2_HOST;
    host.max_freq_khz = SDMMC_FREQ_DEFAULT;

    

    sdspi_device_config_t slot_config = {
        .host_id = host.slot,
        .gpio_cs = GPIO_NUM_10,
        .gpio_cd = -1,
        .gpio_wp = -1,
        .gpio_int = -1,
    };

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = GPIO_NUM_11,
        .miso_io_num = GPIO_NUM_13,
        .sclk_io_num = GPIO_NUM_12,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000
    };

    esp_err_t ret = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return;
    }

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024 //IMPORTANT, if the SD card was formatted with a different allocation unit size, it won't mount, TODO make resilient to this kind of error
    
    };
    sdmmc_card_t *card;
    ret = esp_vfs_fat_sdspi_mount("/sdcard", &host, &slot_config, &mount_config, &card);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount SD card: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "SD card mounted");
        mounted = true;
    }
}

int spi_sdcard_write_csv(const char *filename, char * ts, float temperature, long pressure)
{
    if (!mounted) {
            ESP_LOGE(TAG, "SD card not mounted, cannot write.");
            return SPI_CARD_ERROR_NOT_MOUNTED;
    }

    char filepath[64];
    snprintf(filepath, sizeof(filepath), "/sdcard/%s", filename);

    FILE *f = fopen(filepath, "a");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing: %s", filepath);
        return SPI_CARD_ERROR_FILE_OPEN;
    }

    // Write CSV line: ms,temperature,pressure
    fprintf(f, "%s,%.2f,%ld\n", ts, temperature, pressure);

    fclose(f);
    return SPI_CARD_OK;
}

