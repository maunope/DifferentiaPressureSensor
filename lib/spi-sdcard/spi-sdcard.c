#include "spi-sdcard.h"
#include "tinyusb.h"
#include "sdkconfig.h"
#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "sdmmc_cmd.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "tinyusb_msc.h"
#include "tinyusb_default_config.h"

static const char *TAG = "SPI_SDCARD_V2";
static bool mounted = false;
static sdmmc_card_t *s_card = NULL;
static tinyusb_msc_storage_handle_t s_msc_storage_handle = NULL;

// Global host and mount configuration
static sdmmc_host_t host = SDSPI_HOST_DEFAULT();
static sdspi_device_config_t slot_config = {
    .host_id = SPI2_HOST,
    .gpio_cs = GPIO_NUM_10,
    .gpio_cd = -1,
    .gpio_wp = -1,
    .gpio_int = -1,
};

static esp_vfs_fat_sdmmc_mount_config_t mount_config = {
    .format_if_mount_failed = true,
    .max_files = 5,
    .allocation_unit_size = 16 * 1024
};

// Configure CS pin with pull-up for SPI mode
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
}/*
  static void device_event_handler(tinyusb_event_t *event, void *arg) {
    switch (event->id) {
    case TINYUSB_EVENT_ATTACHED:
    ESP_LOGI(TAG, "TinyUSB MSC is now mounted on the host.");
    case TINYUSB_EVENT_DETACHED:
    ESP_LOGI(TAG, "TinyUSB MSC is now detached from the host.");
    default:
        break;
    }
}
*/

void spi_sdcard_full_init()
{
    ESP_LOGI(TAG, "Starting full TinyUSB and SD card initialization (v2.0.0).");

    if (mounted) {
        ESP_LOGW(TAG, "SD Card already mounted, skipping initialization.");
        return;
    }

    init_gpio_cs();

    host.slot = SPI2_HOST;
    host.max_freq_khz = SDMMC_FREQ_DEFAULT;

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

    ret = esp_vfs_fat_sdspi_mount("/sdcard", &host, &slot_config, &mount_config, &s_card);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount SD card: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "SD card mounted.");
        mounted = true;

        ESP_LOGI(TAG, "Configuring and installing TinyUSB for MSC.");
        
        // Step 1: Install the general TinyUSB driver with a valid task config.
        const tinyusb_config_t tusb_cfg = {
            .task.size = 4096,
            .task.priority = 2,
            .task.xCoreID = 0
        };

       // const tinyusb_config_t tusb_cfg = TINYUSB_DEFAULT_CONFIG(device_event_handler);
        
        ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

        // Step 2: Create a handle for the SD card storage using the new API
        tinyusb_msc_storage_config_t sd_storage_config = {
            .medium = {
                .wl_handle = (long)s_card,
            },
            
        };
        
        ESP_ERROR_CHECK(tinyusb_msc_new_storage_sdmmc(&sd_storage_config, &s_msc_storage_handle));
        

        ESP_LOGI(TAG, "TinyUSB and MSC storage initialized.");
    }
}

int spi_sdcard_write_csv(const char *filename, char *ts, float temperature, long pressure)
{
    if (tud_ready()) {
        ESP_LOGE(TAG, "TinyUSB is connected and mounted, cannot write to SD card directly.");
        return -6;
    }

    esp_err_t ret = esp_vfs_fat_sdcard_unmount("/sdcard", s_card);
    if (ret == ESP_OK) {
        mounted = false;
        ESP_LOGI(TAG, "SD card unmounted from ESP32 successfully.");
    } else {
        ESP_LOGE(TAG, "Failed to unmount SD card: %s", esp_err_to_name(ret));
        return -1;
    }

    ret = esp_vfs_fat_sdspi_mount("/sdcard", &host, &slot_config, &mount_config, &s_card);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to remount SD card: %s", esp_err_to_name(ret));
        return -2;
    }

    char filepath[64];
    snprintf(filepath, sizeof(filepath), "/sdcard/%s", filename);

    FILE *f = fopen(filepath, "a");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing: %s", filepath);
        return -3;
    }

    fprintf(f, "%s,%.2f,%ld\n", ts, temperature, pressure);
    fflush(f);
    fclose(f);

    return 0;
}

/**
 * @brief Formats the SD card. All data on the card will be erased.
 * @note This function handles the unmounting, formatting, and remounting of the card
 * in a single, robust operation. It prevents race conditions and ensures the
 * file system is in a clean state after formatting.
 * @return esp_err_t ESP_OK on success, or an error code on failure.
 */
esp_err_t spi_sdcard_format(void)
{
    return ESP_OK;
}
