#include "spi-sdcard.h"
#include "tinyusb.h"
#include "tusb_msc_storage.h"
#include "sdkconfig.h" // Required for configuration macros
#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "driver/sdspi_host.h" // Use the SDSPI host driver
#include "driver/spi_common.h"
#include "sdmmc_cmd.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h" // Required for vTaskDelay
#include "class/msc/msc.h" // Required for TinyUSB Mass Storage Class functions


static const char *TAG = "SPI_SDCARD";
static bool mounted = false;
static sdmmc_card_t *s_card = NULL;

// Global host and mount configuration for use in the event callback
// Using SDSPI host and slot configuration for SPI mode
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
    .allocation_unit_size = 16 * 1024};

// Configure CS pin with pull-up for SPI mode
static void init_gpio_cs()
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GPIO_NUM_10),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&io_conf);
}


void msc_keep_alive_cb(tinyusb_msc_event_t *event)
{
    ESP_LOGD(TAG, "USB keep-alive signal detected.");
}



void spi_sdcard_full_init()
{



    ESP_LOGI(TAG, "Starting full TinyUSB and SD card initialization.");

    // TinyUSB configuration
    const tinyusb_config_t tusb_cfg = {
        .device_descriptor = NULL,
        .string_descriptor = NULL,
        .external_phy = false, // In the ESP32-S3, the USB PHY is integrated
        .configuration_descriptor = NULL,
    };

    // Step 1: Install the TinyUSB driver.
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
    ESP_LOGI(TAG, "TinyUSB installed");

    // Check if SD card is already mounted
    if (mounted)
    {
        ESP_LOGW(TAG, "SD Card already mounted, skipping initialization.");
        return;
    }

    init_gpio_cs();

    host.slot = SPI2_HOST;
    // Set frequency to default for better signal integrity and reliability.
    host.max_freq_khz = SDMMC_FREQ_PROBING;

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = GPIO_NUM_11,
        .miso_io_num = GPIO_NUM_13,
        .sclk_io_num = GPIO_NUM_12,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000};

    esp_err_t ret = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE)
    {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return;
    }

    // Step 2: Mount the SD card. This populates the `s_card` pointer.
    ret = esp_vfs_fat_sdspi_mount("/sdcard", &host, &slot_config, &mount_config, &s_card);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to mount SD card: %s", esp_err_to_name(ret));
    }
    else
    {
        ESP_LOGI(TAG, "SD card mounted");
        mounted = true;

        ESP_LOGI(TAG, "Initializing TinyUSB MSC storage with SD card.");

        // Step 3: Initialize the TinyUSB MSC storage with the now-valid card object.
        tinyusb_msc_sdmmc_config_t msc_cfg = {
            .card = s_card,
        };
        tinyusb_msc_storage_init_sdmmc(&msc_cfg);

        ESP_LOGI(TAG, "TinyUSB MSC storage initialized.");
    }


}

int spi_sdcard_write_csv(const char *filename, char *ts, float temperature, long pressure)
{
    if (tud_ready())
    {
        return -6;
    }

    
    esp_err_t ret = esp_vfs_fat_sdcard_unmount("/sdcard", s_card);
    if (ret == ESP_OK)
    {
        mounted = false;
        ESP_LOGI(TAG, "SD card unmounted from ESP32 successfully.");
    }
    else
    {
        ESP_LOGE(TAG, "Failed to unmount SD card: %s", esp_err_to_name(ret));
    }

     ret = esp_vfs_fat_sdspi_mount("/sdcard", &host, &slot_config, &mount_config, &s_card);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to mount SD card: %s", esp_err_to_name(ret));
        return ret;
    }

    char filepath[64];
    snprintf(filepath, sizeof(filepath), "/sdcard/%s", filename);

    FILE *f = fopen(filepath, "a");
    if (f == NULL)
    {
        ESP_LOGE(TAG, "Failed to open file for writing: %s", filepath);
        return -2;
    }

    // Write CSV line: ms,temperature,pressure
    fprintf(f, "%s,%.2f,%ld\n", ts, temperature, pressure);

    // Explicitly flush the C stream buffer to the file system to ensure data is written.
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
    // The functionality of this method has been commented out as per user request.
    // It is not needed for the current task and has been temporarily disabled.
    return ESP_OK;
}
