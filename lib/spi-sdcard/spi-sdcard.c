#include "spi-sdcard.h"
#include "tinyusb.h"
#include "tusb_msc_storage.h"
#include "tinyusb_config.h" // Required for TinyUSB configuration struct
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

static const char *TAG = "SPI_SDCARD";
static bool mounted = false;
static sdmmc_card_t *s_card = NULL;
static bool esp32_access_enabled = true; // New global state flag
static SemaphoreHandle_t s_sd_card_mutex; // Mutex for thread-safe access

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
}

// Callback for when the PC mounts or unmounts the disk
static void msc_mount_change_cb(tinyusb_msc_event_t *event)
{
    xSemaphoreTake(s_sd_card_mutex, portMAX_DELAY);

    if (event->mount_changed_data.is_mounted) {
        // PC has mounted the disk, disable ESP32 access
        ESP_LOGI(TAG, "PC has mounted the disk. Unmounting SD card from ESP32.");
        // Add a small delay to ensure all pending writes from the PC are processed by TinyUSB
        vTaskDelay(pdMS_TO_TICKS(200));
        
        if (mounted) {
            esp_err_t ret = esp_vfs_fat_sdcard_unmount("/sdcard", s_card);
            if (ret == ESP_OK) {
                mounted = false;
                esp32_access_enabled = false;
                ESP_LOGI(TAG, "SD card unmounted from ESP32 successfully.");
            } else {
                ESP_LOGE(TAG, "Failed to unmount SD card: %s", esp_err_to_name(ret));
            }
        }
    } else {
        // PC has unmounted the disk, re-enable ESP32 access
        ESP_LOGI(TAG, "PC has unmounted the disk. Remounting SD card for ESP32.");
        // Add a small delay to ensure the PC has fully released the drive
        vTaskDelay(pdMS_TO_TICKS(200));

        esp_err_t ret = esp_vfs_fat_sdspi_mount("/sdcard", &host, &slot_config, &mount_config, &s_card);
        if (ret == ESP_OK) {
            mounted = true;
            esp32_access_enabled = true;
            ESP_LOGI(TAG, "SD card remounted successfully.");
        } else {
            ESP_LOGE(TAG, "Failed to remount SD card: %s", esp_err_to_name(ret));
        }
    }

    xSemaphoreGive(s_sd_card_mutex);
}

// Internal function to initialize the SD card and its file system
static void spi_sdcard_internal_init()
{
  
}

void spi_sdcard_full_init()
{
    ESP_LOGI(TAG, "Starting full TinyUSB and SD card initialization.");
    
    // Create the mutex before any other tasks can try to use it
    s_sd_card_mutex = xSemaphoreCreateMutex();
    if (s_sd_card_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex.");
        return;
    }

    // TinyUSB configuration
    const tinyusb_config_t tusb_cfg = {
        .device_descriptor = NULL,
        .string_descriptor = NULL,
        .external_phy = false, // In the ESP32-S3, the USB PHY is integrated
        .configuration_descriptor = NULL,
    };

    //this methods takes exclusive use of pins 19 and 20 (usually, of two pins anyway), 
    //be sure they are not used by anything else
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
    ESP_LOGI(TAG, "TinyUSB installed");

    // Initialize the SD card after TinyUSB is installed
      ESP_LOGI(TAG, "Starting SD Card Initialization");
    if (mounted) 
    {
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
        ESP_LOGI(TAG, "SD card mounted");
        mounted = true;
        
        ESP_LOGI(TAG, "Initializing TinyUSB MSC storage with SD card.");
        
        tinyusb_msc_sdmmc_config_t msc_cfg = {
            .card = s_card,
        };
        tinyusb_msc_storage_init_sdmmc(&msc_cfg);
        
        // Register the callback for the mount/unmount event
        tinyusb_msc_register_callback(TINYUSB_MSC_EVENT_MOUNT_CHANGED, msc_mount_change_cb);
        
        ESP_LOGI(TAG, "TinyUSB MSC storage initialized.");
    }
}

int spi_sdcard_write_csv(const char *filename, char * ts, float temperature, long pressure)
{
    // Take the mutex before any file operation
    if (xSemaphoreTake(s_sd_card_mutex, portMAX_DELAY) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to acquire mutex.");
        return -4;
    }

    if (!esp32_access_enabled) {
        ESP_LOGW(TAG, "SD card is mounted by USB host, skipping write.");
        xSemaphoreGive(s_sd_card_mutex); // Release mutex before returning
        return -3; // Return a new error code for this state
    }

    char filepath[64];
    snprintf(filepath, sizeof(filepath), "/sdcard/%s", filename);

    FILE *f = fopen(filepath, "a");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing: %s", filepath);
        xSemaphoreGive(s_sd_card_mutex); // Release mutex on failure
        return -2;
    }

    // Write CSV line: ms,temperature,pressure
    fprintf(f, "%s,%.2f,%ld\n", ts, temperature, pressure);

    fclose(f);
    
    // Give the mutex back after all file operations are complete
    xSemaphoreGive(s_sd_card_mutex);
    
    return 0;
}


int spi_sdcard_format(void)
{
    int ret = ESP_OK;

    // Acquire the mutex to ensure no other file operations are in progress
    if (xSemaphoreTake(s_sd_card_mutex, portMAX_DELAY) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to acquire mutex for formatting.");
        return ESP_FAIL;
    }
    
    ESP_LOGW(TAG, "Formatting the SD card. All data will be erased!");

    // Check if the card is currently mounted
    if (mounted) {
        // Unmount the card before formatting
        ret = esp_vfs_fat_sdcard_unmount("/sdcard", s_card);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to unmount SD card before formatting: %s", esp_err_to_name(ret));
            xSemaphoreGive(s_sd_card_mutex);
            return ret;
        }
        mounted = false;
        esp32_access_enabled = false;
        ESP_LOGI(TAG, "SD card unmounted for formatting.");
    }
    
    // Format the SD card
    ret = esp_vfs_fat_sdcard_format("/sdcard", s_card);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to format SD card: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "SD card formatted successfully.");
    }
    
    // Re-mount the SD card after formatting
    esp_err_t mount_ret = esp_vfs_fat_sdspi_mount("/sdcard", &host, &slot_config, &mount_config, &s_card);
    if (mount_ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to remount SD card after formatting: %s", esp_err_to_name(mount_ret));
        ret = mount_ret; // Return the new error if remount fails
    } else {
        mounted = true;
        esp32_access_enabled = true;
        ESP_LOGI(TAG, "SD card remounted successfully after formatting.");
    }

    // Release the mutex
    xSemaphoreGive(s_sd_card_mutex);

    return ret;
}
