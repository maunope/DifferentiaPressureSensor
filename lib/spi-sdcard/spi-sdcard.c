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
#include "../buffers.h"

#include <sys/stat.h>
#include <dirent.h>

static const char *TAG = "SPI_SDCARD_V2";
static bool mounted = false;
static sdmmc_card_t *s_card = NULL;
static tinyusb_msc_storage_handle_t s_msc_storage_handle = NULL;
static char current_filepath[264] = ""; // 8 for "/sdcard/" + 255 for d_name + 1 for null
#define MAX_FILE_SIZE_BYTES (10 * 1024 * 1024) // 10 MB


// Global host and mount configuration
static sdmmc_host_t host = SDSPI_HOST_DEFAULT();
static sdspi_device_config_t slot_config = {
    .host_id = SPI2_HOST,
    .gpio_cs = GPIO_NUM_13,
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
        .pin_bit_mask = (1ULL << GPIO_NUM_13),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&io_conf);
} /*
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

    if (mounted)
    {
        ESP_LOGW(TAG, "SD Card already mounted, skipping initialization.");
        return;
    }

    init_gpio_cs();

    host.slot = SPI2_HOST;
    host.max_freq_khz = SDMMC_FREQ_DEFAULT;
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = GPIO_NUM_12,
        .miso_io_num = GPIO_NUM_10,
        .sclk_io_num = GPIO_NUM_11,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 16384 // Increased DMA transfer size to 16384 bytes for better performance with large files
    };

    esp_err_t ret = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE)
    {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_vfs_fat_sdspi_mount("/sdcard", &host, &slot_config, &mount_config, &s_card);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to mount SD card: %s", esp_err_to_name(ret));
    }
    else
    {
        ESP_LOGI(TAG, "SD card mounted.");
        mounted = true;

        ESP_LOGI(TAG, "Configuring and installing TinyUSB for MSC.");

        // Step 1: Install the general TinyUSB driver with a valid task config.
        const tinyusb_config_t tusb_cfg = {
            .task.size = 16384 * 2,
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

// todo refactor to update struct
void spi_sdcard_write_csv()
{
    //TODO make all erro codes defines

    if (tud_ready())
    {
        ESP_LOGI(TAG, "TinyUSB is connected and mounted, cannot write to SD card directly.");
        if (g_sensor_buffer_mutex && xSemaphoreTake(g_sensor_buffer_mutex, pdMS_TO_TICKS(50)) == pdTRUE)
        {
            g_sensor_buffer.writeStatus = -6; // Indicate USB is connected
            xSemaphoreGive(g_sensor_buffer_mutex);
        }
        else
        {
            ESP_LOGE(TAG, "Failed to acquire mutex to update writeStatus.");
        }
        return;
    }

    if (!mounted) {
        ESP_LOGE(TAG, "SD card not mounted, cannot write.");
         if (g_sensor_buffer_mutex && xSemaphoreTake(g_sensor_buffer_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            g_sensor_buffer.writeStatus = -1; // Indicate mount failure
            xSemaphoreGive(g_sensor_buffer_mutex);
        }
        return;
    }

    sensor_buffer_t local_buffer;

    // Acquire mutex and copy buffer
    if (g_sensor_buffer_mutex && xSemaphoreTake(g_sensor_buffer_mutex, pdMS_TO_TICKS(50)) == pdTRUE)
    {
        local_buffer = g_sensor_buffer;
        xSemaphoreGive(g_sensor_buffer_mutex);
    }
    else
    {
        ESP_LOGE(TAG, "Failed to acquire mutex to read sensor buffer.");
        return;
    }

    // --- File Rotation Logic ---
    struct stat st;
    if (current_filepath[0] == '\0') {
        // First run, find the latest file
        DIR *dir = opendir("/sdcard");
        if (dir) {
            struct dirent *ent;
            long long latest_ts = 0;
            char latest_file[sizeof(current_filepath)] = "";

            while ((ent = readdir(dir)) != NULL) {
                long long ts;
                if (sscanf(ent->d_name, "data_%lld.csv", &ts) == 1) {
                    if (ts > latest_ts) {
                        latest_ts = ts;
                        snprintf(latest_file, sizeof(latest_file), "/sdcard/%.255s", ent->d_name);
                    }
                }
            }
            closedir(dir);

            if (latest_ts > 0) {
                // Found a previous file, check its size
                if (stat(latest_file, &st) == 0 && st.st_size < MAX_FILE_SIZE_BYTES) {
                    ESP_LOGI(TAG, "Continuing with existing log file: %s", latest_file);
                    strncpy(current_filepath, latest_file, sizeof(current_filepath));
                }
            }
        }
    }

    if (current_filepath[0] == '\0' || (stat(current_filepath, &st) == 0 && st.st_size >= MAX_FILE_SIZE_BYTES))
    {
        // Create a new filename based on the current timestamp
        snprintf(current_filepath, sizeof(current_filepath), "/sdcard/data_%lld.csv", (long long)local_buffer.timestamp);
        ESP_LOGI(TAG, "Starting new log file: %s", current_filepath);
    }

    FILE *f = fopen(current_filepath, "a");
    if (f == NULL)
    {
        ESP_LOGE(TAG, "Failed to open file for writing: %s", current_filepath);
        if (g_sensor_buffer_mutex && xSemaphoreTake(g_sensor_buffer_mutex, pdMS_TO_TICKS(50)) == pdTRUE)
        {
            g_sensor_buffer.writeStatus = -3; // Indicate file open failure
            xSemaphoreGive(g_sensor_buffer_mutex);
            return;
        }
        else
        {
            ESP_LOGE(TAG, "Failed to acquire mutex to update writeStatus.");
            return;
        }
    }

    struct tm *local_time = localtime(&local_buffer.timestamp);
    char time_str[20];
    //todo make timestamp format a centralized var
    strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", local_time); // Format as YYYY-MM-DD HH:MM:SS
    fprintf(f, "%lld,%s,%.2f,%ld,%.2f,%d,%d\n", (long long)local_buffer.timestamp, time_str, local_buffer.temperature_c, local_buffer.pressure_pa, local_buffer.battery_voltage, local_buffer.battery_percentage, local_buffer.battery_externally_powered);
    fflush(f);
    fclose(f);

    if (g_sensor_buffer_mutex && xSemaphoreTake(g_sensor_buffer_mutex, pdMS_TO_TICKS(50)) == pdTRUE)
    {
        g_sensor_buffer.writeStatus = 0; // Indicate file write success
        xSemaphoreGive(g_sensor_buffer_mutex);
        return;
    }
    else
    {
        ESP_LOGE(TAG, "Failed to acquire mutex to update writeStatus.");
        return;
    }

}

void spi_sdcard_get_file_count(void)
{
    int file_count = -1; // Default to -1 for error cases

    if (!mounted) {
        ESP_LOGE(TAG, "SD card not mounted, cannot count files.");
    } else {
        int count = 0;
        DIR *dir = opendir("/sdcard");
        if (dir) {
            struct dirent *ent;
            while ((ent = readdir(dir)) != NULL) {
                // Count only regular files, ignore directories
                if (ent->d_type == DT_REG) {
                    count++;
                }
            }
            closedir(dir);
            file_count = count;
        } else {
            ESP_LOGE(TAG, "Failed to open /sdcard directory to count files.");
        }
    }

    // Update the shared buffer with the file count
    if (g_sensor_buffer_mutex && xSemaphoreTake(g_sensor_buffer_mutex, pdMS_TO_TICKS(50)) == pdTRUE)
    {
        g_sensor_buffer.sd_card_file_count = file_count;
        xSemaphoreGive(g_sensor_buffer_mutex);
    }
    else
    {
        ESP_LOGE(TAG, "Failed to acquire mutex to update sd_card_file_count.");
    }
}

void spi_sdcard_get_free_space_mb(void) {
    int free_space_mb = -1; // Default to -1 for error

    if (!mounted) {
        ESP_LOGE(TAG, "SD card not mounted, cannot get free space.");
    } else {
        FATFS *fs;
        DWORD free_clusters;
        // Get volume information and free clusters about the drive 0 (mounted at /sdcard)
        FRESULT res = f_getfree("/sdcard", &free_clusters, &fs);

        if (res == FR_OK) {
            // Calculate free space in bytes
            uint64_t free_bytes = (uint64_t)free_clusters * fs->csize * s_card->csd.sector_size;
            // Convert to MB
            free_space_mb = (int)(free_bytes / (1024 * 1024));
            ESP_LOGI(TAG, "SD card free space: %d MB", free_space_mb);
        } else {
            ESP_LOGE(TAG, "Failed to get free space, f_getfree() error: %d", res);
        }
    }

    // Update the shared buffer
    if (g_sensor_buffer_mutex && xSemaphoreTake(g_sensor_buffer_mutex, pdMS_TO_TICKS(50)) == pdTRUE)
    {
        g_sensor_buffer.sd_card_free_bytes = free_space_mb;
        xSemaphoreGive(g_sensor_buffer_mutex);
    }
    else
    {
        ESP_LOGE(TAG, "Failed to acquire mutex to update sd_card_free_bytes.");
    }
}

/**
 * @brief Formats the SD card. All data on the card will be erased.
 * @note This function handles the unmounting, formatting, and remounting of the card
 * in a single, robust operation. It prevents race conditions and ensures the
 * file system is in a clean state after formatting.
 * @return esp_err_t ESP_OK on success, or an error code on failure.
 */
void spi_sdcard_format(void)
{
    ESP_LOGI(TAG, "Attempting to format SD card.");
    esp_err_t ret = ESP_FAIL;

    if (tud_ready())
    {
        ESP_LOGE(TAG, "Cannot format: SD card is mounted as USB Mass Storage.");
    }
    else if (!mounted)
    {
        ESP_LOGE(TAG, "Cannot format: SD card is not mounted.");
    }
    else
    {
        ret = esp_vfs_fat_sdcard_format("/sdcard", s_card);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to format SD card: %s", esp_err_to_name(ret));
            spi_sdcard_full_init(); // Attempt to remount to restore state
        }
        else
        {
            ESP_LOGI(TAG, "SD card formatted successfully.");
            mounted = true; // esp_vfs_fat_sdcard_format remounts it.
            current_filepath[0] = '\0'; // Reset filename to force creation of a new one
        }
    }

    if (xSemaphoreTake(g_command_status_mutex, portMAX_DELAY)) {
        g_command_status = (ret == ESP_OK) ? CMD_STATUS_SUCCESS : CMD_STATUS_FAIL;
        xSemaphoreGive(g_command_status_mutex);
    }
}
