#include "spi-sdcard.h"
#include "tinyusb.h"
#include "esp_err.h"
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
#include "../ui/time_utils.h"
#include <stdbool.h>

#include <sys/stat.h>
#include <dirent.h>

#include "../buffers.h"
static const char *TAG = "SPI_SDCARD_V2";

// Forward declaration for MSC storage callback
// void tud_msc_set_spidrv(long drv);

static bool mounted = false;
static sdmmc_card_t *s_card = NULL;
static tinyusb_msc_storage_handle_t s_msc_storage_handle = NULL;
static char current_filepath[264] = ""; // 8 for "/sdcard/" + 255 for d_name + 1 for null
#define MAX_FILE_SIZE_BYTES (10 * 1024 * 1024) // 10 MB

static bool s_tinyusb_is_initialized = false;

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
/**
 * @brief Initializes the GPIO pin used for the SD card Chip Select (CS).
 *
 * Configures the pin as an output with pull-up enabled.
 */
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

void spi_sdcard_init_sd_only(void)
{
    ESP_LOGI(TAG, "Starting SD card only initialization.");
    spi_sdcard_deinit(); // Ensure clean state
    init_gpio_cs();
    host.slot = SPI2_HOST;
    host.max_freq_khz = SDMMC_FREQ_DEFAULT;
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = GPIO_NUM_12,
        .miso_io_num = GPIO_NUM_10,
        .sclk_io_num = GPIO_NUM_11,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 16384,
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
    }
}

void spi_sdcard_full_init()
{
    ESP_LOGI(TAG, "Starting full TinyUSB and SD card initialization.");
    spi_sdcard_init_sd_only();
    if (mounted)
    {
        ESP_LOGI(TAG, "Configuring and installing TinyUSB for MSC.");
        const tinyusb_config_t tusb_cfg = {
            .task.size = 16384 * 2,
            .task.priority = 2,
            .task.xCoreID = 0
        };
        ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

        tinyusb_msc_storage_config_t sd_storage_config = {
            .medium = {
                .wl_handle = (long)s_card,
            },
        };
        ESP_ERROR_CHECK(tinyusb_msc_new_storage_sdmmc(&sd_storage_config, &s_msc_storage_handle));
        s_tinyusb_is_initialized = true;
        ESP_LOGI(TAG, "TinyUSB MSC storage initialized.");
    }
}

/**
 * @brief De-initializes the SD card and TinyUSB MSC.
 *
 * Unmounts the filesystem, deletes the MSC storage handle, and uninstalls
 * the TinyUSB driver to prepare for sleep.
 */
void spi_sdcard_deinit(void)
{
    if (!mounted)
    {
        ESP_LOGW(TAG, "SD Card not mounted, skipping de-initialization.");
        return;
    }

    ESP_LOGI(TAG, "De-initializing TinyUSB and SD card.");
    if (s_msc_storage_handle)
    {
        tinyusb_msc_delete_storage(s_msc_storage_handle);
        s_msc_storage_handle = NULL;
    }
    if (s_tinyusb_is_initialized) {
        tinyusb_driver_uninstall();
        s_tinyusb_is_initialized = false;
    }
    esp_vfs_fat_sdcard_unmount("/sdcard", s_card);
    // The SPI bus is not de-initialized here to allow for faster re-init.
    // If full power savings were needed, spi_bus_free(host.slot) could be called.
    mounted = false;
    s_card = NULL;
    current_filepath[0] = '\0'; // Reset current file path
}

/**
 * @brief Writes a line of text to the current log file on the SD card.
 *
 * This function handles file rotation based on a maximum file size. If the
 * current log file exceeds `MAX_FILE_SIZE_BYTES`, a new file is created.
 * @param line The null-terminated string to write to the file.
 * @return esp_err_t `ESP_OK` on success, or an error code on failure.
 */
esp_err_t spi_sdcard_write_line(const char* line)
{
    //TODO make all erro codes defines
    if (spi_sdcard_is_usb_connected())
    {
        ESP_LOGI(TAG, "TinyUSB is connected and mounted, cannot write to SD card directly.");
        return ESP_ERR_INVALID_STATE;
    }

    if (!mounted) {
        ESP_LOGE(TAG, "SD card not mounted, cannot write.");
        return ESP_ERR_INVALID_STATE;
    }

    // --- File Rotation Logic ---
    struct stat st;
    if (current_filepath[0] == '\0') { // If no file is currently open
        // First run, find the latest file
        DIR *dir = opendir("/sdcard");
        if (dir) {
            struct dirent *ent;
            long long latest_ts = 0;
            char latest_file[264] = "";

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
        // Create a new filename based on the current system time
        snprintf(current_filepath, sizeof(current_filepath), "/sdcard/data_%lld.csv", (long long)time(NULL));
        ESP_LOGI(TAG, "Starting new log file: %s", current_filepath);
    }

    FILE *f = fopen(current_filepath, "a");
    if (f == NULL)
    {
        ESP_LOGE(TAG, "Failed to open file for writing: %s", current_filepath);
        return ESP_ERR_NOT_FOUND;
    }

    fprintf(f, "%s\n", line);
    fflush(f);
    fclose(f);
    return ESP_OK;
}

/**
 * @brief Counts the number of files in the root directory of the SD card.
 *
 * The result is stored in the global sensor buffer.
 */
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

/**
 * @brief Gets the free space on the SD card in megabytes (MB).
 *
 * The result is stored in the global sensor buffer.
 */
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

bool spi_sdcard_is_usb_connected(void)
{
    return tud_ready();
}

/**
 * @brief Formats the SD card. All data on the card will be erased.
 *
 * @note This function handles the unmounting, formatting, and remounting of the card
 * in a single, robust operation. It prevents race conditions and ensures the
 * file system is in a clean state after formatting.
 */
void spi_sdcard_format(void)
{
    ESP_LOGI(TAG, "Attempting to format SD card.");
    esp_err_t ret = ESP_FAIL;

    if (spi_sdcard_is_usb_connected())
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
