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
#include <errno.h>
#include <unistd.h>

#include <sys/stat.h>
#include <dirent.h>

#include "../buffers.h"
static const char *TAG = "SPI_SDCARD_V2";

// Forward declaration for MSC storage callback
// void tud_msc_set_spidrv(long drv);

static bool mounted = false;
static sdmmc_card_t *s_card = NULL;
static tinyusb_msc_storage_handle_t s_msc_storage_handle = NULL;
static char current_filepath[264] = "";        // 8 for "/sdcard/" + 255 for d_name + 1 for null
#define MAX_FILE_SIZE_BYTES (10 * 1024 * 1024) // 10 MB
static char s_csv_header[200] = ""; // Buffer to store the CSV header

static bool s_force_new_file = false; // Flag to force creation of a new file
static bool s_tinyusb_is_initialized = false;

// Forward declaration for static helper function
static void create_new_log_file(bool is_hf_mode);

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


void spi_sdcard_init_sd_only(void)
{
    ESP_LOGI(TAG, "Starting SD card only initialization.");
    spi_sdcard_deinit(); // Ensure clean state

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GPIO_NUM_13),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&io_conf);

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
            .task.xCoreID = 0};
        ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

        tinyusb_msc_storage_config_t sd_storage_config = {
            .medium = {
                .card = s_card, // Correctly assign the sdmmc_card_t pointer
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
    if (s_tinyusb_is_initialized)
    {
        tinyusb_driver_uninstall();
        s_tinyusb_is_initialized = false;
    }

    // Unmount the filesystem
    esp_vfs_fat_sdcard_unmount("/sdcard", s_card);

    // The SPI bus is not de-initialized here to allow for faster re-init.
    // If full power savings were needed, spi_bus_free(host.slot) could be called,
    // but it can cause instability on wakeup if not handled carefully.
    mounted = false;
    s_card = NULL;
    current_filepath[0] = '\0'; // Reset current file path
}

/**
 * @brief Forces the creation of a new log file on the next write.
 *
 * This is achieved by simply clearing the `current_filepath` buffer.
 */
void spi_sdcard_rotate_file(void) {
    current_filepath[0] = '\0';
    s_force_new_file = true;
    ESP_LOGI(TAG, "Log file rotation requested. New file will be created on next write.");
}

/**
 * @brief Sets the header line to be written to new CSV files.
 *
 * @param header The null-terminated string to use as the CSV header.
 */
void spi_sdcard_set_csv_header(const char* header) {
    if (header) {
        strncpy(s_csv_header, header, sizeof(s_csv_header) - 1);
        s_csv_header[sizeof(s_csv_header) - 1] = '\0'; // Ensure null termination
    }
}
/**
 * @brief Writes a line of text to the current log file on the SD card.
 *
 * This function handles file rotation based on a maximum file size. If the
 * current log file exceeds `MAX_FILE_SIZE_BYTES`, a new file is created.
 * @param line The null-terminated string to write to the file.
 * @return esp_err_t `ESP_OK` on success, or an error code on failure.
 */
esp_err_t spi_sdcard_write_line(const char *line, bool is_hf_mode)
{
    if (!mounted)
    {
        ESP_LOGE(TAG, "SD card not mounted, cannot write.");
        return ESP_ERR_INVALID_STATE;
    }

    // The application logic (main_task) ensures that this function is not called
    // when the USB is connected by pausing the datalogger task.
    if (spi_sdcard_is_usb_connected())
    {
        ESP_LOGI(TAG, "TinyUSB is connected and mounted, cannot write to SD card directly.");
        return ESP_ERR_INVALID_STATE;
    }
    // --- File Rotation Logic ---
    struct stat st;


    // If a rotation was explicitly requested, force a new file immediately.
    if (s_force_new_file) {
        create_new_log_file(is_hf_mode);
        s_force_new_file = false; // Reset the flag
    }
    else if (current_filepath[0] == '\0')
    { // If no file is currently open
        // This block runs on the first write after boot or after a file rotation.
        // We need to decide whether to resume an old file or create a new one.

        const char* sscanf_format = is_hf_mode ? "data_%lld_hf.csv" : "data_%lld.csv";
        DIR *dir = opendir("/sdcard");
        if (dir)
        {
            struct dirent *ent;
            long long latest_ts = 0;
            char latest_file[264] = "";

            while ((ent = readdir(dir)) != NULL)
            {
                long long ts;
                // Ensure we only parse files that match the current mode.
                // If in HF mode, filename must contain "_hf".
                // If not in HF mode, filename must NOT contain "_hf".
                bool name_matches_mode = (is_hf_mode && strstr(ent->d_name, "_hf")) ||
                                         (!is_hf_mode && !strstr(ent->d_name, "_hf"));

                if (name_matches_mode && sscanf(ent->d_name, sscanf_format, &ts) == 1)
                {
                    if (ts > latest_ts)
                    {
                        latest_ts = ts;
                        snprintf(latest_file, sizeof(latest_file), "/sdcard/%.255s", ent->d_name);
                    }
                }
            }
            closedir(dir);

            if (latest_ts > 0)
            {
                // Found a previous file, check its size
                if (stat(latest_file, &st) == 0 && st.st_size < MAX_FILE_SIZE_BYTES)
                {
                    ESP_LOGI(TAG, "Continuing with existing log file: %s", latest_file);
                    strncpy(current_filepath, latest_file, sizeof(current_filepath) - 1);
                    current_filepath[sizeof(current_filepath) - 1] = '\0';
                }
            }
        }
        // If after searching, current_filepath is still empty, it means no suitable file was found,
        // so we must create a new one.
        if (current_filepath[0] == '\0') {
            create_new_log_file(is_hf_mode);
        }
    } else if (stat(current_filepath, &st) == 0 && st.st_size >= MAX_FILE_SIZE_BYTES) {
        // The current file is full, so create a new one.
        create_new_log_file(is_hf_mode);
    }

    FILE *f = fopen(current_filepath, "a");
    if (f == NULL)
    {
        ESP_LOGE(TAG, "Failed to open file for writing: %s", current_filepath);
        return ESP_ERR_NOT_FOUND;
    }

    // Write the data to the file stream.
    fprintf(f, "%s\n", line);

    // Flush the C library buffer to the underlying filesystem. This ensures the
    // data is passed to the FATFS layer.
    fflush(f);
    // Closing the file ensures that all buffered data and filesystem metadata (like
    // file size) are physically written to the SD card.
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

    if (!mounted)
    {
        ESP_LOGE(TAG, "SD card not mounted, cannot count files.");
    }
    else
    {
        int count = 0;
        DIR *dir = opendir("/sdcard");
        if (dir)
        {
            struct dirent *ent;
            while ((ent = readdir(dir)) != NULL)
            {
                // Count only regular files, ignore directories
                if (ent->d_type == DT_REG)
                {
                    count++;
                }
            }
            closedir(dir);
            file_count = count;
        }
        else
        {
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
 * @brief Creates a new log file with the correct name based on the recording mode.
 *
 * @param is_hf_mode True if in high-frequency mode, which affects filename.
 */
static void create_new_log_file(bool is_hf_mode) {
    const char* filename_format = is_hf_mode ? "/sdcard/data_%lld_hf.csv" : "/sdcard/data_%lld.csv";
    snprintf(current_filepath, sizeof(current_filepath), filename_format, (long long)time(NULL));
    ESP_LOGI(TAG, "Starting new log file: %s", current_filepath);

    // Write the CSV header to the new file
    if (strlen(s_csv_header) > 0) {
        FILE *header_f = fopen(current_filepath, "w");
        if (header_f) {
            fprintf(header_f, "%s\n", s_csv_header);
            fclose(header_f);
        } else {
            ESP_LOGE(TAG, "Failed to open new file to write header: %s", current_filepath);
        }
    }
}

/**
 * @brief Gets the free space on the SD card in megabytes (MB).
 *
 * The result is stored in the global sensor buffer.
 */
void spi_sdcard_get_free_space_mb(void)
{
    int free_space_mb = -1; // Default to -1 for error

    if (!mounted)
    {
        ESP_LOGE(TAG, "SD card not mounted, cannot get free space.");
    }
    else
    {
        FATFS *fs;
        DWORD free_clusters;
        // Get volume information and free clusters about the drive 0 (mounted at /sdcard)
        FRESULT res = f_getfree("/sdcard", &free_clusters, &fs);

        if (res == FR_OK)
        {
            // Calculate free space in bytes
            uint64_t free_bytes = (uint64_t)free_clusters * fs->csize * s_card->csd.sector_size;
            // Convert to MB
            free_space_mb = (int)(free_bytes / (1024 * 1024));
            ESP_LOGI(TAG, "SD card free space: %d MB", free_space_mb);
        }
        else
        {
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
esp_err_t spi_sdcard_format(void)
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
        // The main_task is responsible for pausing the datalogger before calling this.
        ret = esp_vfs_fat_sdcard_format("/sdcard", s_card);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to format SD card: %s", esp_err_to_name(ret));
            // We don't try to remount here as the filesystem is in an unknown state.
        }
        else
        {
            ESP_LOGI(TAG, "SD card formatted successfully.");
            mounted = true;             // esp_vfs_fat_sdcard_format remounts it.
            current_filepath[0] = '\0'; // Reset filename to force creation of a new one
        }
    }

    return ret;
}

/**
 * @brief Deletes a file from the SD card.
 *
 * @param path Full path of the file to delete.
 * @return ESP_OK on success, ESP_FAIL on failure.
 */
esp_err_t spi_sdcard_delete_file(const char *path)
{
    if (!mounted)
    {
        ESP_LOGE(TAG, "SD card not mounted, cannot delete file.");
        return ESP_FAIL;
    }

    if (tud_ready())
    {
        ESP_LOGE(TAG, "USB is connected, cannot delete file.");
        return ESP_FAIL;
    }

    if (path == NULL || strlen(path) == 0)
    {
        ESP_LOGE(TAG, "Invalid path for file deletion.");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Deleting file: %s", path);
    if (unlink(path) == 0)
    {
        ESP_LOGI(TAG, "File deleted successfully.");
        return ESP_OK;
    }

    ESP_LOGE(TAG, "Failed to delete file: %s. Error: %s", path, strerror(errno));
    return ESP_FAIL;
}
