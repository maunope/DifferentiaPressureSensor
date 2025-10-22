#include "datalogger_task.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include <time.h>
#include <math.h>

#include "../lib/buffers.h"
#include "../lib/i2c-bmp280/i2c-bmp280.h"
#include "../lib/i2c-d6fph/i2c-d6fph.h"
#include "../lib/lipo-battery/lipo-battery.h"
#include "../lib/spi-sdcard/spi-sdcard.h"
#include "../lib/ui/time_utils.h"

static const char *TAG = "DataloggerTask";

/**
 * @brief Reads all relevant sensors and updates the global sensor buffer.
 *
 * This function is thread-safe and uses mutexes to protect access to the
 * I2C bus and the shared sensor buffer.
 * @param bmp280_dev Pointer to the initialized BMP280 device descriptor.
 */
static void update_sensor_buffer(bmp280_t *bmp280_dev, d6fph_t *d6fph_dev)
{
    const int MAX_I2C_RETRIES = 3;
    const int RETRY_DELAY_MS = 50;
    ESP_LOGD(TAG, "Updating sensor buffer...");

    // --- Read all sensor values first ---
    float temperature_c = NAN;
    long pressure_pa = 0; // Use 0 as the sentinel for an invalid integer reading
    float diff_pressure_pa = NAN;

    if (xSemaphoreTake(g_i2c_bus_mutex, pdMS_TO_TICKS(2000)) == pdTRUE)
    {
        esp_err_t d6fph_err = ESP_FAIL;

        // Use forced mode to ensure sensor is correctly configured for each read.
        // This is more robust, especially after waking from deep sleep.
        if (bmp280_force_read(bmp280_dev, &temperature_c, &pressure_pa) != ESP_OK)
        {
            //  ESP_LOGE(TAG, "BMP280 force read failed.");
            temperature_c = NAN;
            pressure_pa = 0;
        }

        if (d6fph_dev && d6fph_dev->is_initialized)
        {
            for (int retries = 0; retries < MAX_I2C_RETRIES; retries++)
            {
                d6fph_err = d6fph_read_pressure(d6fph_dev, &diff_pressure_pa);
                if (d6fph_err == ESP_OK)
                    break; // Success, exit retry loop
                vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
            }
            if (d6fph_err != ESP_OK)
            {
                ESP_LOGE(TAG, "D6F-PH read failed after %d attempts.", MAX_I2C_RETRIES);
            }
        }
        else
        { // d6fph_dev is not initialized
            diff_pressure_pa = NAN;
        }

        xSemaphoreGive(g_i2c_bus_mutex);
    }
    else
    {
        ESP_LOGE(TAG, "Failed to acquire I2C mutex for sensor readings.");
    }

    // --- Now, lock the buffer and update it with all new values ---
    if (xSemaphoreTake(g_sensor_buffer_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        g_sensor_buffer.temperature_c = temperature_c;
        g_sensor_buffer.pressure_pa = pressure_pa;
        g_sensor_buffer.diff_pressure_pa = diff_pressure_pa;
        g_sensor_buffer.timestamp = time(NULL);
        g_sensor_buffer.battery_voltage = battery_reader_get_voltage();
        g_sensor_buffer.battery_percentage = battery_reader_get_percentage();
        g_sensor_buffer.battery_externally_powered = battery_is_externally_powered();
        xSemaphoreGive(g_sensor_buffer_mutex);
    }
    else
    {
        ESP_LOGE(TAG, "Failed to acquire mutex to update sensor buffer.");
    }
}

void datalogger_task(void *pvParameters)
{
    // The BMP280 device handle is passed as the task parameter.
    datalogger_task_params_t *params = (datalogger_task_params_t *)pvParameters;
    bmp280_t *bmp280_dev = params->bmp280_dev;
    const uint32_t LOG_INTERVAL_MS_NORMAL = params->log_interval_ms;
    const uint32_t LOG_INTERVAL_MS_HF = params->hf_log_interval_ms;
    ESP_LOGI(TAG, "Datalogger task started with intervals: Normal=%lu ms, HF=%lu ms", (unsigned long)LOG_INTERVAL_MS_NORMAL, (unsigned long)LOG_INTERVAL_MS_HF);

    // Wait until the main app signals that all initialization is complete.
    ESP_LOGI(TAG, "Waiting for initialization to complete...");
    xEventGroupWaitBits(g_init_event_group, INIT_DONE_BIT, pdFALSE, pdTRUE, portMAX_DELAY);

    // Set the CSV header for all new log files.
    spi_sdcard_set_csv_header("timestamp_gmt,datetime_cet,temperature_c,pressure_pa,diff_pressure_pa,battery_voltage,battery_percentage,uptime_seconds");

    // Perform an initial sensor update right after startup.
    update_sensor_buffer(bmp280_dev, params->d6fph_dev);

    bool refresh_requested = false;

    while (1)
    {
        datalogger_command_t cmd;

        // Check for a command without blocking. The main timing is handled by the vTaskDelay at the end.
        if (xQueueReceive(g_datalogger_cmd_queue, &cmd, 0) == pdPASS)
        {
            if (cmd == DATALOGGER_CMD_FORCE_REFRESH)
            {
                // UI requested a refresh. Just update the buffer, don't write to SD.
                ESP_LOGD(TAG, "UI refresh requested.");
                refresh_requested = true;
            }
            else if (cmd == DATALOGGER_CMD_PAUSE_WRITES)
            {
                ESP_LOGI(TAG, "Pausing writes to SD card.");
                vTaskSuspend(NULL); // Suspend self. Execution resumes here after vTaskResume.

                ESP_LOGI(TAG, "Resumed. Performing warm-up read.");
                // After resuming (especially after sleep), perform an initial read
                // to ensure the sensor has fresh data and is fully awake. This "primes"
                // the sensor before the next timed read occurs.
                float temp, diff_press;
                long press;
                if (xSemaphoreTake(g_i2c_bus_mutex, portMAX_DELAY))
                {
                    // Use the new blocking function to guarantee a fresh reading
                    esp_err_t force_read_err = bmp280_force_read(bmp280_dev, &temp, &press);
                    if (params->d6fph_dev && params->d6fph_dev->is_initialized)
                    {
                        // Also read the D6F-PH sensor
                        d6fph_read_pressure(params->d6fph_dev, &diff_press);
                    }
                    else
                    {
                        diff_press = NAN;
                    }
                    xSemaphoreGive(g_i2c_bus_mutex);

                    if (force_read_err == ESP_OK)
                    {
                        // Now update the global buffer with this guaranteed fresh data
                        if (xSemaphoreTake(g_sensor_buffer_mutex, portMAX_DELAY))
                        {
                            g_sensor_buffer.temperature_c = temp;
                            g_sensor_buffer.pressure_pa = press;
                            g_sensor_buffer.diff_pressure_pa = diff_press;
                            g_sensor_buffer.timestamp = time(NULL);
                            g_sensor_buffer.battery_voltage = battery_reader_get_voltage();
                            g_sensor_buffer.battery_percentage = battery_reader_get_percentage();
                            xSemaphoreGive(g_sensor_buffer_mutex);
                        }
                    }
                }
            }
            else if (cmd == DATALOGGER_CMD_ROTATE_FILE)
            {
                ESP_LOGI(TAG, "Rotating log file on request.");
                spi_sdcard_rotate_file();
            }
        }

        time_t current_ts = time(NULL);

        bool hf_mode_active = false;

        time_t last_write_ts = 0;
        uint32_t current_log_interval = LOG_INTERVAL_MS_NORMAL;
        // Get the last write time from the shared buffer
        if (xSemaphoreTake(g_sensor_buffer_mutex, pdMS_TO_TICKS(50)))
        {
            hf_mode_active = g_sensor_buffer.high_freq_mode_enabled;
            current_log_interval = hf_mode_active ? LOG_INTERVAL_MS_HF : LOG_INTERVAL_MS_NORMAL;
            // fake write time if actual is 0 (no write  made since boot) to avoid issues on first iteration (if it is 0, it will trigger immediately)
            // does no harm to "last write display" on UI because it wont' go to shared buffer until after first write,
            // kind of sucks, but that's that
            if (g_sensor_buffer.last_successful_write_ts == 0)
            {
                // This is the first check after boot.
                // If the device has been awake for less than a log interval,
                // pretend the last write just happened to prevent an immediate log.
                if ((esp_timer_get_time() / 1000) < current_log_interval)
                {
                    last_write_ts = current_ts;
                }
                else
                {
                    // Device has been awake longer than an interval, so allow the write.
                    last_write_ts = 0;
                }
            }
            else
            {
                last_write_ts = g_sensor_buffer.last_successful_write_ts;
            }

            xSemaphoreGive(g_sensor_buffer_mutex);
        }

        // Check if enough time (in seconds) has passed for the next log.
        // ESP_LOGI(TAG, "Current TS: %lld, Last Write TS: %lld, Interval: %lu ms", (long long)current_ts, (long long)last_write_ts, (unsigned long)current_log_interval);
        if ((current_ts - last_write_ts) * 1000 >= current_log_interval)
        {
            //  ESP_LOGI(TAG, "Scheduled log interval reached. Logging data.");
            // It's time for a scheduled log. This takes priority.
            // Update sensors and write to SD card.
            update_sensor_buffer(bmp280_dev, params->d6fph_dev);

            // Create a local copy of the buffer for logging and writing to SD
            sensor_buffer_t local_buffer;
            if (xSemaphoreTake(g_sensor_buffer_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
            {
                local_buffer = g_sensor_buffer;
                xSemaphoreGive(g_sensor_buffer_mutex); // Release mutex immediately after copy
            }
            else
            {
                ESP_LOGE(TAG, "Failed to get sensor buffer for logging.");
                continue; // Skip this logging cycle
            }

            // Format timestamp for logging
            char local_time_str[32];
            struct tm local_tm;
            convert_gmt_to_cet(local_buffer.timestamp, &local_tm);
            strftime(local_time_str, sizeof(local_time_str), "%Y-%m-%d %H:%M:%S %Z", &local_tm);

            // Log to console
            ESP_LOGI(TAG, "TS: %s, Temp: %.2fC, Press: %ldPa, Diff: %.2fPa Batt: %d%% (%.2fV), Charging: %d, WriteSD: %d", local_time_str, local_buffer.temperature_c, local_buffer.pressure_pa, local_buffer.diff_pressure_pa, local_buffer.battery_percentage, local_buffer.battery_voltage, local_buffer.battery_externally_powered, local_buffer.writeStatus);

            // Format the data into a CSV string
            char csv_line[200];
            snprintf(csv_line, sizeof(csv_line), "%lld,%s,%.2f,%ld,%.2f,%.2f,%d,%llu",
                     (long long)local_buffer.timestamp,
                     local_time_str,
                     isnan(local_buffer.temperature_c) ? 0.0f : local_buffer.temperature_c,
                     local_buffer.pressure_pa, // Already long, 0 is invalid
                     isnan(local_buffer.diff_pressure_pa) ? 0.0f : local_buffer.diff_pressure_pa,
                     local_buffer.battery_voltage,
                     local_buffer.battery_percentage,
                     local_buffer.uptime_seconds);

            // Write the formatted string to the SD card
            esp_err_t write_err;
            const int MAX_SD_RETRIES = 3;
            for (int i = 0; i < MAX_SD_RETRIES; i++)
            {
                write_err = spi_sdcard_write_line(csv_line, hf_mode_active);
                if (write_err == ESP_OK)
                {
                    break; // Success
                }
                ESP_LOGE(TAG, "Failed to write to SD card (attempt %d/%d). Retrying...", i + 1, MAX_SD_RETRIES);
                vTaskDelay(pdMS_TO_TICKS(100)); // Wait before retrying
            }

            // Update the shared buffer with the final status and timestamps
            if (xSemaphoreTake(g_sensor_buffer_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
            {
                if (write_err == ESP_OK)
                {
                    g_sensor_buffer.writeStatus = WRITE_STATUS_OK;
                    g_sensor_buffer.last_successful_write_ts = current_ts; // Mark successful write with current timestamp
                    ESP_LOGI(TAG, "Successfully wrote to SD card.");
                }
                else
                {
                    g_sensor_buffer.writeStatus = WRITE_STATUS_FAIL;
                    ESP_LOGE(TAG, "Failed to write to SD card after all retries.");
                }
                xSemaphoreGive(g_sensor_buffer_mutex);
            }
            // If the write was successful, signal to the main task that it's safe to sleep.
            if (write_err == ESP_OK)
            {
                if (g_app_cmd_queue != NULL)
                {
                    app_command_t sleep_cmd = APP_CMD_LOG_COMPLETE_SLEEP_NOW;
                    xQueueSend(g_app_cmd_queue, &sleep_cmd, 0);
                }
            }
        }
        else if (refresh_requested)
        {
            // A UI refresh was requested and a timed log is not due.
            // Just update the sensor buffer without writing to SD.
            update_sensor_buffer(bmp280_dev, params->d6fph_dev);
            refresh_requested = false; // Reset the flag
        }

        // This is the main delay for the task loop.
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
