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
#include "../lib/i2c_bmp280/i2c_bmp280.h"
#include "../lib/i2c_d6fph/i2c_d6fph.h"
#include "../lib/lipo_battery/lipo_battery.h"
#include "../lib/spi_sdcard/spi_sdcard.h"
#include "../lib/ui/time_utils.h"
#include "../lib/kalman/kalman.h"
#include "config_params.h"

static const char *TAG = "DataloggerTask";

#define SENSOR_REFRESH_INTERVAL_MS 5000

// --- Kalman Filter States ---
// Statically allocated to persist across function calls without using global variables in the header.
static kalman_filter_t kf_temperature;
static kalman_filter_t kf_pressure;
static kalman_filter_t kf_diff_pressure;
static kalman_filter_t kf_battery_voltage;
static bool kf_initialized = false;

/**
 * @brief Reads all relevant sensors and updates the global sensor buffer.
 *
 * This function is thread-safe and uses mutexes to protect access to the
 * I2C bus and the shared sensor buffer.
 * @param bmp280_dev Pointer to the initialized BMP280 device descriptor.
 */
static void update_sensor_buffer(datalogger_task_params_t *params)
{
    const int MAX_I2C_RETRIES = 3;
    const int RETRY_DELAY_MS = 50;
    ESP_LOGD(TAG, "Updating sensor buffer...");

    // --- Read all raw sensor values first ---
    float raw_temperature_c = NAN;
    float raw_diff_pressure_pa = NAN;
    float raw_pressure_kpa = NAN; // Use NAN for consistency
    float raw_battery_voltage = battery_reader_get_voltage();
    bool read_error = false;

    if (xSemaphoreTake(g_i2c_bus_mutex, pdMS_TO_TICKS(2000)) == pdTRUE)
    {
        esp_err_t bmp_err = ESP_FAIL;
        esp_err_t d6fph_err = ESP_FAIL;

        raw_temperature_c = NAN;
        raw_pressure_kpa = NAN;
        if (params->bmp280_available)
        {
            // Use forced mode to ensure sensor is correctly configured for each read.
            // This is more robust, especially after waking from deep sleep.
            long pressure_kpa_long; // Use long for bmp280_force_read

            for (int retries = 0; retries < MAX_I2C_RETRIES; retries++)
            {
                bmp_err = bmp280_force_read(params->bmp280_dev, &raw_temperature_c, &pressure_kpa_long);
                if (bmp_err == ESP_OK)
                    break; // Success, exit retry loop
                vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
            }
            if (bmp_err != ESP_OK)
            {
                read_error = true;
                ESP_LOGE(TAG, "BMP280 read failed after %d attempts.", MAX_I2C_RETRIES);
            }
            else
            {
                raw_pressure_kpa = (float)pressure_kpa_long / 1000.0f; // Convert from Pa to kPa
            }
        }
        else
        {
            ESP_LOGI(TAG, "BMP280 not available, skipping temperature and pressure read.");
        }

        raw_diff_pressure_pa = NAN;
        if (params->d6fph_available)
        {
            for (int retries = 0; retries < MAX_I2C_RETRIES; retries++)
            {
                d6fph_err = d6fph_read_pressure(params->d6fph_dev, &raw_diff_pressure_pa);
                if (d6fph_err == ESP_OK)
                    break; // Success, exit retry loop
                vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
            }
            if (d6fph_err != ESP_OK)
            {
                read_error = true;
                ESP_LOGE(TAG, "D6F-PH read failed after %d attempts.", MAX_I2C_RETRIES);
            }
        }
        else
        { // d6fph_dev is not initialized
            ESP_LOGI(TAG, "D6F-PH not available, skipping differential pressure read.");
        }

        xSemaphoreGive(g_i2c_bus_mutex);
    }
    else
    {
        read_error = true;
        ESP_LOGE(TAG, "Failed to acquire I2C mutex for sensor readings.");
    }

    // --- Initialize Kalman filters on first valid read ---
    if (!kf_initialized && !isnan(raw_temperature_c) && !isnan(raw_pressure_kpa) && !isnan(raw_diff_pressure_pa) && !isnan(raw_battery_voltage))
    {
        const config_params_t *cfg = config_params_get();
        kalman_init(&kf_temperature, cfg->kf_temp_q, cfg->kf_temp_r, raw_temperature_c);
        kalman_init(&kf_pressure, cfg->kf_press_q, cfg->kf_press_r, raw_pressure_kpa);
        kalman_init(&kf_diff_pressure, cfg->kf_diff_press_q, cfg->kf_diff_press_r, raw_diff_pressure_pa);
        kalman_init(&kf_battery_voltage, cfg->kf_batt_v_q, cfg->kf_batt_v_r, raw_battery_voltage);
        kf_initialized = true;
        ESP_LOGI(TAG, "Kalman filters initialized.");
    }

    // --- Apply Kalman filters if initialized ---
    float filtered_temperature_c = kf_initialized && !isnan(raw_temperature_c) ? kalman_update(&kf_temperature, raw_temperature_c) : raw_temperature_c;
    float filtered_pressure_kpa = kf_initialized && !isnan(raw_pressure_kpa) ? kalman_update(&kf_pressure, raw_pressure_kpa) : raw_pressure_kpa;
    float filtered_diff_pressure_pa = kf_initialized && !isnan(raw_diff_pressure_pa) ? kalman_update(&kf_diff_pressure, raw_diff_pressure_pa) : raw_diff_pressure_pa;
    float filtered_battery_voltage = kf_initialized && !isnan(raw_battery_voltage) ? kalman_update(&kf_battery_voltage, raw_battery_voltage) : raw_battery_voltage;

    // --- Now, lock the buffer and update it with all new values ---
    if (xSemaphoreTake(g_sensor_buffer_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        g_sensor_buffer.raw_temperature_c = raw_temperature_c;
        g_sensor_buffer.filtered_temperature_c = filtered_temperature_c;

        g_sensor_buffer.raw_pressure_kpa = raw_pressure_kpa;
        g_sensor_buffer.filtered_pressure_kpa = filtered_pressure_kpa;

        g_sensor_buffer.raw_diff_pressure_pa = raw_diff_pressure_pa;
        g_sensor_buffer.filtered_diff_pressure_pa = filtered_diff_pressure_pa;

        // Battery voltage is read outside the I2C mutex
        g_sensor_buffer.raw_battery_voltage = raw_battery_voltage;
        g_sensor_buffer.filtered_battery_voltage = filtered_battery_voltage;

        // These don't need filtering
        g_sensor_buffer.battery_percentage = battery_reader_get_percentage();
        g_sensor_buffer.battery_externally_powered = battery_is_externally_powered();
        g_sensor_buffer.sensor_read_error = read_error;
        xSemaphoreGive(g_sensor_buffer_mutex);
    }
    else
    {
        ESP_LOGE(TAG, "Failed to acquire mutex to update sensor buffer.");
    }
}

/**
 * @brief Reads only the battery status and updates the global sensor buffer.
 *
 * This is a lightweight version of update_sensor_buffer, intended for quick
 * UI updates when only the power state changes (e.g., USB connected/disconnected).
 */
static void update_battery_status(void)
{
    float raw_voltage = battery_reader_get_voltage();
    if (xSemaphoreTake(g_sensor_buffer_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        g_sensor_buffer.raw_battery_voltage = raw_voltage;
        if (kf_initialized) {
            g_sensor_buffer.filtered_battery_voltage = kalman_update(&kf_battery_voltage, raw_voltage);
        } else {
            // Pass through raw value if filter is not ready
            g_sensor_buffer.filtered_battery_voltage = raw_voltage;
        }
        g_sensor_buffer.battery_percentage = battery_reader_get_percentage();
        g_sensor_buffer.battery_externally_powered = battery_is_externally_powered();
        xSemaphoreGive(g_sensor_buffer_mutex);
    }
    else
    {
        ESP_LOGE(TAG, "Failed to acquire mutex to update battery status.");
    }
}

void datalogger_task(void *pvParameters)
{
    // The BMP280 device handle is passed as the task parameter.
    datalogger_task_params_t *params = (datalogger_task_params_t *)pvParameters;
    const uint32_t LOG_INTERVAL_MS_NORMAL = params->log_interval_ms;
    const uint32_t LOG_INTERVAL_MS_HF = params->hf_log_interval_ms;
    ESP_LOGI(TAG, "Datalogger task started with intervals: Normal=%lu ms, HF=%lu ms", (unsigned long)LOG_INTERVAL_MS_NORMAL, (unsigned long)LOG_INTERVAL_MS_HF);

    // Wait until the main app signals that all initialization is complete.
    ESP_LOGI(TAG, "Waiting for initialization to complete...");
    xEventGroupWaitBits(g_init_event_group, INIT_DONE_BIT, pdFALSE, pdTRUE, portMAX_DELAY);

    // Set the CSV header for all new log files.
    spi_sdcard_set_csv_header("timestamp_gmt,datetime_local,raw_temp_c,filtered_temp_c,raw_press_kpa,filtered_press_kpa,raw_diff_press_pa,filtered_diff_press_pa,raw_batt_v,filtered_batt_v,batt_perc,uptime_s");

    // Perform an initial sensor update right after startup.
    update_sensor_buffer(params);
    // Track the last refresh time to avoid polling sensors too frequently.
    uint64_t last_sensor_refresh_ms = 0;
    static bool is_logging_paused = false;
    static time_t last_sleep_check_ts = 0;

    while (1)
    {
        datalogger_cmd_msg_t msg;
        uint64_t current_time_ms = esp_timer_get_time() / 1000;

        // Check for a command without blocking. The main timing is handled by the vTaskDelay at the end.
        if (xQueueReceive(g_datalogger_cmd_queue, &msg, 0) == pdPASS)
        {
            if (msg.cmd == DATALOGGER_CMD_FORCE_REFRESH)
            {
                // A full refresh was requested.
                ESP_LOGD(TAG, "Full sensor refresh requested.");
                update_sensor_buffer(params);
                last_sensor_refresh_ms = current_time_ms;
            }
            else if (msg.cmd == DATALOGGER_CMD_REFRESH_BATTERY)
            {
                ESP_LOGD(TAG, "Battery status refresh requested.");
                update_battery_status();
            }
            else if (msg.cmd == DATALOGGER_CMD_PAUSE_WRITES)
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
                    esp_err_t force_read_err = ESP_FAIL;
                    if (params->bmp280_available)
                    {
                        // Use the new blocking function to guarantee a fresh reading
                        force_read_err = bmp280_force_read(params->bmp280_dev, &temp, &press);
                    }
                    else
                    {
                        temp = NAN;
                        press = 0;
                    }
                    if (params->d6fph_available) // Check if D6F-PH is available
                    {
                        // Also read the D6F-PH sensor
                        d6fph_read_pressure(params->d6fph_dev, &diff_press);
                    }
                    xSemaphoreGive(g_i2c_bus_mutex);

                    if (force_read_err == ESP_OK)
                    {
                        // Now update the global buffer with this guaranteed fresh data
                        if (xSemaphoreTake(g_sensor_buffer_mutex, pdMS_TO_TICKS(100)))
                        {
                            g_sensor_buffer.raw_temperature_c = temp;
                            g_sensor_buffer.raw_pressure_kpa = press / 1000.0f; // Convert Pa to kPa
                            g_sensor_buffer.raw_diff_pressure_pa = diff_press;
                            g_sensor_buffer.raw_battery_voltage = battery_reader_get_voltage();
                            g_sensor_buffer.battery_percentage = battery_reader_get_percentage();
                            xSemaphoreGive(g_sensor_buffer_mutex);
                        }
                    }
                }
            }
            else if (msg.cmd == DATALOGGER_CMD_SET_MODE)
            {
                bool was_paused = is_logging_paused;
                is_logging_paused = (msg.mode == DATALOGGER_MODE_PAUSED);

                if (xSemaphoreTake(g_sensor_buffer_mutex, portMAX_DELAY))
                {
                    g_sensor_buffer.high_freq_mode_enabled = (msg.mode == DATALOGGER_MODE_HF);
                    g_sensor_buffer.datalogger_paused = is_logging_paused;
                    xSemaphoreGive(g_sensor_buffer_mutex);
                }

                ESP_LOGI(TAG, "Datalogger mode set to: %d (Paused: %d, HF: %d)", msg.mode, is_logging_paused, (msg.mode == DATALOGGER_MODE_HF));

                // Force a file rotation on any mode change to clearly separate data segments.
                // Especially important when coming out of a paused state.
                if (was_paused && !is_logging_paused)
                    spi_sdcard_rotate_file();
            }
            else if (msg.cmd == DATALOGGER_CMD_ROTATE_FILE)
            {
                ESP_LOGI(TAG, "Rotating log file on request.");
                spi_sdcard_rotate_file();
            }
            else if (msg.cmd == DATALOGGER_CMD_DELETE_FILE)
            {
                ESP_LOGI(TAG, "File deletion requested.");
                sensor_buffer_t local_buffer_copy;
                if (xSemaphoreTake(g_sensor_buffer_mutex, pdMS_TO_TICKS(100)))
                {
                    local_buffer_copy = g_sensor_buffer;
                    xSemaphoreGive(g_sensor_buffer_mutex);

                    esp_err_t delete_ret = spi_sdcard_delete_file(local_buffer_copy.file_to_delete);
                    if (xSemaphoreTake(g_sensor_buffer_mutex, pdMS_TO_TICKS(100)))
                    {
                        g_sensor_buffer.delete_file_status = (delete_ret == ESP_OK) ? CMD_STATUS_SUCCESS : CMD_STATUS_FAIL;
                        xSemaphoreGive(g_sensor_buffer_mutex);
                    }
                }
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
            if (g_sensor_buffer.last_write_attempt_ts == 0)
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
                    last_write_ts = 1;
                }
            }
            else
            {
                last_write_ts = g_sensor_buffer.last_write_attempt_ts;
            }

            xSemaphoreGive(g_sensor_buffer_mutex);
        }

        // If paused, we don't log, but we should still check if it's time to sleep.
        if (is_logging_paused)
        {
            if (current_ts - last_sleep_check_ts >= 1) // Check every second
            {
                if (g_app_cmd_queue != NULL)
                {
                    app_command_t sleep_cmd = {.cmd = APP_CMD_LOG_COMPLETE_SLEEP_NOW};
                    xQueueSend(g_app_cmd_queue, &sleep_cmd, 0);
                }
                last_sleep_check_ts = current_ts;
            }
            vTaskDelay(pdMS_TO_TICKS(100)); // Don't spin too fast when paused
            continue;
        }

        // --- Autonomous Sensor Refresh Logic ---.
        if (current_time_ms - last_sensor_refresh_ms >= SENSOR_REFRESH_INTERVAL_MS)
        {
            // The datalogger task is the sole owner of sensor updates.
            // It updates the buffer periodically, and other tasks read from it.
            update_sensor_buffer(params);
            last_sensor_refresh_ms = current_time_ms;
        }

        // ESP_LOGI(TAG, "Checking log time. Current: %lu, Last write: %lu, Interval: %lu ms, Paused: %d", (unsigned long)current_ts, (unsigned long)last_write_ts, (unsigned long)current_log_interval, is_logging_paused);
        //  Check if enough time (in seconds) has passed for the next log.
        time_t next_log_ts = last_write_ts + (time_t)ceil((double)current_log_interval / 1000.0);
        ESP_LOGD(TAG, "Current TS: %lld, Next Log TS: %lld, Paused: %d", (long long)current_ts, (long long)next_log_ts, is_logging_paused);
        if (!is_logging_paused && current_ts >= next_log_ts)
        {
            ESP_LOGD(TAG, "Scheduled log interval reached. Logging data.");
            // It's time for a scheduled log. This takes priority.

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

            // --- Battery Level Check ---

            const config_params_t *cfg = config_params_get();
            if (local_buffer.filtered_battery_voltage > 0 && local_buffer.filtered_battery_voltage < cfg->battery_voltage_treshold)
            {

                ESP_LOGE(TAG, "Cannot log data: Battery voltage (%.2fV) is below threshold (%.2fV)", local_buffer.filtered_battery_voltage, cfg->battery_voltage_treshold);

                if (xSemaphoreTake(g_sensor_buffer_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
                {
                    // Update status to indicate failure due to low battery.
                    g_sensor_buffer.writeStatus = WRITE_STATUS_FAIL;
                    xSemaphoreGive(g_sensor_buffer_mutex);
                }
                // Signal to sleep, as there's nothing more to do if the battery is low.
                app_command_t sleep_cmd = {.cmd = APP_CMD_LOG_COMPLETE_SLEEP_NOW};
                xQueueSend(g_app_cmd_queue, &sleep_cmd, 0);
                // The specific error code is returned conceptually by not writing and logging an error.
                // The error is logged, and the write is skipped.
                continue; // Skip the rest of the logging process
            }

            // Format timestamp for logging
            char local_time_str[32];
            struct tm local_tm;
            convert_gmt_to_cet(current_ts, &local_tm);
            strftime(local_time_str, sizeof(local_time_str), "%Y-%m-%d %H:%M:%S", &local_tm);

            // Log to console
            ESP_LOGI(TAG, "TS: %s, Temp: %.2fC, Press: %.2fkPa, Diff: %.2fPa Batt: %d%% (%.2fV), Charging: %d, WriteSD: %d", local_time_str, local_buffer.filtered_temperature_c, local_buffer.filtered_pressure_kpa, local_buffer.filtered_diff_pressure_pa, local_buffer.battery_percentage, local_buffer.filtered_battery_voltage, local_buffer.battery_externally_powered, local_buffer.writeStatus);

            // Format the data into a CSV string
            char csv_line[200];
            snprintf(csv_line, sizeof(csv_line), "%lld,%s,%.2f,%.2f,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f,%d,%llu", // Use current_ts for the CSV timestamp
                     (long long)current_ts,
                     local_time_str,
                     local_buffer.raw_temperature_c,
                     local_buffer.filtered_temperature_c,
                     local_buffer.raw_pressure_kpa,
                     local_buffer.filtered_pressure_kpa,
                     local_buffer.raw_diff_pressure_pa,
                     local_buffer.filtered_diff_pressure_pa,
                     local_buffer.raw_battery_voltage,
                     local_buffer.filtered_battery_voltage,
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
                    g_sensor_buffer.last_write_attempt_ts = current_ts;
                    g_sensor_buffer.last_successful_write_ts = current_ts; // Mark successful write with current timestamp
                    ESP_LOGI(TAG, "Successfully wrote to SD card.");
                }
                else
                {
                    g_sensor_buffer.writeStatus = WRITE_STATUS_FAIL;
                    // Also update the timestamp to prevent immediate retries after waking up.
                    g_sensor_buffer.last_write_attempt_ts = current_ts;
                    ESP_LOGE(TAG, "Failed to write to SD card after all retries.");
                }
                xSemaphoreGive(g_sensor_buffer_mutex);
            }
            // send a sleep command regardless of write success, this is a safety measure as sleep
            // triggers a full power cycle on the SPI card, which can help recover from certain SD card issues
            if (g_app_cmd_queue != NULL)
            {
                app_command_t sleep_cmd = {.cmd = APP_CMD_LOG_COMPLETE_SLEEP_NOW};
                xQueueSend(g_app_cmd_queue, &sleep_cmd, 0);
            }
        }

        // This is the main delay for the task loop.
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
