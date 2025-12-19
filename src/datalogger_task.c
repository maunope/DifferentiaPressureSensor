#include "datalogger_task.h"
#include "freertos/FreeRTOS.h"
#include <stdio.h>
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
#include "../lib/wifi_manager/wifi_manager.h"
#include "config_params.h"

static const char *TAG = "DataloggerTask";

#define SENSOR_REFRESH_INTERVAL_MS 5000

// --- Kalman Filter States ---
// Statically allocated to persist across function calls without using global variables in the header.
// These are no longer RTC_DATA_ATTR. We will re-initialize them on every wake-up
// to ensure a clean and consistent state, preventing corruption from deep sleep cycles.
RTC_DATA_ATTR static kalman_filter_t kf_temperature;
RTC_DATA_ATTR static kalman_filter_t kf_pressure;
RTC_DATA_ATTR static kalman_filter_t kf_diff_pressure;
RTC_DATA_ATTR static kalman_filter_t kf_battery_voltage;
RTC_DATA_ATTR static bool kf_initialized = false;


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
    const config_params_t *cfg = config_params_get();
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

    // --- Determine which Kalman parameters to use ---
    bool is_hf = false;
    if (xSemaphoreTake(g_sensor_buffer_mutex, 0) == pdTRUE) {
        is_hf = g_sensor_buffer.high_freq_mode_enabled;
        xSemaphoreGive(g_sensor_buffer_mutex);
    }

    // Update filter parameters in the struct
    kf_temperature.q = is_hf ? cfg->kf_temp_q_hf : cfg->kf_temp_q;
    kf_temperature.r = is_hf ? cfg->kf_temp_r_hf : cfg->kf_temp_r;
    kf_pressure.q = is_hf ? cfg->kf_press_q_hf : cfg->kf_press_q;
    kf_pressure.r = is_hf ? cfg->kf_press_r_hf : cfg->kf_press_r;
    kf_diff_pressure.q = is_hf ? cfg->kf_diff_press_q_hf : cfg->kf_diff_press_q;
    kf_diff_pressure.r = is_hf ? cfg->kf_diff_press_r_hf : cfg->kf_diff_press_r;
    kf_battery_voltage.q = is_hf ? cfg->kf_batt_v_q_hf : cfg->kf_batt_v_q;
    kf_battery_voltage.r = is_hf ? cfg->kf_batt_v_r_hf : cfg->kf_batt_v_r;

    // --- Apply Kalman filters if initialized ---
    float filtered_temperature_c = kf_initialized && !isnan(raw_temperature_c) ? kalman_update(&kf_temperature, raw_temperature_c) : raw_temperature_c;

    float filtered_pressure_kpa;
    if (kf_initialized && !isnan(raw_pressure_kpa)) {
        filtered_pressure_kpa = kalman_update(&kf_pressure, raw_pressure_kpa);
    } else {
        filtered_pressure_kpa = raw_pressure_kpa;
    }

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
    const config_params_t *cfg = config_params_get();
    bool is_hf = false;
    if (xSemaphoreTake(g_sensor_buffer_mutex, 0) == pdTRUE) {
        is_hf = g_sensor_buffer.high_freq_mode_enabled;
        xSemaphoreGive(g_sensor_buffer_mutex);
    }
    kf_battery_voltage.q = is_hf ? cfg->kf_batt_v_q_hf : cfg->kf_batt_v_q;
    kf_battery_voltage.r = is_hf ? cfg->kf_batt_v_r_hf : cfg->kf_batt_v_r;

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

    // Wait until the main app signals that all initialization is complete.
    // This is CRITICAL to ensure the system time is synchronized from the DS3231
    // before we attempt to make any time-based logging decisions.
    ESP_LOGI(TAG, "Waiting for initialization to complete...");
    xEventGroupWaitBits(g_init_event_group, INIT_DONE_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
    ESP_LOGI(TAG, "Initialization complete. Datalogger proceeding.");

    const config_params_t *cfg = config_params_get();

    // --- Kalman Filter Initialization ---
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    bool is_cold_boot = (cause != ESP_SLEEP_WAKEUP_TIMER && cause != ESP_SLEEP_WAKEUP_GPIO);
    bool params_changed = false;

    // Check if the currently loaded Kalman parameters (from RTC memory) match the
    // parameters from the config file. This check is now removed because q and r are no longer
    // part of the filter's state. Instead, we check a separate RTC variable.
    static RTC_DATA_ATTR uint32_t rtc_config_version = 0;
    uint32_t current_config_version = config_get_version();

    if (kf_initialized && rtc_config_version != current_config_version)
    {
        params_changed = true;
        // Update the RTC version to match the new config
        rtc_config_version = current_config_version;
    }

    if (params_changed) {
        ESP_LOGW(TAG, "Kalman filter parameters have changed. Re-seeding filters.");
    }

    // Only re-initialize on a cold boot, if the filters were never initialized, or if config params changed.
    if (is_cold_boot || !kf_initialized || params_changed)
    {
        if (is_cold_boot && !params_changed)
        {
            // This log helps confirm that a filter reset is intentional (e.g., on cold boot)
            ESP_LOGI(TAG, "Cold boot or filters uninitialized. Seeding Kalman filters...");
        }
        // Perform initial sensor readings to get a stable baseline
        float temp_sum = 0, press_sum = 0, diff_press_sum = 0, batt_v_sum = 0;
        const int NUM_SAMPLES = 3;  

        for (int i = 0; i < NUM_SAMPLES; i++)
        {
            vTaskDelay(pdMS_TO_TICKS(250)); // Small delay between readings
            if (xSemaphoreTake(g_i2c_bus_mutex, portMAX_DELAY))
            {
                float temp;
                long press_pa; // Changed from float to long to match bmp280_force_read signature
                if (params->bmp280_available)
                {
                    bmp280_force_read(params->bmp280_dev, &temp, &press_pa);
                    temp_sum += temp;
                    press_sum += (float)press_pa / 1000.0f; // to kPa
                }
                if (params->d6fph_available)
                {
                    float diff_press;
                    d6fph_read_pressure(params->d6fph_dev, &diff_press);
                    diff_press_sum += diff_press;
                }
                xSemaphoreGive(g_i2c_bus_mutex);
            }
            batt_v_sum += battery_reader_get_voltage();
        }

        // Initialize with normal frequency parameters by default.
        // The mode setting command will switch them if needed.
        kalman_init(&kf_temperature, cfg->kf_temp_q, cfg->kf_temp_r, temp_sum / NUM_SAMPLES);
        kalman_init(&kf_pressure, cfg->kf_press_q, cfg->kf_press_r, press_sum / NUM_SAMPLES);
        kalman_init(&kf_diff_pressure, cfg->kf_diff_press_q, cfg->kf_diff_press_r, diff_press_sum / NUM_SAMPLES);
        kalman_init(&kf_battery_voltage, cfg->kf_batt_v_q, cfg->kf_batt_v_r, batt_v_sum / NUM_SAMPLES);
        kf_initialized = true;
        ESP_LOGI(TAG, "Kalman filters initialized and seeded.");

        // If waking from sleep into HF mode, immediately switch to HF params.
        if (!is_cold_boot && g_sensor_buffer.high_freq_mode_enabled) {
            ESP_LOGI(TAG, "Woke up in HF mode, switching to HF Kalman parameters.");
            datalogger_cmd_msg_t hf_cmd = {.cmd = DATALOGGER_CMD_SET_MODE, .mode = DATALOGGER_MODE_HF};
            // Send to self to trigger the parameter switch logic.
            xQueueSend(g_datalogger_cmd_queue, &hf_cmd, 0);
        }
    }


    // Set the CSV header for all new log files.
    spi_sdcard_set_csv_header("timestamp_gmt,datetime_local,raw_temp_c,filtered_temp_c,raw_press_kpa,filtered_press_kpa,raw_diff_press_pa,filtered_diff_press_pa,raw_batt_v,filtered_batt_v,batt_perc,uptime_s");

    // Perform an initial sensor update right after startup.
    update_sensor_buffer(params);
    // This must be in RTC memory to survive deep sleep and prevent a race condition
    // where a sensor refresh happens immediately on wake, causing a double update.
    static RTC_DATA_ATTR uint64_t last_sensor_refresh_ms = 0;
    static bool is_logging_paused = false;
    static time_t last_sleep_check_ts = 0;
    if (is_cold_boot) last_sensor_refresh_ms = 0; // Ensure it's zeroed on a true cold boot
    
    // This variable is local to the task and manages the timestamp for scheduling the next log.
    // It is initialized once and then updated only when a write is attempted.
    // This decouples the scheduling logic from the global buffer's state, which is used for UI display.
    static time_t last_write_ts_for_scheduling = 0;

    while (1)
    {
        datalogger_cmd_msg_t msg;
        uint64_t current_time_ms = esp_timer_get_time() / 1000;

        // --- Command Processing ---
        if (xQueueReceive(g_datalogger_cmd_queue, &msg, 0) == pdPASS)
        {
            if (msg.cmd == DATALOGGER_CMD_FORCE_REFRESH)
            {
                ESP_LOGD(TAG, "Full sensor refresh requested.");
                update_sensor_buffer(params);
                last_sensor_refresh_ms = current_time_ms;
            }
            else if (msg.cmd == DATALOGGER_CMD_REFRESH_BATTERY)
            {
                update_battery_status();
            }
            else if (msg.cmd == DATALOGGER_CMD_PAUSE_WRITES)
            {
                ESP_LOGI(TAG, "Pausing writes to SD card.");
                vTaskSuspend(NULL); // Suspend self. Execution resumes here after vTaskResume.
                ESP_LOGI(TAG, "Resumed from suspension.");
            }
            else if (msg.cmd == DATALOGGER_CMD_SET_MODE)
            {
                bool was_paused = is_logging_paused;
                is_logging_paused = (msg.mode == DATALOGGER_MODE_PAUSED);
                bool is_hf_now = (msg.mode == DATALOGGER_MODE_HF);

                if (xSemaphoreTake(g_sensor_buffer_mutex, portMAX_DELAY))
                {
                    g_sensor_buffer.high_freq_mode_enabled = (msg.mode == DATALOGGER_MODE_HF);
                    g_sensor_buffer.datalogger_paused = is_logging_paused;
                    xSemaphoreGive(g_sensor_buffer_mutex);
                }

                ESP_LOGI(TAG, "Datalogger mode set to: %d (Paused: %d, HF: %d)", msg.mode, is_logging_paused, is_hf_now);
                
                // Force a file rotation on any mode change to separate data segments.
                // This is critical when switching between normal/HF or resuming from pause.
                // We also reset the scheduling timestamp to now to ensure the next log happens
                // after one full interval in the new mode.
                if (was_paused != is_logging_paused || g_sensor_buffer.high_freq_mode_enabled != is_hf_now)
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

        // Initialize with safe defaults in case the mutex is not available.
        uint32_t current_log_interval;

        // Get the last write time from the shared buffer
        if (xSemaphoreTake(g_sensor_buffer_mutex, pdMS_TO_TICKS(50)))
        {
            current_log_interval = g_sensor_buffer.high_freq_mode_enabled ? LOG_INTERVAL_MS_HF : LOG_INTERVAL_MS_NORMAL;
            
            // On first run (cold boot or wake), initialize our scheduling timestamp.
            if (last_write_ts_for_scheduling == 0) {
                ESP_LOGI(TAG, "First run after boot. Setting initial write timestamp to now.");
                last_write_ts_for_scheduling = (g_sensor_buffer.last_write_attempt_ts > 0) ? g_sensor_buffer.last_write_attempt_ts : current_ts;
            }
            xSemaphoreGive(g_sensor_buffer_mutex);
        } else {
            current_log_interval = LOG_INTERVAL_MS_NORMAL;
            ESP_LOGW(TAG, "Could not obtain sensor buffer mutex to check log time.");
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
        time_t next_log_ts = last_write_ts_for_scheduling + (time_t)ceil((double)current_log_interval / 1000.0);
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
                write_err = spi_sdcard_write_line(csv_line, g_sensor_buffer.high_freq_mode_enabled);
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
                }
                // ALWAYS update the attempt timestamp and the local scheduling variable
                g_sensor_buffer.last_write_attempt_ts = current_ts;
                last_write_ts_for_scheduling = current_ts;
                xSemaphoreGive(g_sensor_buffer_mutex);
            }
            else {
                ESP_LOGE(TAG, "Failed to acquire sensor buffer mutex to update write status.");
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
