#include "datalogger_task.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include <time.h>
#include <math.h>

#include "../lib/i2c-bmp280/i2c-bmp280.h"
#include "../lib/spi-sdcard/spi-sdcard.h"
#include "../lib/buffers.h"
#include "../lib/ui/time_utils.h"

#include "../lib/lipo-battery/lipo-battery.h"

#include "../lib/i2c-d6fph/i2c-d6fph.h"


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
    ESP_LOGD(TAG, "Updating sensor buffer...");

    // --- Read all sensor values first ---
    float temperature_c = NAN;
    long pressure_pa = 0; // Use 0 as the sentinel for an invalid integer reading
    float diff_pressure_pa = NAN;

    if (xSemaphoreTake(g_i2c_bus_mutex, pdMS_TO_TICKS(1000)) == pdTRUE)
    {
        int32_t uncomp_temp = bmp280_read_raw_temp(bmp280_dev);
        temperature_c = bmp280_compensate_temperature(bmp280_dev, uncomp_temp);

        ESP_LOGI(TAG, "Raw temp: %ld, Compensated: %.2fC", uncomp_temp, temperature_c);

        int32_t uncomp_press = bmp280_read_raw_pressure(bmp280_dev);
        pressure_pa = bmp280_compensate_pressure(bmp280_dev, uncomp_press);

        if (d6fph_dev && d6fph_dev->is_initialized)
        {
            if (d6fph_read_pressure(d6fph_dev, &diff_pressure_pa) != ESP_OK) {
                diff_pressure_pa = NAN; // Explicitly set to NAN on failure
            }
        } else {
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
    const uint32_t LOG_INTERVAL_MS = params->log_interval_ms;
    ESP_LOGI(TAG, "Datalogger task started with log interval: %lu ms", (unsigned long)LOG_INTERVAL_MS);
    static bool is_paused = false;

    // Wait until the main app signals that all initialization is complete.
    ESP_LOGI(TAG, "Waiting for initialization to complete...");
    xEventGroupWaitBits(g_init_event_group, INIT_DONE_BIT, pdFALSE, pdTRUE, portMAX_DELAY);

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
                is_paused = true;
                if (xSemaphoreTake(g_sensor_buffer_mutex, portMAX_DELAY)) {
                    g_sensor_buffer.datalogger_status = DATA_LOGGER_PAUSED;
                    xSemaphoreGive(g_sensor_buffer_mutex);
                }
            }
            else if (cmd == DATALOGGER_CMD_RESUME_WRITES)
            {
                ESP_LOGI(TAG, "Resuming writes to SD card.");
                is_paused = false;
                if (xSemaphoreTake(g_sensor_buffer_mutex, portMAX_DELAY)) {
                    g_sensor_buffer.datalogger_status = DATA_LOGGER_RUNNING;
                    xSemaphoreGive(g_sensor_buffer_mutex);
                }
                // After resuming (especially after sleep), perform an initial read
                // to ensure the sensor has fresh data and is fully awake. This "primes"
                // the sensor before the next timed read occurs.
                ESP_LOGI(TAG, "Performing warm-up read after resuming.");
                float temp;
                long press;
                if (xSemaphoreTake(g_i2c_bus_mutex, portMAX_DELAY))
                {
                    // Use the new blocking function to guarantee a fresh reading
                    esp_err_t force_read_err = bmp280_force_read(bmp280_dev, &temp, &press);
                    xSemaphoreGive(g_i2c_bus_mutex);

                    if (force_read_err == ESP_OK) {
                        // Now update the global buffer with this guaranteed fresh data
                        if (xSemaphoreTake(g_sensor_buffer_mutex, portMAX_DELAY)) {
                            g_sensor_buffer.temperature_c = temp;
                            g_sensor_buffer.pressure_pa = press;
                            g_sensor_buffer.timestamp = time(NULL);
                            // Also update battery stats here
                            g_sensor_buffer.battery_voltage = battery_reader_get_voltage();
                            g_sensor_buffer.battery_percentage = battery_reader_get_percentage();
                            xSemaphoreGive(g_sensor_buffer_mutex);
                        }
                    }
                }
            }
        }


        uint64_t current_time_ms = esp_timer_get_time() / 1000;
        uint64_t last_write_ms = 0;

        // Get the last write time from the shared buffer
        if (xSemaphoreTake(g_sensor_buffer_mutex, pdMS_TO_TICKS(50))) {
            last_write_ms = g_sensor_buffer.last_write_ms;
            xSemaphoreGive(g_sensor_buffer_mutex);
        }

        if (current_time_ms - last_write_ms >= LOG_INTERVAL_MS && !is_paused)
        {
            // It's time for a scheduled log. This takes priority.
            // Update sensors and write to SD card.
            update_sensor_buffer(bmp280_dev, params->d6fph_dev);

            // Create a local copy of the buffer for logging and writing to SD
            sensor_buffer_t local_buffer;
            if (xSemaphoreTake(g_sensor_buffer_mutex, portMAX_DELAY) == pdTRUE)
            {
                local_buffer = g_sensor_buffer;

                // Format timestamp for logging
                char local_time_str[32];
                struct tm local_tm;
                convert_gmt_to_cet(local_buffer.timestamp, &local_tm);
                strftime(local_time_str, sizeof(local_time_str), "%Y-%m-%d %H:%M:%S %Z", &local_tm);

                // Log to console
                ESP_LOGI(TAG, "TS: %s, Temp: %.2fC, Press: %ldPa, Diff: %.2fPa Batt: %d%% (%.2fV), Charging: %d, WriteSD: %d", local_time_str, local_buffer.temperature_c, local_buffer.pressure_pa,local_buffer.diff_pressure_pa,local_buffer.battery_percentage, local_buffer.battery_voltage, local_buffer.battery_externally_powered, local_buffer.writeStatus);
                
                // Format the data into a CSV string
                char csv_line[200];
                snprintf(csv_line, sizeof(csv_line), "%lld,%s,%.2f,%ld,%.2f,%.2f,%d",
                         (long long)local_buffer.timestamp,
                         local_time_str,
                         local_buffer.temperature_c,
                         local_buffer.pressure_pa,
                         local_buffer.diff_pressure_pa,
                         local_buffer.battery_voltage,
                         local_buffer.battery_percentage);

                // Write the formatted string to the SD card
                esp_err_t write_err = spi_sdcard_write_line(csv_line);
                
                // Update the shared buffer with the new status and timestamps
                g_sensor_buffer.last_write_ms = current_time_ms;
                g_sensor_buffer.writeStatus = (write_err == ESP_OK) ? WRITE_STATUS_OK : WRITE_STATUS_FAIL;
                if (write_err == ESP_OK) {
                    // Use time(NULL) to get a fresh timestamp for the write event,
                    // ensuring it's different from the sensor reading timestamp.
                    g_sensor_buffer.last_successful_write_ts = time(NULL);
                }
                xSemaphoreGive(g_sensor_buffer_mutex);
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
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
