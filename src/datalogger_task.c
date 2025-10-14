#include "datalogger_task.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
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
    float temperature_c = 0.0f;
    long pressure_pa = 0;
    float diff_pressure_pa = NAN;

    if (xSemaphoreTake(g_i2c_bus_mutex, pdMS_TO_TICKS(1000)) == pdTRUE)
    {
        int32_t uncomp_temp = bmp280_read_raw_temp(bmp280_dev);
        temperature_c = bmp280_compensate_temperature(bmp280_dev, uncomp_temp);

        int32_t uncomp_press = bmp280_read_raw_pressure(bmp280_dev);
        pressure_pa = bmp280_compensate_pressure(bmp280_dev, uncomp_press);

        if (d6fph_dev)
        {
            d6fph_read_pressure(d6fph_dev, &diff_pressure_pa);
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

    ESP_LOGI(TAG, "Datalogger task started.");
    static bool is_paused = false;
    uint64_t last_write_ms = 0;
    const uint32_t log_interval_ms = 58000; // making this slightly shorted to ensure quickest recording after sleep

    while (1)
    {
        datalogger_command_t cmd;

        // Wait for a command or for 1 second to pass
        if (xQueueReceive(g_datalogger_cmd_queue, &cmd, pdMS_TO_TICKS(1000)) == pdPASS)
        {
            if (cmd == DATALOGGER_CMD_FORCE_REFRESH)
            {
                ESP_LOGI(TAG, "Forcing sensor data refresh due to command.");
                update_sensor_buffer(bmp280_dev, params->d6fph_dev);
                continue;
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
            }
        }


        uint64_t current_time_ms = esp_timer_get_time() / 1000;

        if (current_time_ms - last_write_ms >= log_interval_ms && !is_paused)
        {
            last_write_ms = current_time_ms;

            // Update all sensor values in the shared buffer
            update_sensor_buffer(bmp280_dev, params->d6fph_dev);

            // Create a local copy of the buffer for logging and writing to SD
            sensor_buffer_t local_buffer;
            if (xSemaphoreTake(g_sensor_buffer_mutex, pdMS_TO_TICKS(50)) == pdTRUE)
            {
                local_buffer = g_sensor_buffer;
                xSemaphoreGive(g_sensor_buffer_mutex);

                // Format timestamp for logging
                char local_time_str[32];
                struct tm local_tm;
                convert_gmt_to_cet(local_buffer.timestamp, &local_tm);
                strftime(local_time_str, sizeof(local_time_str), "%Y-%m-%d %H:%M:%S %Z", &local_tm);

                // Log to console
                ESP_LOGI(TAG, "TS: %s, Temp: %.2fC, Press: %ldPa, Batt: %d%% (%.2fV), Charging: %d, WriteSD: %d", local_time_str, local_buffer.temperature_c, local_buffer.pressure_pa, local_buffer.battery_percentage, local_buffer.battery_voltage, local_buffer.battery_externally_powered, local_buffer.writeStatus);
                
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
                
                g_sensor_buffer.writeStatus = (write_err == ESP_OK) ? WRITE_STATUS_OK : WRITE_STATUS_FAIL; // Update status based on result
                if (write_err == ESP_OK) g_sensor_buffer.last_successful_write_ts = local_buffer.timestamp;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
