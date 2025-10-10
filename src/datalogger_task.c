#include "datalogger_task.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <time.h>

#include "../lib/i2c-bmp280/i2c-bmp280.h"
#include "../lib/spi-sdcard/spi-sdcard.h"
#include "../lib/buffers.h"

#include "../lib/lipo-battery/lipo-battery.h"


static const char *TAG = "DataloggerTask";

extern bmp280_t g_bmp280; // Use the global BMP280 handle

/**
 * @brief Reads all relevant sensors and updates the global sensor buffer.
 *
 * This function is thread-safe and uses mutexes to protect access to the
 * I2C bus and the shared sensor buffer.
 */
static void update_sensor_buffer(void)
{
    ESP_LOGD(TAG, "Updating sensor buffer...");

    // --- Read all sensor values first ---
    float temperature_c = 0.0f;
    long pressure_pa = 0;

    if (xSemaphoreTake(g_i2c_bus_mutex, pdMS_TO_TICKS(1000)) == pdTRUE)
    {
        int32_t uncomp_temp = bmp280_read_raw_temp(&g_bmp280);
        temperature_c = bmp280_compensate_temperature(&g_bmp280, uncomp_temp);

        int32_t uncomp_press = bmp280_read_raw_pressure(&g_bmp280);
        pressure_pa = bmp280_compensate_pressure(&g_bmp280, uncomp_press);
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
    ESP_LOGI(TAG, "Datalogger task started.");
    uint64_t last_write_ms = 0;
    const uint32_t log_interval_ms = 30000; // 30 seconds

    while (1)
    {
        datalogger_command_t cmd;
        bool force_refresh = false;

        // Wait for a command or for 1 second to pass
        if (xQueueReceive(g_datalogger_cmd_queue, &cmd, pdMS_TO_TICKS(1000)) == pdPASS)
        {
            if (cmd == DATALOGGER_CMD_FORCE_REFRESH)
            {
                ESP_LOGI(TAG, "Forcing sensor data refresh due to command.");
                update_sensor_buffer();
                force_refresh = true; // Flag that we just refreshed
            }
        }

        uint64_t current_time_ms = esp_timer_get_time() / 1000;

        if (!force_refresh && (current_time_ms - last_write_ms >= log_interval_ms))
        {
            last_write_ms = current_time_ms;

            // Update all sensor values in the shared buffer
            update_sensor_buffer();

            // Create a local copy of the buffer for logging and writing to SD
            sensor_buffer_t local_buffer;
            if (xSemaphoreTake(g_sensor_buffer_mutex, pdMS_TO_TICKS(50)) == pdTRUE)
            {
                local_buffer = g_sensor_buffer;
                xSemaphoreGive(g_sensor_buffer_mutex);

                // Format timestamp for logging
                char local_time_str[32];
                struct tm local_tm;
                localtime_r(&local_buffer.timestamp, &local_tm);
                strftime(local_time_str, sizeof(local_time_str), "%Y-%m-%d %H:%M:%S %Z", &local_tm);

                // Log to console
                ESP_LOGI(TAG, "TS: %s, Temp: %.2fC, Press: %ldPa, Batt: %d%% (%.2fV), Charging: %d, WriteSD: %d", local_time_str, local_buffer.temperature_c, local_buffer.pressure_pa, local_buffer.battery_percentage, local_buffer.battery_voltage, local_buffer.battery_externally_powered, local_buffer.writeStatus);
                // Write to SD card (this is a potentially slow operation)
                spi_sdcard_write_csv();
            }
        }
    }
}
