#include "datalogger_task.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"

#include <time.h>

#include "../lib/i2c-bmp280/i2c-bmp280.h"
#include "../lib/spi-sdcard/spi-sdcard.h"
#include "../lib/buffers.h"

#include "../lib/lipo-battery/lipo-battery.h"

static const char *TAG = "DataloggerTask";

extern bmp280_t g_bmp280; // Use the global BMP280 handle

void datalogger_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Datalogger task started.");
 
    while (1)
    {
        // --- BMP280 Readings ---
        float temperature_c = 0.0f;
        long pressure_pa = 0;

        if (xSemaphoreTake(g_i2c_bus_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
            int32_t uncomp_temp = bmp280_read_raw_temp(&g_bmp280);
            temperature_c = bmp280_compensate_temperature(&g_bmp280, uncomp_temp);

            int32_t uncomp_press = bmp280_read_raw_pressure(&g_bmp280);
            pressure_pa = bmp280_compensate_pressure(&g_bmp280, uncomp_press);
            // Give the mutex back as soon as we are done with the I2C bus
            xSemaphoreGive(g_i2c_bus_mutex);
        }

        
        // --- Battery Readings ---
        float battery_voltage = battery_reader_get_voltage();
        int battery_percentage = battery_reader_get_percentage();
        int battery_externally_powered = battery_is_externally_powered();


        time_t timestamp = time(NULL); // System time is synchronized with RTC at startup

        // --- Format timestamps for logging ---
        char local_time_str[32];
        char utc_time_str[32];
        struct tm local_tm, utc_tm;

        localtime_r(&timestamp, &local_tm);
        gmtime_r(&timestamp, &utc_tm);
        strftime(local_time_str, sizeof(local_time_str), "%Y-%m-%d %H:%M:%S %Z", &local_tm);
        strftime(utc_time_str, sizeof(utc_time_str), "%Y-%m-%d %H:%M:%S", &utc_tm);

        // --- Copy to shared buffer under mutex ---
        if (g_sensor_buffer_mutex && xSemaphoreTake(g_sensor_buffer_mutex, pdMS_TO_TICKS(50)) == pdTRUE)
        {
            g_sensor_buffer.temperature_c = temperature_c;
            g_sensor_buffer.pressure_pa = pressure_pa;
            g_sensor_buffer.timestamp = timestamp;
            g_sensor_buffer.battery_voltage = battery_voltage;
            g_sensor_buffer.battery_percentage = battery_percentage;
            g_sensor_buffer.battery_externally_powered = battery_externally_powered;
            xSemaphoreGive(g_sensor_buffer_mutex);
        }
        else
        {
            ESP_LOGE(TAG, "Failed to acquire mutex to update sensor buffer.");
        }

        // Log to console
        ESP_LOGI(TAG, "TS: %s, Temp: %.2fC, Press: %ldPa, Batt: %d%% (%.2fV), Charging: %d, WriteSD: %d", local_time_str, temperature_c, pressure_pa, battery_percentage, battery_voltage,  g_sensor_buffer.battery_externally_powered,g_sensor_buffer.writeStatus);
        // Write to SD card (this is a potentially slow operation)
        spi_sdcard_write_csv();

        // Wait for the next cycle
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}