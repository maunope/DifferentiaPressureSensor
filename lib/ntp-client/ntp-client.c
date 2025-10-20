#include "ntp-client.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_sntp.h"
#include <string.h>
#include "../wifi_manager/wifi_manager.h"

static const char *TAG = "NTP_CLIENT";

static EventGroupHandle_t s_ntp_event_group;
#define NTP_SYNCED_BIT BIT0

static void time_sync_notification_cb(struct timeval *tv)
{
    ESP_LOGI(TAG, "NTP time synchronized");
    if (s_ntp_event_group) {
        xEventGroupSetBits(s_ntp_event_group, NTP_SYNCED_BIT);
    }
}

esp_err_t ntp_client_get_utc_time(const char* ssid, const char* password, uint32_t timeout_ms, time_t* out_timestamp)
{
    if (!ssid || !password || !out_timestamp) {
        return ESP_ERR_INVALID_ARG;
    }

    if (wifi_manager_connect(ssid, password) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to connect to Wi-Fi for NTP sync.");
        wifi_manager_disconnect(); // Ensure we are disconnected on failure
        return ESP_FAIL;
    }

    // Wi-Fi is connected, now sync time
    ESP_LOGI(TAG, "Initializing SNTP");
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_set_time_sync_notification_cb(time_sync_notification_cb);
    esp_sntp_init();
    s_ntp_event_group = xEventGroupCreate();

    EventBits_t bits = xEventGroupWaitBits(s_ntp_event_group,
            NTP_SYNCED_BIT,
            pdFALSE,
            pdFALSE,
            pdMS_TO_TICKS(timeout_ms));

    esp_err_t result = ESP_FAIL;
    if (bits & NTP_SYNCED_BIT) {
        time(out_timestamp);
        struct tm timeinfo;
        gmtime_r(out_timestamp, &timeinfo);
        ESP_LOGI(TAG, "Successfully synchronized time: %s", asctime(&timeinfo));
        result = ESP_OK;
    } else {
        ESP_LOGE(TAG, "Failed to synchronize time within the timeout period.");
        result = ESP_ERR_TIMEOUT;
    }

    // Clean up
    esp_sntp_stop();
    wifi_manager_disconnect();
    vEventGroupDelete(s_ntp_event_group);
    s_ntp_event_group = NULL;

    return result;
}