#include "ntp-client.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_sntp.h"
#include <string.h>

static const char *TAG = "NTP_CLIENT";

// Event group to signal Wi-Fi connection and NTP sync events
static EventGroupHandle_t s_wifi_event_group;
// Event bits
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
#define NTP_SYNCED_BIT     BIT2

static void time_sync_notification_cb(struct timeval *tv)
{
    ESP_LOGI(TAG, "NTP time synchronized");
    xEventGroupSetBits(s_wifi_event_group, NTP_SYNCED_BIT);
}

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGE(TAG, "Wi-Fi disconnected, connection failed.");
        xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP address:" IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static esp_err_t wifi_init_and_connect(const char* ssid, const char* password, uint32_t timeout_ms, esp_netif_t** p_netif) {
    ESP_LOGI(TAG, "Initializing NTP client and network stack...");
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    *p_netif = esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    strncpy((char*)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid));
    strncpy((char*)wifi_config.sta.password, password, sizeof(wifi_config.sta.password));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Connecting to Wi-Fi SSID: %s", ssid);

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE, // Don't wait for both bits
            pdMS_TO_TICKS(timeout_ms));

    // Unregister the event handlers now that the connection attempt is complete
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Wi-Fi Connected.");
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "Failed to connect to Wi-Fi.");
        return ESP_FAIL;
    }
}

static void wifi_deinit_and_disconnect(esp_netif_t* sta_netif) {
    ESP_LOGI(TAG, "De-initializing Wi-Fi and network stack...");
    esp_wifi_stop();
    esp_wifi_deinit();
    esp_event_loop_delete_default();
    esp_netif_destroy_default_wifi(sta_netif);
    esp_netif_deinit();
    vEventGroupDelete(s_wifi_event_group);
    s_wifi_event_group = NULL;
    ESP_LOGI(TAG, "Wi-Fi de-initialized.");
}

esp_err_t ntp_client_get_utc_time(const char* ssid, const char* password, uint32_t timeout_ms, time_t* out_timestamp)
{
    if (!ssid || !password || !out_timestamp) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_netif_t* sta_netif = NULL;
    if (wifi_init_and_connect(ssid, password, timeout_ms, &sta_netif) != ESP_OK) {
        wifi_deinit_and_disconnect(sta_netif);
        return ESP_FAIL;
    }

    // Wi-Fi is connected, now sync time
    ESP_LOGI(TAG, "Initializing SNTP");
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_set_time_sync_notification_cb(time_sync_notification_cb);
    esp_sntp_init();

    // Wait for NTP sync or timeout
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
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
    wifi_deinit_and_disconnect(sta_netif);

    return result;
}