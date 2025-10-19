#pragma once

#include "esp_err.h"
#include <time.h>

/**
 * @brief Connects to Wi-Fi, synchronizes with an NTP server, and retrieves the UTC time.
 *
 * This function is a blocking call that handles the entire process of:
 * 1. Initializing and connecting to a Wi-Fi network.
 * 2. Initializing the SNTP client and synchronizing time.
 * 3. Cleaning up and de-initializing Wi-Fi.
 *
 * @param ssid The SSID of the Wi-Fi network to connect to.
 * @param password The password for the Wi-Fi network.
 * @param timeout_ms The maximum time in milliseconds to wait for the entire process.
 * @param out_timestamp Pointer to a time_t variable where the synchronized UTC timestamp will be stored.
 * @return ESP_OK on success, or an error code on failure (e.g., timeout, connection failed).
 */
esp_err_t ntp_client_get_utc_time(const char* ssid, const char* password, uint32_t timeout_ms, time_t* out_timestamp);