#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include "esp_err.h"
#include <stdbool.h>

/**
 * @brief Initializes the Wi-Fi manager.
 * This function should be called once at startup. It initializes the underlying network stack.
 */
void wifi_manager_init(void);

/**
 * @brief Connects to the Wi-Fi network using credentials from the configuration.
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t wifi_manager_connect(const char* ssid, const char* password);

/**
 * @brief Disconnects from the Wi-Fi network and stops the Wi-Fi stack.
 */
void wifi_manager_disconnect(void);

/**
 * @brief Checks if the Wi-Fi is currently connected.
 * @return true if connected, false otherwise.
 */
bool wifi_manager_is_connected(void);

/**
 * @brief De-initializes the Wi-Fi manager and releases all resources.
 */
void wifi_manager_deinit(void);

#endif // WIFI_MANAGER_H