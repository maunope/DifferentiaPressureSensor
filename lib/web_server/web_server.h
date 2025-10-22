#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include "esp_err.h"
#include "esp_http_server.h"

/**
 * @brief Starts the web server.
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t start_web_server(void);

/**
 * @brief Stops the web server.
 */
void stop_web_server(void);

#endif // WEB_SERVER_H