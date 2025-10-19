# NTP Client Library

This library provides a simple, blocking function to synchronize the system's time with an NTP server. It is designed for the ESP-IDF framework.

## Features

*   Handles Wi-Fi connection and disconnection.
*   Initializes and de-initializes the SNTP service.
*   Retrieves the current UTC time from `pool.ntp.org`.
*   Provides a configurable timeout for the entire operation.

## Usage

Call `ntp_client_get_utc_time()` with the Wi-Fi SSID, password, a timeout, and a pointer to a `time_t` variable. The function will return `ESP_OK` on success and populate the `time_t` variable with the current UTC timestamp.