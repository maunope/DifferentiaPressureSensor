# ESP32-S3 Differential Pressure Datalogger

This project is a datalogger built on the ESP32-S3 platform using the ESP-IDF framework. It's designed to read environmental data from a BMP280 sensor, display it on an OLED screen with a menu-driven interface, and log the data to an SD card. The system uses a DS3231 Real-Time Clock (RTC) for accurate timestamping and exposes the SD card as a USB Mass Storage Device for easy data access.

Please refer to the following main guides for usage guidance and documentation:

*   **[User Interface Guide](UI_GUIDE.md)**: A complete guide to the on-screen menu system and controls.
*   **[Web UI Guide](WEB_UI.md)**: Instructions for using the web interface to manage files.
*   **[Software Architecture](SOFTWARE_ARCHITECTURE.md)**: An in-depth look at the software design, tasks, and communication mechanisms.
*   **[Board Design](BOARD_DESIGN.md)**: Details on the custom PCB, hardware components, and pinout.

## Overview
 

### Key Features

*   **Sensor Datalogging**: Periodically reads:
    *   Temperature and atmospheric pressure from a BMP280 sensor.
    *   Differential pressure from an Omron D6F-PH sensor.
    *   LiPo battery voltage and percentage. 
    *   Device uptime in seconds.
*   **SD Card Storage**: Logs sensor data to CSV files on an SD card, with support for file rotation based on size (1MB limit, approx. 2 weeks of recording at 1 point per minute).
*   **Real-Time Clock (RTC)**: Utilizes a DS3231 RTC for accurate timestamps. The system can synchronize its time from the RTC on startup.
*   **OLED Display & UI**: Features a menu-driven user interface on an OLED display, controlled by a rotary encoder with a push-button. The UI displays:
    *   Real-time sensor data (temperature, pressure, differential pressure, battery status).
    *   Current local timestamp (CET/CEST).
    *   SD card write status.
    *   Persistent status icons for battery level, charging status, and SD card write errors.
    *   A settings menu to manage the RTC, SD card, view configuration, and display a QR code linking to the project repository.
*   **NTP Time Sync**: Can synchronize the RTC with an NTP server over Wi-Fi.
*   **Web Server**: Can start a web server to allow downloading and previewing logged data files directly from a web browser.
*   **Multiple Sampling Modes**: Supports "Normal", "High Frequency", and "Paused" logging modes, selectable via the UI.
*   **Robust Power Management**: Implements a command-driven deep sleep cycle to conserve battery. The datalogger task explicitly signals when it's safe to sleep, preventing race conditions and ensuring data integrity. The device wakes up on a timer for the next log or via user interaction. A safety threshold on battery voltage avoids data corruption when measured battery voltage is too low to guarantee stable power to the 3.3V rail.
*   **Web File Management**: The web interface allows users to list, preview, download, and delete log files directly from the device.
*   **Exclusive File System Access**: To prevent data corruption, the system ensures that only one component (Datalogger, Web Server, or USB Mass Storage) can access the SD card at a time. For example, starting the web server will pause the datalogger, and connecting the device to a PC will disable both the web server and the datalogger.
*   **Configuration & Persistence**:
    *   **Permanent Settings**: Key parameters like sleep intervals, Wi-Fi credentials, and sensor models can be configured via a `config.ini` file on the SD card. These settings are loaded into flash memory and persist across reboots.
    *   **Session Settings**: UI-selected modes (like "High Frequency" or "Paused") are stored in RTC memory and persist through deep sleep cycles, but will be reset to their default values after a full power cycle or reboot.
*   **RTOS-based**: Built on FreeRTOS, with separate tasks for data logging and UI rendering for a responsive and robust application.

### Hardware Components

*   **Microcontroller**: ESP32-S3
*   **Sensors**:
    *   BMP280: Barometric Pressure and Temperature (I2C)
    *   Omron D6F-PH: Differential Pressure (I2C)
*   **Storage**:
    *   MicroSD Card Slot (SPI)
*   **Display**:
    *   OLED Display (I2C)
*   **User Input**:
    *   Rotary Encoder with Push-button
*   **Timekeeping**:
    *   DS3231 Real-Time Clock (I2C)
*   **Power**:
    *   LiPo Battery with a voltage divider for monitoring.

For more details on the custom PCB, see the [board design documentation](BOARD_DESIGN.md).

## Hardware Notes

### DS3231 RTC Configuration

Please be aware that this firmware automatically configures any connected DS3231 Real-Time Clock with specific default settings upon boot. These settings are optimized for low-power operation and resilience. For details on the exact register values being set, please refer to the `readme.md` file located in the `lib/i2c_ds3231/` directory.


### Usage

1.  **Configuration**: Copy the `config_sample.ini` file to the root of your SD card and rename it to `config.ini`. Edit this file to set your Wi-Fi credentials, sensor model, and other parameters.
2.  **Operation**: The device is controlled with the rotary encoder and its built-in push-button. For a detailed explanation of the menu system and on-screen information, please see the User Interface Guide.
3.  **Web Interface**: When the web server is active, you can manage files from your browser. See the Web UI Guide for more details.
## Software & Libraries

*   **Framework**: ESP-IDF
*   **RTOS**: FreeRTOS
*   **Component Management**: Dependencies like `qrcode` and `tinyusb` are managed by the ESP-IDF Component Manager via the `idf_component.yml` file. The `esp_tinyusb` component for USB Mass Storage is enabled and configured directly through `menuconfig`, with settings reflected in the PlatformIO build profiles.
*   **Software Architecture**: The project is built on a multi-tasking architecture using FreeRTOS. For a detailed explanation of the tasks and how they communicate, see the Software Architecture Guide.
*   **Custom Libraries (located in `lib/`)**: This project uses a modular structure with several custom libraries. Click on a library name to see its specific documentation.
    *   `config_manager`: Handles loading configuration from an INI file.
    *   `i2c_bmp280`: Driver for the BMP280 sensor.
    *   `i2c_d6fph`: Driver for the Omron D6F-PH sensor.
    *   `i2c_ds3231`: Driver for the DS3231 RTC.
    *   `i2c_oled`: Driver for the OLED display.
    *   `lipo_battery`: Logic for reading battery voltage.
    *   `ntp_client`: Helper for synchronizing time from an NTP server.
    *   `rotary_encoder`: Driver for the rotary encoder input.
    *   `spi_sdcard`: High-level manager for SD card operations and USB MSC.
    *   `ui`: Manages the complete user interface, menu, and pages. See the UI Guide.
    *   `web_server`: Provides a web interface for downloading log files. See the Web UI Guide.
    *   `wifi_manager`: Centralizes Wi-Fi connection and disconnection logic.
