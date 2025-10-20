# ESP32-S3 Differential Pressure Datalogger

This project is a datalogger built on the ESP32-S3 platform using the ESP-IDF framework. It's designed to read environmental data from a BMP280 sensor, display it on an OLED screen with a menu-driven interface, and log the data to an SD card. The system uses a DS3231 Real-Time Clock (RTC) for accurate timestamping and exposes the SD card as a USB Mass Storage Device for easy data access.

### Key Features

*   **Sensor Datalogging**: Periodically reads:
    *   Temperature and atmospheric pressure from a BMP280 sensor.
    *   Differential pressure from an Omron D6F-PH sensor.
    *   LiPo battery voltage and percentage.
*   **SD Card Storage**: Logs sensor data to CSV files on an SD card, with support for file rotation based on size (10MB limit).
*   **Real-Time Clock (RTC)**: Utilizes a DS3231 RTC for accurate timestamps. The system can synchronize its time from the RTC on startup.
*   **OLED Display & UI**: Features a menu-driven user interface on an OLED display, controlled by a rotary encoder with a push-button. The UI displays:
    *   Real-time sensor data (temperature, pressure, differential pressure, battery status).
    *   Current timestamp.
    *   SD card write status.
    *   Persistent status icons for battery level, charging status, and SD card write errors.
    *   A settings menu to manage the RTC, SD card, view configuration, and display a QR code linking to the project repository.
*   **NTP Time Sync**: Can synchronize the RTC with an NTP server over Wi-Fi.
*   **Web Server**: Can strt a web server to allow downloading logged data files directly from a web browser.
*   **Power Management**: Implements deep sleep to conserve battery, waking up on a timer or via user interaction (rotary encoder button). The OLED screen also powers down after a period of inactivity.
*   **Exclusive File System Access**: To prevent data corruption, the system ensures that only one component (Datalogger, Web Server, or USB Mass Storage) can access the SD card at a time. For example, starting the web server will pause the datalogger, and connecting the device to a PC will disable both the web server and the datalogger.
*   **Configuration via INI file**: Key parameters like sleep intervals, Wi-Fi credentials, and sensor models can be configured via a `config.ini` file on the SD card. The device does not go to sleep while the web server is active.
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

### Usage

1.  **Configuration**: Copy the `config_sample.ini` file to the root of your SD card and rename it to `config.ini`. Edit this file to set your Wi-Fi credentials, sensor model, and other parameters.
2.  **Operation**: The device is controlled with the rotary encoder and its built-in push-button. For a detailed explanation of the menu system and on-screen information, please see the [User Interface Guide](UI_GUIDE.md).
### Software & Libraries

*   **Framework**: ESP-IDF
*   **RTOS**: FreeRTOS
*   **Component Management**: Dependencies like `qrcode` and `tinyusb` are managed by the ESP-IDF Component Manager via the `idf_component.yml` file. The `esp_tinyusb` component for USB Mass Storage is enabled and configured directly through `menuconfig`, with settings reflected in the PlatformIO build profiles.
*   **Software Architecture**: The project is built on a multi-tasking architecture using FreeRTOS. For a detailed explanation of the tasks and how they communicate, see the Software Architecture Guide.
*   **Custom Libraries (located in `lib/`)**: This project uses a modular structure with several custom libraries. Click on a library name to see its specific documentation.
    *   `config-manager`: Handles loading configuration from an INI file.
    *   `i2c-bmp280`: Driver for the BMP280 sensor.
    *   `i2c-d6fph`: Driver for the Omron D6F-PH sensor.
    *   `i2c-ds3231`: Driver for the DS3231 RTC.
    *   `i2c-oled`: Driver for the OLED display.
    *   `lipo-battery`: Logic for reading battery voltage.
    *   `ntp-client`: Helper for synchronizing time from an NTP server.
    *   `rotaryencoder`: Driver for the rotary encoder input.
    *   `spi-sdcard`: High-level manager for SD card operations and USB MSC.
    *   `ui`: Manages the complete user interface, menu, and pages.
    *   `web_server`: Provides a web interface for downloading log files.
    *   `wifi_manager`: Centralizes Wi-Fi connection and disconnection logic.
