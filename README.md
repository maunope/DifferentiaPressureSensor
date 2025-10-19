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
    *   A settings menu to manage the RTC, SD card, and view configuration.
*   **NTP Time Sync**: Can synchronize the RTC with an NTP server over Wi-Fi.
*   **USB Mass Storage**: The SD card can be accessed as a USB Mass Storage Device when connected to a computer, allowing for easy retrieval of log files. The system  prevents SD card writes while the USB is mounted.
*   **Power Management**: Implements deep sleep to conserve battery, waking up on a timer or via user interaction (rotary encoder button). The OLED screen also powers down after a period of inactivity.
*   **Configuration via INI file**: Key parameters like sleep intervals, Wi-Fi credentials, and sensor models can be configured via a `config.ini` file on the SD card.
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
2.  **Operation**: The device is controlled with the rotary encoder and its built-in push-button. For a detailed explanation of the menu system and on-screen information, please see the [user interface guide](UI_GUIDE.md)..

### Software & Libraries

*   **Framework**: ESP-IDF
*   **RTOS**: FreeRTOS
*   **Custom Libraries (located in `lib/`)**:
    *   `driver/i2c`, `driver/spi`, `driver/gpio`: ESP-IDF drivers for hardware communication.
    *   `esp_vfs_fat`, `sdmmc_cmd`: For SD card filesystem handling.
    *   `tinyusb`: For USB Mass Storage functionality.
    *   `i2c-bmp280`: Driver for the BMP280 sensor.
    *   `i2c-d6fph`: Driver for the Omron D6F-PH sensor.
    *   `i2c-ds3231`: Driver for the DS3231 RTC.
    *   `i2c-oled`: Driver for the OLED display.
    *   `rotaryencoder`: Driver for the rotary encoder input.
    *   `spi-sdcard`: High-level manager for SD card operations.
    *   `lipo-battery`: Logic for reading battery voltage.
    *   `ntp-client`: Helper for synchronizing time from an NTP server.
    *   `config-manager`: Handles loading configuration from an INI file.
    *   `ui`: Manages the complete user interface, menu, and pages.
