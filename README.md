# ESP32-S3 Environmental Datalogger

This project is a datalogger built on the ESP32-S3 platform using the ESP-IDF framework. It's designed to read environmental data from a BMP280 sensor, display it on an OLED screen with a menu-driven interface, and log the data to an SD card. The system uses a DS3231 Real-Time Clock (RTC) for accurate timestamping and exposes the SD card as a USB Mass Storage Device for easy data access.

### Key Features

*   **Sensor Datalogging**: Periodically reads temperature and pressure from a BMP280 sensor.
*   **SD Card Storage**: Logs sensor data to CSV files on an SD card, with support for file rotation based on size (10MB limit).
*   **Real-Time Clock (RTC)**: Utilizes a DS3231 RTC for accurate timestamps. The system can synchronize its time from the RTC on startup and provides a menu option to set the RTC to the compile-time.
*   **OLED Display & UI**: Features a menu-driven user interface on an OLED display, controlled by a rotary encoder with a push-button. The UI displays:
    *   Real-time sensor data (temperature, pressure).
    *   Current timestamp.
    *   SD card write status.
    *   A settings menu to manage the RTC.
*   **USB Mass Storage**: The SD card can be accessed as a USB Mass Storage Device when connected to a computer, allowing for easy retrieval of log files. The system  prevents SD card writes while the USB is mounted.
*   **RTOS-based**: Built on FreeRTOS, with separate tasks for data logging and UI rendering for a responsive and robust application.

### Hardware Components

*   **Microcontroller**: ESP32-S3
*   **Sensors**:
    *   BMP280 Barometric Pressure and Temperature Sensor (I2C)
*   **Storage**:
    *   MicroSD Card Slot (SPI)
*   **Display**:
    *   OLED Display (I2C)
*   **User Input**:
    *   Rotary Encoder with Push-button
*   **Timekeeping**:
    *   DS3231 Real-Time Clock (I2C)

### Software & Libraries

*   **Framework**: ESP-IDF
*   **RTOS**: FreeRTOS
*   **Key Libraries**:
    *   `driver/i2c`, `driver/spi`, `driver/gpio`: ESP-IDF drivers for hardware communication.
    *   `esp_vfs_fat`, `sdmmc_cmd`: For SD card filesystem handling.
    *   `tinyusb`: For USB Mass Storage functionality.
    *   Custom drivers for peripherals: BMP280, DS3231, OLED, and Rotary Encoder.
