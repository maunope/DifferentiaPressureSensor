# User Interface Guide

This guide explains how to navigate and use the user interface of the Differential Pressure Datalogger. The UI is controlled using a single rotary encoder with a built-in push-button.

## Controls

*   **Rotate Clockwise (CW)**: Navigates **down** in menus or cycles forward through pages.
*   **Rotate Counter-Clockwise (CCW)**: Navigates **up** in menus or cycles backward through pages.
*   **Short Press**:
    *   In a menu, it **selects** the highlighted item. This may enter a submenu, trigger an action, or show an information page.
    *   On an information page (like "About" or "Sensor Data"), it **returns to the main menu**.
*   **Long Press (Hold for ~1 second)**: From any menu screen, it acts as a "home" button, immediately returning you to the **Sensor Data screen**.

## Status Icons

The top-right corner of the screen displays persistent icons that provide at-a-glance information about the device's status.

*   **Battery Icon**: Shows the current battery charge level. The fill level represents the remaining charge.

    ```
    +--------------------+
    | Main Menu      [[]] <-- Battery Icon
    | ...                |
    +--------------------+
    ```

*   **Charging Icon**: When the device is connected to USB power, a lightning bolt symbol appears **inside** the battery icon.

    ```
    +--------------------+
    | Main Menu      [[#]] <-- Charging Icon
    | ...                |
    +--------------------+
    ```

*   **SD Card or Sensor Error**: If the device fails to write to the SD card or a sensor fails to initialize or read, an **error icon (!!)** will appear to the **left** of the battery icon. This icon persists until the condition is resolved.

    ```
    +--------------------+
| Main Menu   !! [[]] <-- Error Icon
    | ...                |
    +--------------------+
    ```

*   **USB Connection Icon**: When the device is connected to a computer and mounted as a USB drive, a **USB plug icon** will appear. This indicates that the SD card is accessible from your computer. While this icon is active, other status icons (like SD Error, Paused, and High-Frequency) are hidden, as logging is temporarily suspended.

*   **Paused Icon**: When data logging is paused via the "Sampling Mode" menu, a **pause icon (||)** will appear to the left of the battery icon. In this mode, the device will not write any new data to the SD card and will not enter deep sleep automatically.

    ```
    +--------------------+
| Main Menu   || [[]] <-- Paused Icon
    | ...                |
    +--------------------+
    ```

*   **High-Frequency Mode**: When enabled, a **fast-forward icon (>>)** appears to the left of the battery icon. This indicates that the device is using shorter logging and sleep intervals.

    ```
    +--------------------+
| Main Menu   >> [[]] <-- HF Mode Icon
    | ...                |
    +--------------------+
    ```

## Main Screens

### Sensor Data Screen (Home)

This is the default screen shown on startup. It is composed of two pages that you can cycle through by **rotating the encoder**. A **short press** on the button will take you to the Main Menu.

#### Page 1: Sensor Readings

This page shows the primary environmental sensor data, now with icons for clarity.

```
+--------------------+
| Sensor Data        |
|                    |
| 2023-10-27 14:30:15|  <-- Current Date & Time (CET)
|                    |
| T: 24.51 C         |  <-- Temperature
| P: 101325 Pa       |  <-- Atmospheric Pressure
| DP: -2.34 Pa       |  <-- Differential Pressure
+--------------------+
```

#### Page 2: System Information

This page displays system-level information, including the persistent uptime and the time of the last successful write to the SD card.

```
+--------------------+
| Sensor Data        |
|                    |
| Uptime: 0:01:23:45 |  <-- Total awake time (D:HH:MM:SS)
|                    |
| lw: 14:30:05       |  <-- Time of last successful SD write
+--------------------+
```

### View Config Screen

This page allows you to view the current configuration parameters loaded from the `config.ini` file. You can scroll through the parameters using the rotary encoder.

```
+--------------------+
| View Config        |
| v_div_ratio        |
|  4.1333            |
| inactive_ms        |
|  30000             |
| sleep_ms           |
|  60000             |
+--------------------+
```

## Menu System

A short press on the button from the main data screen enters the menu system.

### Main Menu

```
+--------------------+
| Main Menu          |
|> Sensor data       |
|  Options >         |
|  Web Server >      |
|  High Freq. Mode   |
|  About             |
|                    |
|                    |
+--------------------+
```

*   **Sensor data**: Returns to the main data display screen.
*   **High Freq. Mode**: Enters a menu to enable or disable high-frequency logging mode. This mode is temporary and resets on reboot.
*   **Sampling Mode**: Enters a submenu to control the data logging behavior. These modes persist through deep sleep cycles but are reset to "Normal" on a full reboot.
*   **Web Server**: Enables a web server to allow data files download through WiFi, pauses data logging and disables sleep.
*   **Options**: Enters the `Options` submenu for device settings.
*   **About**: Displays a QR code and a short link to the project's GitHub repository.

### Sampling Mode Submenu

This menu allows you to change how the datalogger operates for the current session.

```
+--------------------+
| Sample Mode        |
|> High Frequency    |
|  Normal            |
|  Paused            |
|                    |
|                    |
|                    |
+--------------------+
```


### Web Server page

When you start the web server from the menu, the device will connect to your Wi-Fi and display a QR code and a URL on the OLED screen. You can scan the QR code or enter the URL into a web browser on a device connected to the same network.

```
+--------------------+
|                    |
|    ████████████    |
|    █ ▄▄▄▄▄▄ █ █    |
|    █ █▄█▄█▄ █ █    |
|    █▄▄▄▄▄▄▄█ █    |
|                    |
+--------------------+
```

*   **Download**: Click the "Download" button to save a CSV file to your device.
*   **Delete**: Click the "Delete" button to permanently remove a file from the SD card.
*   **Preview**: Click the "Preview" button to view the contents of a CSV file directly in your browser. This feature loads the file in chunks, making it fast and mobile-friendly. It includes a "sticky" header so you always see the column titles, and "Load More" / "Load All" buttons for viewing large files.


### Options Menu

```
+--------------------+
| Options            |
|> Real Time Clock > |
|  File System >     |
|  View Config       |
|  Back              |
|                    |
|                    |
+--------------------+
```


*   **Real Time Clock**: Enters a submenu for time-related actions.
*   **File System**: Enters a submenu for SD card actions.
*   **View Config**: Shows the configuration viewer page.
*   **Back**: Returns to the Main Menu.

### Real Time Clock Submenu

```
+--------------------+
| Real Time Clock    |
|> Set Time (build)  |
|  Sync NTP          |
|  Back              |
|                    |
|                    |
|                    |
+--------------------+
```

*   **Set Time (build)**: Sets the DS3231 RTC to the time the firmware was compiled. A confirmation screen will appear.
*   **Sync NTP**: Attempts to connect to Wi-Fi (using credentials from `config.ini`) and synchronize the RTC with an internet time server.

### File System Submenu

```
+--------------------+
| File System        |
|> View Stats        |
|  Format SD         |
|  Back              |
|                    |
|                    |
|                    |
+--------------------+
```

*   **View Stats**: Shows a page with SD card statistics, including free space and file count.
*   **Format SD**: **(Warning!)** Enters a confirmation screen to format the SD card, which will erase all data.

### About Page

Selecting "About" from the Main Menu will display a QR code. Scan this with your phone to visit the project's source code repository.

```
+--------------------+
|                    |
|    ████████████    |
|    █ ▄▄▄▄▄▄ █ █    |
|    █ █▄█▄█▄ █ █    |
|    █▄▄▄▄▄▄▄█ █    |
|                    |
+--------------------+
```