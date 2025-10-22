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

*   **SD Card Error**: If the device fails to write to the SD card, a two exclamation marks (**!!**) will appear to the **left** of the battery icon. This icon persists until the next successful write.

    ```
    +--------------------+
    | Main Menu   !! [[]] <-- SD Error
    | ...                |
    +--------------------+
    ```

*   **High-Frequency Mode**: When enabled, a `>>` symbol appears in the top-left corner. This indicates that the device is using shorter logging and sleep intervals.
    If an SD card error occurs, the `>>` icon will shift to the right to make space for the `!!` warning icon.

    ```
    +--------------------+
    | >> !! Main Menu[[]] <-- HF and SD Error
    | ...                |
    +--------------------+
    ```

## Main Screens

### Sensor Data Screen

This is the default screen shown on startup. It is composed of two pages that you can cycle through by **rotating the encoder**. A **short press** on the button will take you to the Main Menu.

#### Page 1: Sensor Readings
#### Page 1: Main Sensor Readings

This page shows the primary environmental sensor data.

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
| last wr.: 14:30:05 |  <-- Time of last successful SD write
| batt: 4.2V (100%)  |  <-- Current battery voltate 
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
*   **Web Server**: Enables a web server to allow data files download through WiFi, pauses data logging and disables sleep.
*   **Options**: Enters the `Options` submenu for device settings.
*   **About**: Displays a QR code and a short link to the project's GitHub repository.

### High Freq. Mode Submenu

High Frequency polling sunmenu allows to enable/disable high frequency polling, the default HF logging interval is 5 seconds. 

```
+--------------------+
| High Freq. Mode    |
|> Enable            |
|  Disable           |
|                    |
|                    |
|                    |
+--------------------+
```


### Web Server page

Selecting "Web Server" from the Main Menu and confirming will display a QR code. Scan this with your phone to visit the data file download page, scroll the rotary encoder to access the plain text URL

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
Buld timestamp and text format repo link are available scrolling through subpages.

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