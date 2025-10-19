# User Interface Guide

This guide explains how to navigate and use the user interface of the Differential Pressure Datalogger. The UI is controlled using a single rotary encoder with a built-in push-button.

## Controls

*   **Rotate Clockwise (CW)**: Navigates **down** in menus or cycles forward through pages.
*   **Rotate Counter-Clockwise (CCW)**: Navigates **up** in menus or cycles backward through pages.
*   **Short Press**:
    *   From the main data screen, it **enters the main menu**.
    *   In a menu, it **selects** the highlighted item. This may enter a submenu, trigger an action, or show an information page.
    *   On an information page (like "About" or "Sensor Data"), it **returns to the previous menu**.
*   **Long Press (Hold for ~1 second)**: From any screen, it acts as a "home" button, immediately returning you to the **main menu**.

## Main Screens

### Sensor Data Screen

This is the default screen shown on startup and after a period of inactivity. It displays real-time data from all sensors.

```
+--------------------+
| Sensors Data       |  <-- Title
| 2023-10-27 14:30:15|  <-- Current Date & Time (CET)
| T: 24.51 C         |  <-- Temperature
| P: 101325 Pa       |  <-- Atmospheric Pressure
| DP: -2.34 Pa       |  <-- Differential Pressure
| Batt: 3.85V-65%    |  <-- Battery Voltage & Percentage
| Last write: OK     |  <-- Status of the last SD card write
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
|  About             |
|                    |
|                    |
|                    |
+--------------------+
```

*   **Sensor data**: Returns to the main data display screen.
*   **Options**: Enters the `Options` submenu for device settings.
*   **About**: Displays the "About" page with device information.

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