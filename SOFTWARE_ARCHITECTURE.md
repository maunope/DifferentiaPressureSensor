# Software Architecture

This document provides an overview of the software architecture for the Differential Pressure Datalogger project. The system is built on the FreeRTOS real-time operating system, leveraging a multi-tasking design to ensure responsiveness and modularity.

## Core Principles

The architecture is designed around a few key principles:

*   **Separation of Concerns**: Each major function (data logging, user interface, input handling) is managed by its own dedicated task.
*   **Asynchronous Communication**: Tasks communicate primarily through message queues, which prevents tasks from blocking each other and keeps the system responsive.
*   **Shared State with Protection**: A central data structure, protected by a mutex, holds the current state and sensor data, providing a single source of truth for all tasks.

## Task Overview

The system is composed of several key tasks, each with a specific role and priority. In FreeRTOS, a higher number indicates a higher priority.

| Task Name               | Priority | Core Responsibility                                      |
| ----------------------- | :------: | -------------------------------------------------------- |
| `rotary_encoder_task`   |    7     | Hardware input processing (highest priority).            |
| `uiRender_task`         |    6     | User interface rendering and event handling.             |
| `main_task`             |    5     | Central controller and state management.                 |
| `datalogger_task`       |    5     | Sensor reading and data logging to the SD card.          |
| Web Server Task         |  (ESP)   | Handles HTTP requests (managed by `esp_http_server`).    |

### `rotary_encoder_task` (Priority 7)

This is the highest-priority application task. Its sole responsibility is to capture and decode hardware interrupts from the rotary encoder. It detects rotation (clockwise/counter-clockwise) and button events (short and long presses), debounces them, and sends them as messages to the `uiRender_task` for processing. This high priority ensures that no user input is ever missed, even if other tasks are busy.

### `uiRender_task` (Priority 6)

This task is the heart of the user interface. It has a higher priority than the main logic to ensure the display is always responsive to user input.

*   **Receives**: Raw input events (e.g., `UI_EVENT_CW`, `UI_EVENT_BTN`) from the `rotary_encoder_task` via the `ui_event_queue`.
*   **Reads**: The shared `g_sensor_buffer` to get the latest data for display.
*   **Sends**: High-level application commands (e.g., `APP_CMD_START_WEB_SERVER`) to the `main_task` via `g_app_cmd_queue`.
*   **Responsibilities**: Manages menu navigation, renders all pages, and displays feedback from long-running commands (e.g., "Loading...").

### `main_task` (Priority 5)

This task acts as the central coordinator for the entire system. It runs a state machine that manages the device's overall behavior.

*   **Receives**: High-level commands from the `uiRender_task` via `g_app_cmd_queue`.
*   **Responsibilities**:
    *   Orchestrates complex sequences, such as starting the web server (which involves pausing the datalogger and connecting to Wi-Fi).
    *   Manages the power-saving logic, including inactivity timers and the deep sleep sequence.
    *   **Arbitrates SD Card Access**: Ensures exclusive access to the file system between the datalogger, web server, and USB Mass Storage.
    *   Ensures exclusive access to the SD card by preventing the web server from starting if USB Mass Storage is active, and stopping the web server if USB is connected.
    *   Handles system-level requests like NTP time sync and SD card formatting.

### `datalogger_task` (Priority 5)

This task handles all sensor reading and data storage.

*   **Receives**: Commands like `DATALOGGER_CMD_PAUSE_WRITES` and `DATALOGGER_CMD_RESUME_WRITES` from the `main_task`.
*   **Writes**: The latest sensor readings and status information to the shared `g_sensor_buffer`.
*   **Responsibilities**:
    *   Periodically wakes up to read data from the BMP280, D6F-PH, and battery monitor.
    *   Formats the data into CSV format.
    *   Writes the data to the SD card.

## Communication Mechanisms

### Shared Data Buffer

*   **`g_sensor_buffer`**: A global `struct` that holds the most recent state of the entire system. This includes sensor values, battery status, SD card statistics, and web server status.
*   **`g_sensor_buffer_mutex`**: A mutex that protects `g_sensor_buffer` from concurrent access. Any task that needs to read from or write to the buffer must first acquire this mutex. To keep the system responsive, tasks are designed to hold the mutex for the shortest possible time.

### Message Queues

*   **`g_app_cmd_queue`**: Used to send high-level commands *to* the `main_task`. This decouples the UI from the application logic. For example, the UI sends `APP_CMD_START_WEB_SERVER`, and the `main_task` handles the multi-step process of making that happen.
*   **`g_datalogger_cmd_queue`**: Used to send specific commands *to* the `datalogger_task`, such as pausing or resuming SD card writes.
*   **`ui_event_queue`**: A dedicated queue for the `uiRender_task`. The `rotary_encoder_task` places raw input events here for the UI to process.
*   **`g_i2c_bus_mutex`**: Protects the main I2C bus (`I2C_NUM_0`), which is shared by multiple sensors (BMP280, DS3231, D6F-PH). This prevents tasks from attempting to communicate on the bus at the same time.
*   **`g_command_status_mutex`**: Protects the `g_command_status` variable. This variable is used for asynchronous feedback between the `main_task` and the `uiRender_task` for long-running operations like formatting the SD card or syncing with an NTP server.

## Exclusive File System Access

The SD card is a critical shared resource. To prevent file system corruption that could occur if multiple tasks tried to write to it simultaneously, the system enforces a strict exclusive access policy. This arbitration is managed by the `main_task`.

There are three components that can access the SD card:

1.  **Datalogger Task**: The default owner, responsible for writing sensor data logs.
2.  **Web Server**: Allows a user to list, download, and delete files via a web browser.
3.  **USB Mass Storage (MSC)**: Allows a host computer to mount the SD card as a drive.

The access rules are as follows:

*   **USB Mass Storage has absolute priority.** If the device is connected to a PC and the SD card is mounted by the host computer, the `main_task` will automatically stop the web server (if it's running) and will prevent both the web server and the datalogger from starting. This gives the host computer exclusive control to ensure a stable connection and prevent data corruption.
*   **The Web Server has the next priority.** When the user starts the web server, the `main_task` first commands the `datalogger_task` to pause. Only after the datalogger is suspended does the web server start. When the web server is stopped, the `main_task` resumes the datalogger.
*   **The Datalogger runs by default.** It operates whenever no other component needs exclusive access to the file system.

This state management ensures that there is only ever one "writer" or "manager" for the SD card, guaranteeing data integrity. As an additional safety measure, the `spi_sdcard_write_line` function inside the `spi-sdcard` library also performs its own check (`spi_sdcard_is_usb_connected()`) before any write operation. This provides a final safeguard against writes, since the activation of the USB MSC stack is handled by the TinyUSB component and is not directly controlled by the application tasks.

## Power Management & Deep Sleep

The device is designed for low-power operation by using a deep sleep cycle. The `main_task` is responsible for managing the sleep and wake logic.

### Entering Deep Sleep

The device will enter deep sleep when all of the following conditions are met:

1.  The UI has been inactive for a configurable period (`inactivity_timeout_ms`), which causes the UI task to suspend.
2.  The `datalogger_task` has completed a write to the SD card and sent an `APP_CMD_LOG_COMPLETE_SLEEP_NOW` command to the `main_task`.
3.  The web server is not active at the time the sleep command is processed.

The sleep sequence is as follows:

1.  The `main_task` receives the `APP_CMD_LOG_COMPLETE_SLEEP_NOW` command.
2.  It verifies that the UI is suspended and the web server is inactive.
3.  It sends a `DATALOGGER_CMD_PAUSE_WRITES` command to the `datalogger_task`.
4.  It waits for the `datalogger_task` to confirm it has suspended itself.
5.  Once paused, it de-initializes the SD card, Wi-Fi, and cuts power to the main peripheral power rail.
6.  It saves critical state variables (e.g., the timestamp of the last write, total uptime) to RTC memory, which persists through deep sleep.
7.  It configures two wakeup sources for the ESP32-S3:
    *   **GPIO**: A press on the rotary encoder button.
    *   **Timer**: A timer set by the `sleep_duration_ms` configuration value.
8.  It calls `esp_deep_sleep_start()` to enter deep sleep.

### Waking Up

The device wakes up when either the timer expires or the user presses the button.

*   **On Timer Wakeup**: The device wakes up to perform a data logging cycle. The UI remains off to conserve power. After the logging is complete, it immediately prepares to enter deep sleep again.
*   **On GPIO Wakeup (User Activity)**: The device performs a full boot sequence, powering on the OLED display and initializing the UI. The user can then interact with the device normally.