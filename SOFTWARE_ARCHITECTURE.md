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

## Power Management & Deep Sleep

The device is designed for low-power operation by using a deep sleep cycle. The `main_task` is responsible for managing the sleep and wake logic.

### Entering Deep Sleep

The device will enter deep sleep when all of the following conditions are met:

1.  The UI has been inactive for a configurable period (`inactivity_timeout_ms`).
2.  A new data log has been successfully written to the SD card.
3.  The web server is not active.

The sleep sequence is as follows:

1.  The `main_task` detects that the sleep conditions have been met.
2.  It suspends the `uiRender_task` and powers off the OLED display to save power.
3.  It sends a `DATALOGGER_CMD_PAUSE_WRITES` command to the `datalogger_task` to ensure no data is being written to the SD card.
4.  It waits in a non-blocking loop, periodically checking the shared buffer until the `datalogger_task` confirms it has paused.
5.  Once paused, it de-initializes the SD card and cuts power to the main peripheral power rail.
6.  It saves critical state variables (e.g., the timestamp of the last write) to RTC memory, which persists through deep sleep.
7.  It configures two wakeup sources:
    *   **GPIO**: A press on the rotary encoder button.
    *   **Timer**: A timer set by the `sleep_duration_ms` configuration value.
8.  It calls `esp_deep_sleep_start()` to enter deep sleep.

### Waking Up

The device wakes up when either the timer expires or the user presses the button.

*   **On Timer Wakeup**: The device wakes up to perform a data logging cycle. The UI remains off to conserve power. After the logging is complete, it immediately prepares to enter deep sleep again.
*   **On GPIO Wakeup (User Activity)**: The device performs a full boot sequence, powering on the OLED display and initializing the UI. The user can then interact with the device normally.