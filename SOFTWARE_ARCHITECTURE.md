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