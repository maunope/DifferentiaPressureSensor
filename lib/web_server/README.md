# Web Server

This library provides a simple HTTP web server for accessing data logs.
It is built on the `esp_http_server` component provided by ESP-IDF.

## Features

- Serves a web page listing all log files on the SD card.
- Provides a download endpoint to retrieve individual log files.
- Provides API endpoints for listing files, deleting files, and previewing file content.
- Serves a modern, responsive single-page application (SPA) that acts as a file browser and data visualizer.

## How It Works

The web server is designed to be memory-efficient while still providing a rich user experience. Remember, this is a web server running on a tiny microcontroller, not a full-blown Kubernetes cluster! Due to the limited RAM, some operations have to be carefully managed to prevent memory exhaustion and data corruption.

### Core Components

*   **`start_web_server()`**: This is the entry point. It allocates a `file_server_data_t` struct which holds resources that need to be shared across all HTTP requests. It then configures and starts the `esp_http_server`.
*   **`stop_web_server()`**: This function gracefully shuts down the server and, crucially, frees all the resources that were allocated in `start_web_server`. This includes logging the available heap space to help diagnose memory leaks.
*   **URI Handlers**: These are the functions that respond to specific URL requests from a web browser.
    *   `index_html_handler`: Serves the main `/` page. The entire web interface (HTML, CSS, and JavaScript) is hardcoded into a single string in this function. This is a simple and effective way to serve the UI without needing a separate filesystem for the web assets.
    *   `api_files_handler`: Responds to `/api/files` by scanning the `/sdcard/` directory and streaming a JSON list of all log files.
    *   `api_file_delete_handler`: Responds to `DELETE /api/files/...`. Instead of deleting the file directly, it safely sends a command to the `datalogger_task` to perform the deletion, ensuring all filesystem operations are centralized.
    *   `download_handler` & `api_preview_handler`: These are the workhorses that read file data from the SD card and send it to the client.

### Memory Management and Concurrency: The Mutex

The most critical aspect of this web server is how it handles memory for file operations. To avoid allocating a large buffer for every single web request (which would quickly exhaust the device's RAM), we use a single, shared **scratch buffer** (`SCRATCH_BUFSIZE`).

However, the `esp_http_server` is multi-threaded. It can, and will, handle multiple simultaneous connections from different clients, each in its own FreeRTOS task. If two users tried to download or preview a file at the same time, their respective handler tasks would try to write into the *same* scratch buffer, leading to a classic race condition and sending corrupted data to both users.

**This is where the mutex comes in.**

The `file_server_data_t` struct contains not only the `scratch` buffer but also a `scratch_mutex`.

```c
typedef struct
{
    char *scratch;
    SemaphoreHandle_t scratch_mutex;
} file_server_data_t;
```

Before any handler (like `download_handler` or `api_preview_handler`) can use the scratch buffer, it **must** take the mutex:

```c
if (xSemaphoreTake(server_data->scratch_mutex, pdMS_TO_TICKS(5000)) != pdTRUE) {
    // Failed to get the buffer in time, abort the request.
    httpd_resp_send_500(req);
    return ESP_FAIL;
}
```

Once it's finished reading from the file and sending the data, it releases the mutex, allowing another task to use the buffer:

```c
xSemaphoreGive(server_data->scratch_mutex);
```

This approach effectively serializes the file-intensive operations, ensuring that only one request can use the large memory buffer at a time. 
The "large" buffer is actually 50kB, which is around a third of the available heap space, serialization only kicks in if more than one
user is loading a file preview at the same time, and become noticeable only when downloading weeks worth of data. 