#include "web_server.h"
#include "esp_http_server.h"
#include "../buffers.h"
#include "esp_log.h"
#include "esp_vfs.h"
#include <dirent.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>
#include <stdlib.h>

static const char *TAG = "web_server";
static httpd_handle_t server = NULL;

#define FILE_PATH_MAX (ESP_VFS_PATH_MAX + 128)
#define SCRATCH_BUFSIZE (10240)

typedef struct {
    char *scratch;
} file_server_data_t;

/* Handler to serve the main index.html page */
static esp_err_t index_html_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");

    const char *index_html =
        "<!DOCTYPE html>"
        "<html lang=\"en\">"
        "<head>"
            "<meta charset=\"UTF-8\">"
            "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">"
            "<title>Differential Pressure Sensor</title>"
            "<style>"
                "body { font-family: -apple-system, BlinkMacSystemFont, \"Segoe UI\", Roboto, Helvetica, Arial, sans-serif; background-color: #121212; color: #e0e0e0; margin: 0; padding: 20px; line-height: 1.6; }"
                ".container { max-width: 800px; margin: auto; background-color: #1e1e1e; padding: 20px; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.5); }"
                "h1 { color: #bb86fc; border-bottom: 2px solid #373737; padding-bottom: 10px; }"
                "ul { list-style-type: none; padding: 0; }"
                "li { background-color: #2c2c2c; margin-bottom: 10px; border-radius: 4px; display: flex; justify-content: space-between; align-items: center; }"
                ".file-info { padding: 12px 15px; }"
                ".file-info a { text-decoration: none; color: #03dac6; font-weight: 500; font-size: 1.1em; display: block; }"
                ".file-info a:hover { text-decoration: underline; }"
                ".file-metadata { font-size: 0.8em; color: #aaa; margin-top: 4px; }"
                "button.delete-btn { background-color: #cf6679; color: #121212; border: none;  padding: 0 15px; align-self: stretch; cursor: pointer; border-radius: 0 4px 4px 0; font-weight: bold; transition: background-color 0.2s ease-in-out; }"
                "button.delete-btn:hover { background-color: #ff7991; }"
                "footer { text-align: center; margin-top: 30px; padding-top: 20px; border-top: 1px solid #373737; font-size: 0.9em; color: #888; }"
                "footer a { color: #03dac6; text-decoration: none; }"
                "footer a:hover { text-decoration: underline; }"
                "#file-list-status { color: #aaa; font-style: italic; }"
            "</style>"
        "</head>"
        "<body>"
            "<div class=\"container\">"
                "<h1>Available Data Files</h1>"
                "<ul id=\"file-list\"><p id=\"file-list-status\">Loading files...</p></ul>"
                "<footer>"
                    "<p><a href=\"" PROJECT_GITHUB_URL "\" target=\"_blank\">DifferentialPressureSensor</a> project on GitHub</p>"
                "</footer>"
            "</div>"
            "<script>"
                "document.addEventListener('DOMContentLoaded', function() {"
                    "const fileList = document.getElementById('file-list');"
                    "function formatSize(bytes) {"
                        "if (bytes === 0) return '0 B';"
                        "const k = 1024;"
                        "const sizes = ['B', 'KB', 'MB', 'GB'];"
                        "const i = Math.floor(Math.log(bytes) / Math.log(k));"
                        "return parseFloat((bytes / Math.pow(k, i)).toFixed(2)) + ' ' + sizes[i];"
                    "}"
                    "function loadFiles() {"
                        "fileList.innerHTML = '<p id=\"file-list-status\">Loading files...</p>';"
                        "fetch('/api/files')"
                            ".then(response => response.json())"
                            ".then(data => {"
                                "fileList.innerHTML = '';"
                                "if (data.files && data.files.length > 0) {"
                                    "data.files.sort((a, b) => b.mtime - a.mtime).forEach(file => {"
                                        "const li = document.createElement('li');"
                                        "const infoDiv = document.createElement('div');"
                                        "infoDiv.className = 'file-info';"
                                        "const a = document.createElement('a');"
                                        "a.href = '/download?file=' + encodeURIComponent(file.name);"
                                        "a.textContent = file.name;"
                                        "const meta = document.createElement('div');"
                                        "meta.className = 'file-metadata';"
                                        "const date = new Date(file.mtime * 1000).toLocaleString();"
                                        "meta.textContent = formatSize(file.size) + ' - ' + date;"
                                        "const deleteBtn = document.createElement('button');"
                                        "deleteBtn.textContent = 'Delete';"
                                        "deleteBtn.className = 'delete-btn';"
                                        "deleteBtn.onclick = function() {"
                                            "if (confirm('Are you sure you want to delete ' + file.name + '?')) {"
                                                "fetch('/api/files/' + encodeURIComponent(file.name), { method: 'DELETE' })"
                                                    ".then(response => {"
                                                        "if (response.ok) {"
                                                            "li.remove();"
                                                        "} else {"
                                                            "alert('Failed to delete file.');"
                                                        "}"
                                                    "});"
                                            "}"
                                        "};"
                                        "infoDiv.appendChild(a);"
                                        "infoDiv.appendChild(meta);"
                                        "li.appendChild(infoDiv);"
                                        "li.appendChild(deleteBtn);"
                                        "fileList.appendChild(li);"
                                    "});"
                                "} else {"
                                    "const li = document.createElement('li');"
                                    "li.textContent = 'No files found on SD card.';"
                                    "li.style.padding = '12px 15px';"
                                    "fileList.appendChild(li);"
                                "}"
                            "})"
                            ".catch(error => {"
                                "console.error('Error fetching file list:', error);"
                                "alert('Failed to load file list. The web server may have been turned off or disconnected from Wi-Fi.');"
                                "fileList.innerHTML = '<p>Error loading file list. Please refresh the page.</p>';"
                            "});"
                    "}"
                    "loadFiles();"
                "});"
            "</script>"
        "</body>"
        "</html>";

    return httpd_resp_send(req, index_html, HTTPD_RESP_USE_STRLEN);
}

/* Handler to serve the file list as JSON */
static esp_err_t api_files_handler(httpd_req_t *req) {
    DIR *dir = opendir("/sdcard/");
    if (!dir) {
        ESP_LOGE(TAG, "Failed to open directory: /sdcard/");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    // Start building the JSON response
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send_chunk(req, "{\"files\":[", 10);

    bool first_entry = true;
    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL) {
        if (entry->d_type == DT_REG) { // Only list regular files
            char json_entry[350];
            char full_path[ESP_VFS_PATH_MAX + CONFIG_FATFS_MAX_LFN];
            snprintf(full_path, sizeof(full_path), "/sdcard/%s", entry->d_name);
            struct stat st;
            if (stat(full_path, &st) != 0) {
                continue; // Skip if we can't get file stats
            }

            if (!first_entry) {
                httpd_resp_send_chunk(req, ",", 1);
            }
            snprintf(json_entry, sizeof(json_entry), 
                "{\"name\":\"%s\",\"size\":%ld,\"mtime\":%lld}", 
                entry->d_name, st.st_size, (long long)st.st_mtime);
            httpd_resp_send_chunk(req, json_entry, HTTPD_RESP_USE_STRLEN);
            first_entry = false;
        }
    }
    closedir(dir);

    // Close the JSON array and object
    httpd_resp_send_chunk(req, "]}", 2);
    httpd_resp_send_chunk(req, NULL, 0); // Final chunk
    return ESP_OK;
}

/*
 * @brief Handler to delete a file from the SD card.
 * @note It is safe to delete files here because the main application logic ensures that
 * the datalogger task is paused and the USB Mass Storage is not active while the web server is running.
 */
/* Handler to delete a file */
static esp_err_t api_file_delete_handler(httpd_req_t *req) {
    char filepath[FILE_PATH_MAX];
    // The filename is the part of the URI after the last '/'
    const char *filename = req->uri + strlen("/api/files/");

    // Basic security check: prevent path traversal
    if (strstr(filename, "..")) {
        ESP_LOGE(TAG, "Path traversal attempt detected in delete: %s", filename);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Bad Request");
        return ESP_FAIL;
    }

    snprintf(filepath, sizeof(filepath), "/sdcard/%s", filename);
    ESP_LOGI(TAG, "Attempting to delete file: %s", filepath);

    if (unlink(filepath) == 0) {
        ESP_LOGI(TAG, "File deleted successfully: %s", filepath);
        httpd_resp_set_status(req, "204 No Content");
        httpd_resp_send(req, NULL, 0);
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "Failed to delete file: %s", filepath);
        // Could be 404 Not Found or 500 Internal Server Error
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to delete file");
        return ESP_FAIL;
    }
}

/* Handler to download a file */
static esp_err_t download_handler(httpd_req_t *req) {
    char filepath[FILE_PATH_MAX];
    char filename[64];
    file_server_data_t *server_data = (file_server_data_t *)req->user_ctx;

    // Get the length of the query string
    size_t query_len = httpd_req_get_url_query_len(req) + 1;
    if (query_len <= 1) {
        ESP_LOGE(TAG, "No query string found in download request");
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    // Allocate buffer and get the query string
    char* query_str = malloc(query_len);
    if (httpd_req_get_url_query_str(req, query_str, query_len) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get query string");
        free(query_str);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    // Parse the filename from the query string
    if (httpd_query_key_value(query_str, "file", filename, sizeof(filename)) != ESP_OK) {
        ESP_LOGE(TAG, "Filename not found in query string");
        free(query_str);
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }
    free(query_str);

    // Basic security check: prevent path traversal
    if (strstr(filename, "..")) {
        ESP_LOGE(TAG, "Path traversal attempt detected: %s", filename);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Bad Request");
        return ESP_FAIL;
    }

    snprintf(filepath, sizeof(filepath), "/sdcard/%s", filename);
    ESP_LOGD(TAG, "Serving file: %s", filepath);

    int fd = open(filepath, O_RDONLY, 0);
    if (fd == -1) {
        ESP_LOGE(TAG, "Failed to open file: %s", filepath);
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    // Get file size
    struct stat file_stat;
    if (fstat(fd, &file_stat) == -1) {
        ESP_LOGE(TAG, "Failed to get file stat");
        close(fd);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    char content_disposition[128];
    snprintf(content_disposition, sizeof(content_disposition), "attachment; filename=\"%s\"", filename);
    httpd_resp_set_type(req, "application/octet-stream");
    httpd_resp_set_hdr(req, "Content-Disposition", content_disposition);

    ssize_t read_bytes;
    do {
        read_bytes = read(fd, server_data->scratch, SCRATCH_BUFSIZE);
        if (read_bytes > 0) {
            if (httpd_resp_send_chunk(req, server_data->scratch, read_bytes) != ESP_OK) {
                close(fd);
                ESP_LOGE(TAG, "File sending failed!");
                return ESP_FAIL;
            }
        }
    } while (read_bytes > 0);

    close(fd);
    httpd_resp_send_chunk(req, NULL, 0); // Final chunk
    return ESP_OK;
}

esp_err_t start_web_server(void) {
    if (server) {
        ESP_LOGI(TAG, "Web server already running");
        return ESP_OK;
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.uri_match_fn = httpd_uri_match_wildcard;
    config.max_uri_handlers = 8;

    file_server_data_t *server_data = calloc(1, sizeof(file_server_data_t));
    if (!server_data) {
        ESP_LOGE(TAG, "Failed to allocate memory for server data");
        return ESP_ERR_NO_MEM;
    }
    server_data->scratch = malloc(SCRATCH_BUFSIZE);
    if (!server_data->scratch) {
        ESP_LOGE(TAG, "Failed to allocate scratch buffer");
        free(server_data);
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Starting HTTP Server");
    if (httpd_start(&server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start httpd");
        free(server_data->scratch);
        free(server_data);
        server = NULL;
        return ESP_FAIL;
    }

    httpd_uri_t file_list_uri = {
        .uri       = "/",
        .method    = HTTP_GET,
        .handler   = index_html_handler,
        .user_ctx  = server_data
    };
    httpd_register_uri_handler(server, &file_list_uri);

    httpd_uri_t api_files_uri = {
        .uri       = "/api/files",
        .method    = HTTP_GET,
        .handler   = api_files_handler,
        .user_ctx  = server_data
    };
    httpd_register_uri_handler(server, &api_files_uri);

    httpd_uri_t api_delete_uri = {
        .uri       = "/api/files/*",
        .method    = HTTP_DELETE,
        .handler   = api_file_delete_handler,
        .user_ctx  = server_data
    };
    httpd_register_uri_handler(server, &api_delete_uri);

    httpd_uri_t download_uri = {
        .uri       = "/download",
        .method    = HTTP_GET,
        .handler   = download_handler,
        .user_ctx  = server_data
    };
    httpd_register_uri_handler(server, &download_uri);

    return ESP_OK;
}

void stop_web_server(void) {
    if (server) {
        ESP_LOGI(TAG, "Stopping HTTP Server");
        file_server_data_t* server_data = (file_server_data_t*) httpd_get_global_user_ctx(server);
        if (server_data) {
            free(server_data->scratch);
            free(server_data);
        }
        httpd_stop(server);
        server = NULL;
    }
}