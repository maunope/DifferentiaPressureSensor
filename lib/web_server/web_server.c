#include "web_server.h"
#include "esp_http_server.h"
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

/* Handler to serve the file list */
static esp_err_t file_list_handler(httpd_req_t *req) {
    char filepath[FILE_PATH_MAX];
    snprintf(filepath, sizeof(filepath), "/sdcard/");
    DIR *dir = opendir(filepath);
    if (!dir) {
        ESP_LOGE(TAG, "Failed to open directory: %s", filepath);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "text/html");
    httpd_resp_send_chunk(req, "<html><body><h1>Files on SD Card</h1><ul>", HTTPD_RESP_USE_STRLEN);

    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL) {
        if (entry->d_type == DT_REG) { // Only list regular files
            // Accommodate the base string, plus 2 * max filename length, plus null terminator
            char entry_path[128 + (2 * 256)];
            snprintf(entry_path, sizeof(entry_path), "<li><a href=\"/download?file=%s\">%s</a></li>", entry->d_name, entry->d_name);
            httpd_resp_send_chunk(req, entry_path, HTTPD_RESP_USE_STRLEN);
        }
    }
    closedir(dir);

    httpd_resp_send_chunk(req, "</ul></body></html>", HTTPD_RESP_USE_STRLEN);
    httpd_resp_send_chunk(req, NULL, 0); // Final chunk
    return ESP_OK;
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
        .handler   = file_list_handler,
        .user_ctx  = server_data
    };
    httpd_register_uri_handler(server, &file_list_uri);

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