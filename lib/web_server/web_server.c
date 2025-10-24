#include "web_server.h"
#include "esp_http_server.h"
#include "../buffers.h"
#include "esp_log.h"
#include "../../src/datalogger_task.h"
#include "esp_vfs.h"
#include <dirent.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <ctype.h> // For isspace
#include <regex.h> // For regular expressions
#include <stdlib.h>

static const char *TAG = "web_server";
static httpd_handle_t server = NULL;
static regex_t csv_header_regex;
static bool regex_is_compiled = false;

#define FILE_PATH_MAX (ESP_VFS_PATH_MAX + 128)
#define SCRATCH_BUFSIZE (10240)

typedef struct {
    char *scratch;
} file_server_data_t;

/* Forward declarations */
static esp_err_t api_preview_handler(httpd_req_t *req);

/**
 * @brief Trims leading and trailing whitespace from a string.
 * @param str The string to trim.
 * @return A pointer to the trimmed string (in-place modification).
 */
static char* trim_whitespace(char *str) {
    char *end;
    // Trim leading space
    while (isspace((unsigned char)*str)) str++;

    if (*str == 0)  // All spaces?
        return str;

    // Trim trailing space
    end = str + strlen(str) - 1;
    while (end > str && isspace((unsigned char)*end)) end--;

    // Write new null terminator
    *(end + 1) = '\0';
    return str;
}

/**
 * @brief Decodes a URL-encoded string in-place.
 *
 * Handles %xx encoding and replaces '+' with a space.
 * @param str The string to decode.
 */
static void url_decode(char *str) {
    char *p = str;
    char hex[3] = {0};
    while (*str) {
        if (*str == '%') {
            if (str[1] && str[2]) {
                hex[0] = str[1];
                hex[1] = str[2];
                *p++ = strtol(hex, NULL, 16);
                str += 3;
            } else {
                *p++ = *str++;
            }
        } else if (*str == '+') {
            *p++ = ' ';
            str++;
        } else {
            *p++ = *str++;
        }
    }
    *p = '\0';
}


/* Handler to serve the main index.html page */
/**
 * @brief HTTP GET handler for the root URL ("/").
 * 
 * Serves the main HTML page which contains the file browser UI and JavaScript logic.
 * @param req The HTTP request.
 * @return ESP_OK on success.
 */
static esp_err_t index_html_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");

    const char *index_html =
        "<!DOCTYPE html>"                                                                                                                                   \
        "<html lang=\"en\">"                                                                                                                                \
        "<head>"                                                                                                                                            \
            "<meta charset=\"UTF-8\">"                                                                                                                      \
            "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">"                                                                    \
            "<title>Differential Pressure Sensor</title>"                                                                                                  \
            "<style>"                                                                                                                                       \
                "body { font-family: -apple-system, BlinkMacSystemFont, \"Segoe UI\", Roboto, Helvetica, Arial, sans-serif; background-color: #121212; color: #e0e0e0; margin: 0; padding: 20px; line-height: 1.6; }" \
                ".container { max-width: 800px; margin: auto; background-color: #1e1e1e; padding: 20px; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.5); }" \
                "h1 { color: #bb86fc; border-bottom: 2px solid #373737; padding-bottom: 10px; }"                                                              \
                "ul { list-style-type: none; padding: 0; }"                                                                                                 \
                "li { background-color: #2c2c2c; margin-bottom: 10px; border-radius: 4px; display: flex; justify-content: space-between; align-items: center; }" \
                ".file-info { padding: 12px 15px; flex-grow: 1; min-width: 0; }"                                                                             \
                ".file-info a { text-decoration: none; color: #03dac6; font-weight: 500; font-size: 1.1em; display: block; word-break: break-all; }"          \
                ".file-info a:hover { text-decoration: underline; }"                                                                                        \
                ".file-metadata { font-size: 0.8em; color: #aaa; margin-top: 4px; }"                                                                         \
                ".actions { display: flex; align-items: stretch; flex-shrink: 0; }"                                                                         \
                "button, .btn { background-color: #373737; color: #e0e0e0; border: none; padding: 12px 15px; cursor: pointer; font-weight: bold; transition: background-color 0.2s ease-in-out; }" \
                "button.preview-btn { background-color: #03dac6; color: #121212; border-top-left-radius: 4px; border-bottom-left-radius: 4px; }"             \
                "button.preview-btn:hover { background-color: #33ffe7; }"                                                                                   \
                "button.delete-btn { background-color: #cf6679; color: #121212; border-radius: 0 4px 4px 0; }"                                               \
                "button.delete-btn:hover { background-color: #ff7991; }"                                                                                    \
                "#preview-container { display: none; margin-top: 20px; background-color: #2c2c2c; padding: 15px; border-radius: 8px; }"                      \
                "#preview-box { background-color: #121212; border: 1px solid #373737; max-height: 60vh; overflow-y: auto; position: relative; }"             \
                "#preview-table { width: 100%; border-collapse: collapse; font-family: monospace; font-size: 0.9em; }"                                       \
                "#preview-table thead { position: sticky; top: 0; background-color: #3a3a3a; }"                                                              \
                "#preview-table th, #preview-table td { padding: 6px 8px; border: 1px solid #373737; text-align: left; white-space: nowrap; }"               \
                "#preview-buttons { margin-top: 10px; }"                                                                                                    \
                "footer { text-align: center; margin-top: 30px; padding-top: 20px; border-top: 1px solid #373737; font-size: 0.9em; color: #888; }"          \
                "footer a { color: #03dac6; text-decoration: none; }"                                                                                       \
                "footer a:hover { text-decoration: underline; }"                                                                                            \
                "#file-list-status { color: #aaa; font-style: italic; }"                                                                                    \
            "</style>"                                                                                                                                      \
        "</head>"                                                                                                                                           \
        "<body>"                                                                                                                                            \
            "<div class=\"container\">"                                                                                                                     \
                "<h1>Available Data Files</h1>"                                                                                                             \
                "<div id=\"preview-container\" style=\"display: none;\">"                                                                                   \
                    "<h2>File Preview</h2>"                                                                                                                 \
                    "<div id=\"preview-box\">"                                                                                                              \
                        "<table id=\"preview-table\">"                                                                                                      \
                            "<thead></thead>"                                                                                                               \
                            "<tbody></tbody>"                                                                                                               \
                        "</table>"                                                                                                                          \
                    "</div>"                                                                                                                                \
                    "<div id=\"preview-buttons\">"                                                                                                          \
                        "<button onclick=\"jumpTo('top')\">Jump to Top</button>"                                                                            \
                        "<button onclick=\"navigateTime(-86400)\"> &lt;&lt; Day</button>"                                                                   \
                        "<button onclick=\"navigateTime(-3600)\"> &lt; Hour</button>"                                                                       \
                        "<button onclick=\"navigateTime(3600)\">Hour &gt; </button>"                                                                        \
                        "<button onclick=\"navigateTime(86400)\">Day &gt;&gt; </button>"                                                                    \
                        "<button onclick=\"jumpTo('bottom')\">Jump to Bottom</button>"                                                                      \
                        "<button id=\"close-preview-btn\">Close</button>"                                                                                   \
                    "</div>"                                                                                                                                \
                "</div>"                                                                                                                                    \
                "<ul id=\"file-list\"><p id=\"file-list-status\">Loading files...</p></ul>"                                                                  \
                "<footer>"                                                                                                                                  \
                    "<p><a href=\"" PROJECT_GITHUB_URL "\" target=\"_blank\">DifferentialPressureSensor Project on GitHub</a></p>" 
                "</footer>"                                                                                                                                 \
            "</div>"                                                                                                                                        \
            "<script>"                                                                                                                                      \
                "document.addEventListener('DOMContentLoaded', function() {"                                                                               \
                    "let currentFile = ''; let isLoading = false;"                                                                                          \
                    "const chunkSize = 100; const maxRows = 1000; const buffer = 200;"                                                                      \
                    "const previewContainer = document.getElementById('preview-container');"                                                               \
                    "const fileList = document.getElementById('file-list');"                                                                               \
                    "const previewBox = document.getElementById('preview-box');"                                                                           \
                    "const previewTable = document.getElementById('preview-table');"                                                                       \
                    "const tbody = previewTable.querySelector('tbody');"                                                                                   \
                    "function formatSize(bytes) {"                                                                                                         \
                        "if (bytes === 0) return '0 B';"                                                                                                    \
                        "const k = 1024;"                                                                                                                   \
                        "const sizes = ['B', 'KB', 'MB', 'GB'];"                                                                                           \
                        "const i = Math.floor(Math.log(bytes) / Math.log(k));"                                                                              \
                        "return parseFloat((bytes / Math.pow(k, i)).toFixed(2)) + ' ' + sizes[i];"                                                          \
                    "};"                                                                                                                                    \
                    "window.fetchChunk = function(file, start, count, timestamp, callback) {"                                                              \
                        "if (isLoading) return;"                                                                                                            \
                        "isLoading = true;"                                                                                                                 \
                        "let url = '/api/preview?file=' + encodeURIComponent(file) + '&count=' + count;"                                                    \
                        "if (timestamp !== null) { url += '&timestamp=' + timestamp; }"                                                                     \
                        "else if (start !== null) { url += '&start=' + start; }"                                                                            \
                        "fetch(url).then(function(res) {"                                                                                                   \
                            "const contentType = res.headers.get('content-type');"                                                                         \
                            "if (contentType && contentType.includes('application/json')) {"                                                                \
                                "return res.json().then(function(data) { return {isCsv: true, data: data}; });"                                            \
                            "} else {"                                                                                                                      \
                                "return res.text().then(function(data) { return {isCsv: false, data: data}; });"                                           \
                            "}"                                                                                                                             \
                        "})"                                                                                                                                \
                        ".then(function(response) {"                                                                                                        \
                            "isLoading = false; callback(response);"                                                                                        \
                        "}).catch(function(err) {"                                                                                                          \
                            "console.error('Fetch error:', err); isLoading = false;"                                                                        \
                        "});"                                                                                                                               \
                    "};"                                                                                                                                    \
                    "function renderRows(lines, prepend) {"                                                                                                \
                        "if (!lines || lines.length === 0) return;"                                                                                         \
                        "const fragment = document.createDocumentFragment();"                                                                              \
                        "for (var i = 0; i < lines.length; i++) {"                                                                                          \
                            "var line = lines[i];"                                                                                                          \
                            "if (!line) continue;"                                                                                                          \
                            "const row = document.createElement('tr');"                                                                                    \
                            "const cells = line.split(',');"                                                                                               \
                            "row.dataset.line = cells[0];"                                                                                                 \
                            "for (var j = 0; j < cells.length; j++) {"                                                                                      \
                                "const td = document.createElement('td');"                                                                                 \
                                "td.textContent = cells[j];"                                                                                               \
                                "row.appendChild(td);"                                                                                                     \
                            "}"                                                                                                                             \
                            "fragment.appendChild(row);"                                                                                                   \
                        "}"                                                                                                                                 \
                        "if (prepend) {"                                                                                                                    \
                            "tbody.insertBefore(fragment, tbody.firstChild);"                                                                              \
                            "while (tbody.rows.length > maxRows) { tbody.deleteRow(tbody.rows.length - 1); }"                                              \
                        "} else {"                                                                                                                          \
                            "tbody.appendChild(fragment);"                                                                                                  \
                            "while (tbody.rows.length > maxRows) { tbody.deleteRow(0); }"                                                                   \
                        "}"                                                                                                                                 \
                    "}"                                                                                                                                     \
                    "const rawTextDisplay = document.createElement('pre');"                                                                                \
                    "rawTextDisplay.id = 'raw-text-preview';"                                                                                              \
                    "rawTextDisplay.style.whiteSpace = 'pre-wrap';"                                                                                        \
                    "rawTextDisplay.style.wordBreak = 'break-all';"                                                                                        \
                    "rawTextDisplay.style.display = 'none';"                                                                                               \
                    "previewBox.appendChild(rawTextDisplay);"                                                                                              \
                    "window.previewFile = function(file) {"                                                                                                 \
                        "currentFile = file;"                                                                                                               \
                        "previewContainer.style.display = 'block';"                                                                                       \
                        "const title = document.querySelector('#preview-container h2');"                                                                   \
                        "title.textContent = 'Preview: ' + file;"                                                                                           \
                        "previewTable.querySelector('thead').innerHTML = '';"                                                                              \
                        "tbody.innerHTML = '<tr><td colspan=\"100\" style=\"text-align: center; padding: 20px;\">Loading...</td></tr>';"                      \
                        "document.querySelectorAll('#preview-buttons button:not(#close-preview-btn)').forEach(function(b) { b.style.display = 'none'; });" \
                        "window.fetchChunk(file, 0, chunkSize, null, function(response) {"                                                                 \
                            "document.getElementById('close-preview-btn').style.display = '';"                                                             \
                            "if (response.isCsv) {"                                                                                                         \
                                "previewTable.style.display = '';"                                                                                         \
                                "rawTextDisplay.style.display = 'none';"                                                                                   \
                                "const data = response.data;"                                                                                               \
                                "title.textContent += ' (CSV)';"                                                                                            \
                                "previewTable.querySelector('thead').innerHTML = '<tr>' + data.header.split(',').map(function(h) { return '<th>' + h + '</th>'; }).join('') + '</tr>';" \
                                "tbody.innerHTML = '';"                                                                                                    \
                                "renderRows(data.lines);"                                                                                                   \
                                "tbody.dataset.startLine = data.start_line;"                                                                                \
                                "tbody.dataset.endLine = data.start_line + data.lines.length;"                                                              \
                                "document.querySelectorAll('#preview-buttons button:not(#close-preview-btn)').forEach(function(b) { b.style.display = ''; });" \
                            "} else {"                                                                                                                      \
                                "previewTable.style.display = 'none';"                                                                                     \
                                "rawTextDisplay.style.display = 'block';"                                                                                  \
                                "document.querySelectorAll('#preview-buttons button:not(#close-preview-btn)').forEach(function(b) { b.style.display = 'none'; });" \
                                "title.textContent += ' (Raw Text)';"                                                                                       \
                                "rawTextDisplay.textContent = response.data;"                                                                              \
                            "}"                                                                                                                             \
                            "document.getElementById('close-preview-btn').onclick = function() { previewContainer.style.display = 'none'; previewTable.style.display=''; rawTextDisplay.style.display='none'; document.querySelectorAll('#preview-buttons button').forEach(function(b) { b.style.display = ''; }); };" \
                        "});"                                                                                                                               \
                    "};"                                                                                                                                    \
                    "function loadFiles() {"                                                                                                                \
                        "fileList.innerHTML = '<p id=\"file-list-status\">Loading files...</p>';"                                                            \
                        "fetch('/api/files')"                                                                                                               \
                            ".then(function(response) { return response.json(); })"                                                                         \
                            ".then(function(data) {"                                                                                                        \
                                "fileList.innerHTML = '';"                                                                                                 \
                                "if (data.files && data.files.length > 0) {"                                                                                \
                                    "data.files.sort(function(a, b) { return b.mtime - a.mtime; }).forEach(function(file) {"                               \
                                        "const li = document.createElement('li');"                                                                         \
                                        "const infoDiv = document.createElement('div');"                                                                   \
                                        "infoDiv.className = 'file-info';"                                                                                  \
                                        "const a = document.createElement('a');"                                                                           \
                                        "a.href = '/download?file=' + encodeURIComponent(file.name);"                                                       \
                                        "a.textContent = file.name;"                                                                                        \
                                        "const meta = document.createElement('div');"                                                                      \
                                        "const actionsDiv = document.createElement('div');"                                                                \
                                        "actionsDiv.className = 'actions';"                                                                                 \
                                        "const previewBtn = document.createElement('button');"                                                             \
                                        "previewBtn.textContent = 'Preview';"                                                                              \
                                        "previewBtn.className = 'preview-btn';"                                                                            \
                                        "previewBtn.onclick = function() { previewFile(file.name); };"                                                     \
                                        "meta.className = 'file-metadata';"                                                                                \
                                        "const date = new Date(file.mtime * 1000).toLocaleString(undefined, {dateStyle: 'short', timeStyle: 'medium'});"     \
                                        "meta.textContent = formatSize(file.size) + ' - ' + date;"                                                          \
                                        "const deleteBtn = document.createElement('button');"                                                              \
                                        "deleteBtn.textContent = 'Delete';"                                                                                \
                                        "deleteBtn.className = 'delete-btn';"                                                                              \
                                        "deleteBtn.onclick = function() {"                                                                                  \
                                            "if (confirm('Are you sure you want to delete ' + file.name + '?')) {"                                         \
                                                "fetch('/api/files/' + encodeURIComponent(file.name), { method: 'DELETE' })"                                \
                                                    ".then(function(response) {"                                                                           \
                                                        "if (response.ok) {"                                                                               \
                                                            "li.remove();"                                                                                 \
                                                        "} else {"                                                                                         \
                                                            "alert('Failed to delete file.');"                                                              \
                                                        "}"                                                                                                \
                                                    "});"                                                                                                   \
                                            "}"                                                                                                             \
                                        "};"                                                                                                                \
                                        "infoDiv.appendChild(a);"                                                                                           \
                                        "infoDiv.appendChild(meta);"                                                                                        \
                                        "actionsDiv.appendChild(previewBtn);"                                                                              \
                                        "actionsDiv.appendChild(deleteBtn);"                                                                               \
                                        "li.appendChild(infoDiv);"                                                                                          \
                                        "li.appendChild(actionsDiv);"                                                                                       \
                                        "fileList.appendChild(li);"                                                                                         \
                                    "});"                                                                                                                   \
                                "} else {"                                                                                                                  \
                                    "const li = document.createElement('li');"                                                                             \
                                    "li.textContent = 'No files found on SD card.';"                                                                       \
                                    "li.style.padding = '12px 15px';"                                                                                       \
                                    "fileList.appendChild(li);"                                                                                             \
                                "}"                                                                                                                         \
                            "})"                                                                                                                            \
                            ".catch(function(error) {"                                                                                                      \
                                "console.error('Error fetching file list:', error);"                                                                        \
                                "alert('Failed to load file list. The web server may have been turned off or disconnected from Wi-Fi.');"                   \
                                "fileList.innerHTML = '<p>Error loading file list. Please refresh the page.</p>';"                                          \
                            "});"                                                                                                                           \
                    "}"                                                                                                                                     \
                    "window.jumpTo = function(position) {"                                                                                                  \
                        "const start = (position === 'top') ? 0 : -1;"                                                                                      \
                        "window.fetchChunk(currentFile, start, chunkSize, null, function(response) {"                                                       \
                            "if (!response.isCsv) return;"                                                                                                  \
                            "const data = response.data;"                                                                                                   \
                            "tbody.innerHTML = '';"                                                                                                        \
                            "renderRows(data.lines, position === 'bottom');"                                                                               \
                            "tbody.dataset.startLine = data.start_line;"                                                                                    \
                            "tbody.dataset.endLine = data.start_line + data.lines.length;"                                                                  \
                            "if (position === 'bottom') { previewBox.scrollTop = previewBox.scrollHeight; }"                                                \
                            "else { previewBox.scrollTop = 0; }"                                                                                            \
                        "});"                                                                                                                               \
                    "};"                                                                                                                                    \
                    "window.navigateTime = function(offset) {"                                                                                              \
                        "const firstVisibleRow = tbody.rows[0];"                                                                                             \
                        "if (!firstVisibleRow) return;"                                                                                                     \
                        "const currentTimestamp = parseInt(firstVisibleRow.dataset.line, 10);"                                                             \
                        "const targetTimestamp = currentTimestamp + offset;"                                                                                \
                        "window.fetchChunk(currentFile, null, chunkSize, targetTimestamp, function(response) {"                                             \
                            "if (!response.isCsv) return;"                                                                                                  \
                            "const data = response.data;"                                                                                                   \
                            "tbody.innerHTML = '';"                                                                                                        \
                            "renderRows(data.lines);"                                                                                                       \
                            "tbody.dataset.startLine = data.start_line;"                                                                                    \
                            "tbody.dataset.endLine = data.start_line + data.lines.length;"                                                                  \
                            "previewBox.scrollTop = 0;"                                                                                                     \
                        "});"                                                                                                                               \
                    "};"                                                                                                                                    \
                    "previewBox.addEventListener('scroll', function() {"                                                                                    \
                        "if (previewBox.scrollTop < buffer && parseInt(tbody.dataset.startLine, 10) > 0) {"                                                 \
                            "const prevStart = Math.max(0, parseInt(tbody.dataset.startLine, 10) - chunkSize);"                                            \
                            "window.fetchChunk(currentFile, prevStart, chunkSize, null, function(r) { if(r.isCsv){renderRows(r.data.lines, true); tbody.dataset.startLine = r.data.start_line;} });" \
                        "} else if (previewBox.scrollTop + previewBox.clientHeight > previewBox.scrollHeight - buffer) {"                                   \
                            "const nextStart = parseInt(tbody.dataset.endLine, 10);"                                                                        \
                            "window.fetchChunk(currentFile, nextStart, chunkSize, null, function(r) { if(r.isCsv){renderRows(r.data.lines, false); tbody.dataset.endLine = r.data.start_line + r.data.lines.length;} });" \
                        "}"                                                                                                                                 \
                    "});"                                                                                                                                   \
                    "loadFiles();"                                                                                                                          \
                "});"                                                                                                                                       \
            "</script>"                                                                                                                                     \
        "</body>"                                                                                                                                           \
        "</html>";

    return httpd_resp_send(req, index_html, HTTPD_RESP_USE_STRLEN);
}

/* Handler to serve the file list as JSON */
/**
 * @brief HTTP GET handler for the `/api/files` endpoint.
 * 
 * Scans the `/sdcard/` directory and returns a JSON array of file objects,
 * each containing the file's name, size, and modification time.
 * @param req The HTTP request.
 * @return ESP_OK on success.
 */
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

/**
 * @brief HTTP DELETE handler for the /api/files/ endpoint.
 * 
 * Deletes a specified file from the SD card. This function does not perform the
 * deletion directly. Instead, it sends a `DATALOGGER_CMD_DELETE_FILE` command
 * to the `datalogger_task` and polls a status variable in the shared buffer for the result.
 * This ensures that all filesystem operations are centralized and thread-safe.
 */
static esp_err_t api_file_delete_handler(httpd_req_t *req)
{
    char filepath[FILE_PATH_MAX];
    // The filename is the part of the URI after the last '/'
    const char *filename = req->uri + strlen("/api/files/");

    // Basic security check: prevent path traversal
    if (strstr(filename, ".."))
    {
        ESP_LOGE(TAG, "Path traversal attempt detected in delete: %s", filename);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Bad Request");
        return ESP_FAIL;
    }

    snprintf(filepath, sizeof(filepath), "/sdcard/%s", filename);
    ESP_LOGI(TAG, "Web server requesting to delete file: %s", filepath);

    // 1. Set the command and filename in the shared buffer
    if (xSemaphoreTake(g_sensor_buffer_mutex, pdMS_TO_TICKS(100)))
    {
        strncpy(g_sensor_buffer.file_to_delete, filepath, sizeof(g_sensor_buffer.file_to_delete) - 1);
        g_sensor_buffer.delete_file_status = CMD_STATUS_PENDING;
        xSemaphoreGive(g_sensor_buffer_mutex);
    }
    else
    {
        ESP_LOGE(TAG, "Failed to acquire buffer mutex to request file deletion");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Server busy");
        return ESP_FAIL;
    }

    // 2. Send the command to the datalogger task
    datalogger_command_t cmd = DATALOGGER_CMD_DELETE_FILE;
    if (xQueueSend(g_datalogger_cmd_queue, &cmd, pdMS_TO_TICKS(100)) != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to send delete command to datalogger queue");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Server busy");
        return ESP_FAIL;
    }

    // 3. Poll for the result
    command_status_t status = CMD_STATUS_PENDING;
    for (int i = 0; i < 100; i++) { // Poll for up to 5 seconds (100 * 50ms)
        vTaskDelay(pdMS_TO_TICKS(50));
        if (xSemaphoreTake(g_sensor_buffer_mutex, pdMS_TO_TICKS(50))) {
            status = g_sensor_buffer.delete_file_status;
            xSemaphoreGive(g_sensor_buffer_mutex);
        }
        if (status == CMD_STATUS_SUCCESS || status == CMD_STATUS_FAIL) {
            break;
        }
    }

    if (status == CMD_STATUS_SUCCESS) {
        httpd_resp_set_status(req, "204 No Content");
        httpd_resp_send(req, NULL, 0);
        return ESP_OK;
    } else { // CMD_STATUS_FAIL or timeout (still PENDING)
        ESP_LOGE(TAG, "Failed to delete file: %s. Final status: %d", filepath, status);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to delete file");
        return ESP_FAIL;
    }
}

/* Handler to download a file */
/**
 * @brief HTTP GET handler for the `/download` endpoint.
 * 
 * Serves a specified file from the SD card for download. The filename is
 * passed as a URL query parameter (e.g., `/download?file=data.csv`).
 * @param req The HTTP request.
 * @return ESP_OK on success.
 */
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

/**
 * @brief Skips a specified number of lines in a file.
 * @param f The file pointer.
 * @param num_lines The number of lines to skip.
 * @return The actual number of lines skipped.
 */
static int skip_lines(FILE *f, int num_lines) {
    char temp_line[256];
    int skipped_count = 0;
    for (int i = 0; i < num_lines; i++) {
        if (fgets(temp_line, sizeof(temp_line), f) == NULL) {
            break; // Reached end of file
        }
        skipped_count++;
    }
    return skipped_count;
}

/**
 * @brief Counts the total number of lines in a file.
 * @param f The file pointer.
 * @return The total number of lines.
 */
static int count_lines(FILE *f) {
    char temp_line[256];
    int count = 0;
    long current_pos = ftell(f); // Save current position
    fseek(f, 0, SEEK_SET); // Go to beginning
    while (fgets(temp_line, sizeof(temp_line), f) != NULL) {
        count++;
    }
    fseek(f, current_pos, SEEK_SET); // Restore position
    return count;
}

/**
 * @brief Finds the line number (0-indexed after header) corresponding to or after a target timestamp.
 * Assumes the first column of each line is a UNIX timestamp.
 * @param f The file pointer (should be positioned after the header).
 * @param target_timestamp The UNIX timestamp to search for.
 * @return The 0-indexed line number relative to the start of data, or -1 if not found.
 */
static int find_line_by_timestamp(FILE *f, time_t target_timestamp) {
    char line_buf[256];
    long initial_pos = ftell(f);
    int line_idx = 0;

    while (fgets(line_buf, sizeof(line_buf), f) != NULL) {
        time_t line_timestamp = atol(line_buf); // Assuming first column is UNIX timestamp
        if (line_timestamp >= target_timestamp) {
            fseek(f, initial_pos, SEEK_SET); // Rewind to after header
            skip_lines(f, line_idx); // Seek to the found line
            return line_idx;
        }
        line_idx++;
    }
    return -1; // Timestamp not found
}

/**
 * @brief HTTP GET handler for the `/api/preview` endpoint.
 * 
 * Returns a specified number of lines from a text file, starting at a given
 * line number or timestamp. Used by the web UI to preview large CSV files without
 * downloading the entire file.
 * @param req The HTTP request.
 * @return ESP_OK on success.
 *
 * Query Parameters:
 * - `file`: (required) The name of the file to preview.
 * - `start`: The 0-indexed line number (after header) to start reading from. Default 0.
 *            If -1, reads from the end of the file.
 * - `count`: The number of lines to return. Default 100.
 * - `timestamp`: A UNIX timestamp. If provided, the server will search for the first line at or after this time.
 *                This parameter takes precedence over `start`.
 */
static esp_err_t api_preview_handler(httpd_req_t *req)
{
    char filepath[FILE_PATH_MAX];
    char filename[64] = {0};
    int requested_start_line = 0; // 0-indexed, after header
    int line_count = 100;         // Default to 100 lines
    time_t requested_timestamp = 0; // 0 indicates no timestamp search

    size_t query_len = httpd_req_get_url_query_len(req) + 1;
    if (query_len > 1) {
        char* query_buf = malloc(query_len);
        if (query_buf == NULL) {
            ESP_LOGE(TAG, "Failed to allocate query buffer");
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }
        if (httpd_req_get_url_query_str(req, query_buf, query_len) == ESP_OK) {
            char param[64];
            if (httpd_query_key_value(query_buf, "file", param, sizeof(param)) == ESP_OK) {
                strncpy(filename, param, sizeof(filename) - 1);
                url_decode(filename); // Decode the URL-encoded filename
            }
            if (httpd_query_key_value(query_buf, "start", param, sizeof(param)) == ESP_OK) {
                requested_start_line = atoi(param);
            }
            if (httpd_query_key_value(query_buf, "count", param, sizeof(param)) == ESP_OK) {
                int val = atoi(param);
                if (val > 0) line_count = val;
            }
            if (httpd_query_key_value(query_buf, "timestamp", param, sizeof(param)) == ESP_OK) {
                requested_timestamp = atol(param);
            }
        }
        free(query_buf);
    }

    if (strlen(filename) == 0) {
        ESP_LOGE(TAG, "Filename parameter is missing in preview request");
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    snprintf(filepath, sizeof(filepath), "/sdcard/%s", filename);

    FILE *f = fopen(filepath, "r");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for preview: %s", filepath);
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    char line[256];
    char header_line[256] = "";
    long header_pos = ftell(f); // Save position of file start

    // Read the first line to check for the CSV header.
    if (fgets(header_line, sizeof(header_line), f) == NULL) {
        fclose(f);
        httpd_resp_send_500(req);
        ESP_LOGE(TAG, "Failed to read header line from %s", filename);
        return ESP_FAIL;
    }
    header_line[strcspn(header_line, "\r\n")] = 0; // Trim newline characters from fgets
    char* trimmed_header = trim_whitespace(header_line); // Trim leading/trailing spaces

    // Execute the regular expression match
    if (!regex_is_compiled) {
        fclose(f);
        ESP_LOGE(TAG, "CSV header regex not compiled!");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    int reti = regexec(&csv_header_regex, trimmed_header, 0, NULL, 0);

    if (reti == REG_NOMATCH) { // reti != 0 means no match was found
        // --- Fallback to Plain Text Preview ---
        ESP_LOGI(TAG, "File '%s' does not match generic CSV header format. Sending as plain text.", filename);
        httpd_resp_set_type(req, "text/plain");

        // Rewind to the beginning of the file to send the content from the start
        fseek(f, header_pos, SEEK_SET);

        int lines_sent = 0;
        while (fgets(line, sizeof(line), f) != NULL && lines_sent < line_count) { // Use line_count for raw text too
            httpd_resp_send_chunk(req, line, strlen(line));
            lines_sent++;
        }
        if (!feof(f)) {
            httpd_resp_send_chunk(req, "\n(TRUNCATED)", 12);
        }
        httpd_resp_send_chunk(req, NULL, 0);
        fclose(f);
        return ESP_OK;
    }
    else if (reti != 0) {
        // An error occurred during regex execution
        char msgbuf[100];
        regerror(reti, &csv_header_regex, msgbuf, sizeof(msgbuf));
        ESP_LOGE(TAG, "Regex match failed: %s", msgbuf);
        fclose(f);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    // --- Proceed with JSON CSV Preview ---
    int actual_start_line_idx = 0;

    if (requested_timestamp != 0) {
        // Search by timestamp takes precedence
        int found_line_idx = find_line_by_timestamp(f, requested_timestamp);
        if (found_line_idx != -1) {
            actual_start_line_idx = found_line_idx;
        } else {
            // If timestamp not found, default to start of file
            fseek(f, 0, SEEK_SET);
            skip_lines(f, 1); // Skip header
            actual_start_line_idx = 0;
        }
    } else if (requested_start_line == -1) {
        // Request for end of file
        int total_lines = count_lines(f) - 1; // Subtract header line
        actual_start_line_idx = (total_lines > line_count) ? (total_lines - line_count) : 0;
        fseek(f, 0, SEEK_SET);
        skip_lines(f, 1 + actual_start_line_idx); // Skip header + lines
    } else {
        // Request for specific start line (or default 0)
        fseek(f, 0, SEEK_SET);
        skip_lines(f, 1 + requested_start_line); // Skip header + lines
        actual_start_line_idx = requested_start_line;
    }

    httpd_resp_set_type(req, "application/json");
    char json_start[512];
    snprintf(json_start, sizeof(json_start), "{\"header\":\"%s\",\"start_line\":%d,\"lines\":[", header_line, actual_start_line_idx);
    httpd_resp_send_chunk(req, json_start, strlen(json_start));

    bool first_data_chunk_line = true;
    int lines_sent = 0;
    while (fgets(line, sizeof(line), f) != NULL && lines_sent < line_count) {
        line[strcspn(line, "\r\n")] = 0; // Trim newline
        if (strlen(line) == 0) continue; // Skip empty lines

        if (!first_data_chunk_line) httpd_resp_send_chunk(req, ",", 1);
        httpd_resp_send_chunk(req, "\"", 1);
        httpd_resp_send_chunk(req, line, strlen(line));
        httpd_resp_send_chunk(req, "\"", 1);
        first_data_chunk_line = false;
        lines_sent++;
    }

    fclose(f);
    httpd_resp_send_chunk(req, "]}", 2); // Close JSON
    httpd_resp_send_chunk(req, NULL, 0); // Finalize
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

    // Compile the regex for CSV header detection at startup
    const char* csv_pattern = "^[a-zA-Z0-9_\\.\\s]+(,[a-zA-Z0-9_\\.\\s]+)*";
    int reg_err_code = regcomp(&csv_header_regex, csv_pattern, REG_EXTENDED | REG_NOSUB);
    if (reg_err_code != 0) {
        char msgbuf[100];
        regerror(reg_err_code, &csv_header_regex, msgbuf, sizeof(msgbuf));
        ESP_LOGE(TAG, "Could not compile regex for CSV header: %s", msgbuf);
        free(server_data->scratch);
        free(server_data);
        return ESP_FAIL;
    }
    regex_is_compiled = true;

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

    httpd_uri_t api_preview_uri = {
        .uri       = "/api/preview",
        .method    = HTTP_GET,
        .handler   = api_preview_handler,
        .user_ctx  = server_data
    };
    httpd_register_uri_handler(server, &api_preview_uri);

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
        if (regex_is_compiled) {
            regfree(&csv_header_regex);
            regex_is_compiled = false;
        }
        httpd_stop(server);
        server = NULL;
    }
}