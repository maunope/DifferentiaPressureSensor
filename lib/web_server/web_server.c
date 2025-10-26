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
#include <math.h> // For isnan()
#include "../ui/time_utils.h"

static const char *TAG = "web_server";
static httpd_handle_t server = NULL;
static regex_t csv_header_regex;
static bool regex_is_compiled = false;

#define FILE_PATH_MAX (ESP_VFS_PATH_MAX + 128)
#define SCRATCH_BUFSIZE (10240)

typedef struct
{
    char *scratch;
} file_server_data_t;

/* Forward declarations */
static esp_err_t api_preview_handler(httpd_req_t *req);
static esp_err_t api_sensordata_handler(httpd_req_t *req);
static esp_err_t api_fileinfo_handler(httpd_req_t *req);

/**
 * @brief Trims leading and trailing whitespace from a string.
 * @param str The string to trim.
 * @return A pointer to the trimmed string (in-place modification).
 */
static char *trim_whitespace(char *str)
{
    char *end;
    // Trim leading space
    while (isspace((unsigned char)*str))
        str++;

    if (*str == 0) // All spaces?
        return str;

    // Trim trailing space
    end = str + strlen(str) - 1;
    while (end > str && isspace((unsigned char)*end))
        end--;

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
static void url_decode(char *str)
{
    char *p = str;
    char hex[3] = {0};
    while (*str)
    {
        if (*str == '%')
        {
            if (str[1] && str[2])
            {
                hex[0] = str[1];
                hex[1] = str[2];
                *p++ = strtol(hex, NULL, 16);
                str += 3;
            }
            else
            {
                *p++ = *str++;
            }
        }
        else if (*str == '+')
        {
            *p++ = ' ';
            str++;
        }
        else
        {
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
static esp_err_t index_html_handler(httpd_req_t *req)
{
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
        ".status-banner { background-color: #cf6679; color: #121212; padding: 10px; border-radius: 4px; text-align: center; font-weight: bold; display: none; align-items: center; justify-content: center; margin-bottom: 20px; }"
        ".status-banner span { flex-grow: 1; text-align: center; }"
        ".status-banner button { margin-left: 15px; font-size: 0.9em; border-radius: 4px; }"
        ".sensor-panel { background-color: #2c2c2c; margin-bottom: 20px; padding: 15px; border-radius: 8px; }"
        "#preview-container { position: relative; }" /* For absolute positioning of close button */
        ".sensor-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; }"
        ".sensor-panel h3 { color: #e0e0e0; margin-top: 0; margin-bottom: 15px; border-bottom: 1px solid #373737; padding-bottom: 10px; }"
        ".sensor-item { background-color: #373737; padding: 10px; border-radius: 4px; }"
        ".sensor-item h3 { margin-top: 0; font-size: 0.9em; color: #aaa; text-transform: uppercase; }"
        ".file-info { padding: 12px 15px; flex-grow: 1; min-width: 0; }"
        ".file-info a { text-decoration: none; color: #03dac6; font-weight: 500; font-size: 1.1em; display: block; word-break: break-all; }"
        "#sparkline-container { display: none; grid-template-columns: repeat(auto-fit, minmax(200px, 1fr)); gap: 10px; margin-bottom: 15px; }"
        ".sparkline-item { background-color: #373737; padding: 10px; border-radius: 4px; }"
        ".sparkline-item h4 { margin: 0 0 5px 0; font-size: 0.9em; color: #aaa; text-transform: uppercase; }"
        "#sparkline-timestamp-range { font-size: 0.8em; color: #aaa; margin-bottom: 10px; display: none; align-items: center; justify-content: space-between; gap: 10px; }"
        ".timeline-container { flex-grow: 1; background-color: #373737; height: 18px; border-radius: 9px; display: flex; align-items: center; justify-content: flex-start; padding: 2px; }"
        "#timeline-bar { background-color: #03dac6; height: 100%; border-radius: 7px; text-align: center; color: #121212; font-weight: bold; font-size: 0.9em; line-height: 14px; white-space: nowrap; overflow: hidden; position: absolute; transition: width 0.3s ease, left 0.3s ease; }"
        "#sparkline-start-ts, #sparkline-end-ts { white-space: nowrap; }"
        ".sparkline-item canvas { width: 100%; height: 40px; }"
        ".file-info a:hover { text-decoration: underline; }"
        ".file-metadata { font-size: 0.8em; color: #aaa; margin-top: 4px; }"
        ".actions { display: flex; align-items: stretch; flex-shrink: 0; padding-right: 15px; }"
        "button.btn-primary { background-color: #03dac6; color: #121212; border-radius: 4px; }" /* This is a general purpose primary button */
        "button.btn-primary:hover { background-color: #33ffe7; }"
        "button, .btn { background-color: #373737; color: #e0e0e0; border: none; padding: 12px 15px; cursor: pointer; font-weight: bold; transition: background-color 0.2s ease-in-out; }"
        "button.preview-btn { background-color: #03dac6; color: #121212; border-radius: 4px; margin-right: 5px; }"
        "button.preview-btn:hover { background-color: #33ffe7; }"
        "button.delete-btn { background-color: #cf6679; color: #121212; border-radius: 4px; }"
        "button.delete-btn:hover { background-color: #ff7991; }"
        "#preview-box { background-color: #121212; border: 1px solid #373737; max-height: 60vh; overflow-y: auto; position: relative; }"
        "li.active-preview { background-color: #4a4a4a; border-radius: 4px; }"
        ".timeline-container { position: relative; }" /* Add this */
        "#controls-panel { background-color: #373737; padding: 10px; border-radius: 4px; margin-bottom: 10px; }"
        "#controls-panel h4 { margin: 0 0 10px 0; font-size: 0.9em; color: #aaa; text-transform: uppercase; border-bottom: 1px solid #555; padding-bottom: 5px; }"
        "#preview-table { width: 100%; border-collapse: collapse; font-family: monospace; font-size: 0.9em; }"
        "#preview-table thead { position: sticky; top: 0; background-color: #373737; }"
        "#preview-table th, #preview-table td { padding: 8px 10px; border: 1px solid #444; text-align: left; white-space: nowrap; }"
        "#scale-buttons { display: flex; flex-wrap: wrap; gap: 5px; margin-bottom: 10px; }"
        "#scale-buttons button { flex-grow: 1; border-radius: 4px; background-color: #373737; color: #e0e0e0; }"
        "#scale-buttons button.active { background-color: #bb86fc; color: #121212; }"
        "#preview-buttons { display: flex; flex-wrap: wrap; gap: 5px; }"
        "#preview-buttons button { flex-grow: 1; border-radius: 4px; background-color: #03dac6; color: #121212; }"
        "#close-preview-btn { position: absolute; top: 10px; right: 10px; background-color: #cf6679; color: #121212; border-radius: 4px; z-index: 10; }"
        "#loading-indicator { display: none; align-items: center; justify-content: center; gap: 10px; text-align: center; color: #aaa; margin-top: 10px; min-height: 1em; }"
        ".spinner { width: 1.5em; height: 1.5em; transform-origin: center; animation: spinner_zKoa 2s linear infinite; }"
        ".spinner circle { stroke-linecap: round; animation: spinner_YpZS 1.5s ease-in-out infinite; }"
        "@keyframes spinner_zKoa { 100% { transform: rotate(360deg); } }"
        "@keyframes spinner_YpZS { 0% { stroke-dasharray: 0 150; stroke-dashoffset: 0; } 47.5% { stroke-dasharray: 42 150; stroke-dashoffset: -16; } 95%, 100% { stroke-dasharray: 42 150; stroke-dashoffset: -59; } }"
        "#preview-buttons button:hover { background-color: #33ffe7; }"
        "#preview-buttons #close-preview-btn { background-color: #cf6679; }"
        "#preview-buttons #close-preview-btn:hover { background-color: #ff7991; }"
        "footer { text-align: center; margin-top: 30px; padding-top: 20px; border-top: 1px solid #373737; font-size: 0.9em; color: #888; }"
        "footer a { color: #03dac6; text-decoration: none; }"
        "footer a:hover { text-decoration: underline; }"
        "#file-list-status { color: #aaa; font-style: italic; }"
        "</style>"
        "<style>#preview-title { padding-right: 80px; word-break: break-all; }</style>"
        "</head>"
        "<body>"
        "<div class=\"container\">"
        "<div id=\"global-status-banner\" class=\"status-banner\"><span id=\"global-status-message\"></span><button id=\"global-retry-btn\" class=\"btn-primary\"></button></div>"
        "<h1>Live Data</h1>"
        "<div class=\"sensor-panel\">"
        "<div class=\"sensor-grid\">"
        "<div class=\"sensor-item\"><h3>Timestamp</h3><p id=\"sensor-timestamp\">-</p></div>"
        "<div class=\"sensor-item\"><h3>Temperature</h3><p id=\"sensor-temp\">-</p></div>"
        "<div class=\"sensor-item\"><h3>Pressure</h3><p id=\"sensor-press\">-</p></div>"
        "<div class=\"sensor-item\"><h3>Diff. Pressure</h3><p id=\"sensor-diff-press\">-</p></div>"
        "<div class=\"sensor-item\"><h3>Battery</h3><p id=\"sensor-batt\">-</p></div>"
        "</div></div>" /* Ends sensor-panel and sensor-grid */
        "<h1>File Browser</h1>"
        "<div id=\"preview-container\" class=\"sensor-panel\" style=\"display: none; margin-top: 20px;\">"
        "<h3 id=\"preview-title\">File Preview</h3>"
        "<button id=\"close-preview-btn\">Close</button>" /* Moved here */
        "<div id=\"sparkline-timestamp-range\">"
        "  <span id=\"sparkline-start-ts\"></span>"
        "  <div class=\"timeline-container\">"
        "    <div id=\"timeline-bar\"></div>"
        "  </div>"
        "  <span id=\"sparkline-end-ts\"></span>"
        "</div>"
        "<div id=\"sparkline-container\">"
        "  <div class=\"sparkline-item\"><h4>Temperature</h4><canvas id=\"spark-temp\"></canvas></div>"
        "  <div class=\"sparkline-item\"><h4>Pressure</h4><canvas id=\"spark-press\"></canvas></div>"
        "  <div class=\"sparkline-item\"><h4>Diff. Pressure</h4><canvas id=\"spark-diff-press\"></canvas></div>"
        "  <div class=\"sparkline-item\"><h4>Battery</h4><canvas id=\"spark-batt\"></canvas></div>"
        "</div>"
        "<div id=\"controls-panel\">"
        "  <h4>Controls</h4>"
        "  <div id=\"scale-buttons\">"
        "    <button data-scale=\"3600\">1 hour</button>"
        "    <button data-scale=\"14400\">4 hours</button>"
        "    <button data-scale=\"43200\">12 hours</button>"
        "    <button data-scale=\"86400\">24 hours</button>"
        "    <button data-scale=\"172800\">48 hours</button>"
        "    <button data-scale=\"-1\">Max</button>"
        "  </div>"
        "  <div id=\"preview-buttons\">"
        "    <button onclick=\"jumpTo('top')\">&lt;&lt;&lt; head</button><button onclick=\"navigateTime(-86400)\"> &lt;&lt; Day</button><button onclick=\"navigateTime(-3600)\"> &lt; Hour</button><button onclick=\"navigateTime(3600)\">Hour &gt; </button><button onclick=\"navigateTime(86400)\">Day &gt;&gt; </button><button onclick=\"jumpTo('bottom')\">tail &gt;&gt;&gt;</button>"
        "  </div>"
        "  <div id=\"loading-indicator\">"
        "    <svg class=\"spinner\" viewBox=\"0 0 24 24\" xmlns=\"http://www.w3.org/2000/svg\"><g><circle cx=\"12\" cy=\"12\" r=\"9.5\" fill=\"none\" stroke-width=\"3\" stroke=\"#03dac6\"></circle></g></svg>"
        "    <span>Loading...</span>"
        "  </div>"
        "</div>"
        "<div id=\"preview-box\">"
        "<table id=\"preview-table\">"
        "<thead></thead>"
        "<tbody></tbody>"
        "</table>"
        "</div>" /* Ends preview-box */
        "</div>" /* Ends preview-container */
        "<ul id=\"file-list\"><p id=\"file-list-status\">Loading files...</p></ul>"
        "<footer>"
        "<p><a href=\"" PROJECT_GITHUB_URL "\" target=\"_blank\">DifferentialPressureSensor Project on GitHub</a></p>"
        "</footer>"
        "</div>"
        "<script>"
        "document.addEventListener('DOMContentLoaded', function() {"
        "let currentFile = ''; let isLoading = false; let sensorIntervalId = null;"
        "let currentScale = 14400; /* Default to 4 hours */ let fileInfo = {}; let lastChunkInfo = {};"
        "const maxRows = 10000; const buffer = 200;"
        "const previewContainer = document.getElementById('preview-container');"
        "const closePreviewBtn = document.getElementById('close-preview-btn');"
        "function closePreview() {"
        "    previewContainer.style.display = 'none';"
        "    currentFile = '';"
        "    setActiveFile(null);"
        "    document.getElementById('sparkline-container').style.display = 'none';"
        "    document.getElementById('sparkline-timestamp-range').style.display = 'none';"
        "}"
        "const tbody = document.querySelector('#preview-table tbody');"
        "const fileList = document.getElementById('file-list');"
        "const previewBox = document.getElementById('preview-box');"
        "function updateSensorData() {"
        "    document.getElementById('global-status-banner').style.display = 'none';"
        "    fetch('/api/sensordata').then(r => r.json()).then(data => {"
        "        const timestampEl = document.getElementById('sensor-timestamp'); if(timestampEl) timestampEl.textContent = data.datetime_local || 'N/A';"
        "        const tempEl = document.getElementById('sensor-temp'); if(tempEl) tempEl.textContent = data.temperature_c !== null ? data.temperature_c.toFixed(2) + ' C' : 'N/A';"
        "        const pressEl = document.getElementById('sensor-press'); if(pressEl) pressEl.textContent = data.pressure_kpa !== 0 ? data.pressure_kpa.toFixed(3) + ' kPa' : 'N/A';"
        "        const diffPressEl = document.getElementById('sensor-diff-press'); if(diffPressEl) diffPressEl.textContent = data.diff_pressure_pa !== null ? data.diff_pressure_pa.toFixed(2) + ' Pa' : 'N/A';"
        "        let batt_str = 'N/A';"
        "        if (data.battery_voltage !== null) {"
        "           batt_str = `${data.battery_voltage.toFixed(2)}V ${data.battery_percentage}%`;"
        "           if (data.battery_externally_powered) { batt_str += ' (Charging)'; }"
        "        }"
        "        const battEl = document.getElementById('sensor-batt'); if(battEl) battEl.textContent = batt_str;"
        "    }).catch(err => {"
        "       console.error('Error fetching sensor data:', err);"
        "       showGlobalBanner('Failed to load live data. Server may be offline.', startSensorRefresh);"
        "     ['sensor-timestamp', 'sensor-temp', 'sensor-press', 'sensor-diff-press', 'sensor-batt'].forEach(id => { const el = document.getElementById(id); if(el) el.textContent = 'N/A'; });"
        "       if (sensorIntervalId) { clearInterval(sensorIntervalId); sensorIntervalId = null; }"
        "    });"
        "}"
        "function showGlobalBanner(message, retryAction) {"
        "    document.getElementById('global-status-message').textContent = message;"
        "    const retryBtn = document.getElementById('global-retry-btn');"
        "    retryBtn.textContent = 'Retry';"
        "    retryBtn.onclick = retryAction;"
        "    document.getElementById('global-status-banner').style.display = 'flex';"
        "}"
        "window.startSensorRefresh = function() {"
        "    document.getElementById('global-status-banner').style.display = 'none';"
        "    if (sensorIntervalId) {"
        "        clearInterval(sensorIntervalId);"
        "    }"
        "    updateSensorData();"
        "    sensorIntervalId = setInterval(updateSensorData, 5000);"
        "};"

        "function drawSparkline(canvasId, data, color, unit, fixedMin, fixedMax) {"
        "    const canvas = document.getElementById(canvasId);"
        "    if (!canvas) return;"
        "    canvas.width = canvas.clientWidth;"
        "    canvas.height = canvas.clientHeight;"
        "    const ctx = canvas.getContext('2d');"
        "    const width = canvas.width; "
        "    const height = canvas.clientHeight;"
        "    const leftMargin = 60;"
        "    ctx.clearRect(0, 0, width, height);"
        "    if (!data || data.length < 2) { return; }"
        "    let minVal = (fixedMin !== undefined) ? fixedMin : data.reduce((min, v) => Math.min(min, v), Infinity);"
        "    let maxVal = (fixedMax !== undefined) ? fixedMax : data.reduce((max, v) => Math.max(max, v), -Infinity);"
        "    let range = maxVal - minVal;"
        "    if (range < 0.1) { "
        "        const mid = (minVal + maxVal) / 2;"
        "        minVal = mid - 0.1;"
        "        maxVal = mid + 0.1;"
        "        range = 0.2;"
        "    }"
        "    const precision = (unit === 'kPa') ? 2 : 1;"
        "    ctx.font = '10px monospace';"
        "    ctx.fillStyle = '#888';"
        "    ctx.textAlign = 'right';"
        "    ctx.fillText(maxVal.toFixed(precision) + unit, leftMargin - 5, 10);"
        "    ctx.fillText(minVal.toFixed(precision) + unit, leftMargin - 5, height - 2);"
        "    if (minVal < 0 && maxVal > 0) {"
        "        const zeroY = height - ((0 - minVal) / range) * height;"
        "        ctx.beginPath();"
        "        ctx.strokeStyle = '#888'; /* Lighter grey line for zero level */"
        "        ctx.lineWidth = 1;"
        "        ctx.moveTo(leftMargin, zeroY);"
        "        ctx.lineTo(width, zeroY);"
        "        ctx.stroke();"
        "    }"
        "    const graphWidth = width - leftMargin;"
        "    const xStep = graphWidth / (data.length - 1);"
        "    ctx.beginPath();"
        "    ctx.strokeStyle = color;"
        "    ctx.lineWidth = 1.5;"
        "    ctx.moveTo(leftMargin, height - ((data[0] - minVal) / range) * height);"
        "    for (let i = 0; i < data.length; i++) {"
        "        const x = leftMargin + i * xStep;"
        "        const y = height - ((data[i] - minVal) / range) * height;"
        "        ctx.lineTo(x, y);"
        "    } "
        "    ctx.stroke();"
        "    return {min: minVal, max: maxVal};"
        "}"
        "function resizeCanvases() {"
        "    ['spark-temp', 'spark-press', 'spark-diff-press', 'spark-batt'].forEach(id => {"
        "        const canvas = document.getElementById(id); if (!canvas) return;"
        "        canvas.width = canvas.clientWidth; canvas.height = canvas.clientHeight;"
        "    });"
        "}"
        "function updateSparklines() {"
        "    const rows = Array.from(tbody.rows);"
        "    if (rows.length < 2) { document.getElementById('sparkline-container').style.display = 'none'; document.getElementById('sparkline-timestamp-range').style.display = 'none'; return; }"
        "    document.getElementById('sparkline-timestamp-range').style.display = 'block';"
        "    const headerRow = previewTable.querySelector('thead tr');"
        "    if (!headerRow) { console.warn('updateSparklines called before header was ready.'); return; }"
        "    const headerCells = Array.from(headerRow.cells).map(th => th.textContent);"
        "    const getColumnData = (colName) => {"
        "        const idx = headerCells.indexOf(colName);"
        "        return idx === -1 ? [] : rows.map(row => parseFloat(row.cells[idx].textContent)).filter(v => !isNaN(v));"
        "    };"
        "    const firstRow = rows[0]; const lastRow = rows[rows.length - 1];"
        "    const chunkStartTimestamp = parseInt(firstRow.dataset.timestamp, 10);"
        "    const chunkEndTimestamp = parseInt(lastRow.dataset.timestamp, 10);"
        "    const fileStartTimestamp = fileInfo[currentFile]?.first_ts;"
        "    const fileEndTimestamp = fileInfo[currentFile]?.last_ts;"
        "    if (!fileStartTimestamp || !fileEndTimestamp) return;"
        "    const fileTotalDuration = Math.max(1, fileEndTimestamp - fileStartTimestamp);"
        "    const chunkDurationSec = chunkEndTimestamp - chunkStartTimestamp;"
        "    /* For very small files, the chunk duration can be the total duration */"
        "    const displayDuration = Math.min(chunkDurationSec, fileTotalDuration);"
        "    /* Ensure chunk duration is at least 1 second to avoid division by zero if timestamps are identical*/"
        "    const effectiveChunkDuration = Math.max(1, chunkDurationSec);"

        "    document.getElementById('sparkline-start-ts').textContent = new Date(fileStartTimestamp * 1000).toLocaleString([], {month: 'short', day: 'numeric', hour: '2-digit', minute:'2-digit'});"
        "    document.getElementById('sparkline-end-ts').textContent = new Date(fileEndTimestamp * 1000).toLocaleString([], {month: 'short', day: 'numeric', hour: '2-digit', minute:'2-digit'});"
        "    function formatDuration(seconds, useShort) {"
        "        if (seconds < 3600) return `${Math.round(seconds / 60)} ` + (useShort ? 'm' : 'min');"
        "        if (seconds < 86400) {"
        "            const totalMinutes = Math.round(seconds / 60);"
        "            const h = Math.floor(totalMinutes / 60);"
        "            const m = totalMinutes % 60;"
        "            if (h > 0 && m > 0) return useShort ? `${h}h${m}m` : `${h} hour${h > 1 ? 's' : ''} ${m} min`;"
        "            return `${h > 0 ? h : m} ` + (useShort ? (h > 0 ? 'h' : 'm') : (h > 0 ? (h === 1 ? 'hour' : 'hours') : 'min'));"
        "        }"
        "        const d = seconds / 86400; return `${d.toFixed(d < 10 ? 1 : 0)} ` + (useShort ? 'd' : (d === 1 ? 'day' : 'days'));"
        "    }"
        "    const timelineBar = document.getElementById('timeline-bar');"
        "    const leftPercent = ((chunkStartTimestamp - fileStartTimestamp) / fileTotalDuration) * 100;"
        "    const idealWidthPercent = (displayDuration / fileTotalDuration) * 100;"
        "    const remainingPercent = 100 - leftPercent;"
        "    const finalWidthPercent = Math.min(idealWidthPercent, remainingPercent);"

        "    timelineBar.style.left = `${leftPercent}%`;"
        "    timelineBar.style.width = `${finalWidthPercent}%`;"
        "    timelineBar.style.minWidth = '0';" /* Reset minWidth before calculating */

        "    /* Only apply minimum width if it doesn't cause an overshoot */"
        "    const minWidthPercent = 5;"
        "    if (finalWidthPercent >= minWidthPercent || leftPercent + minWidthPercent > 100) {"
        "        timelineBar.textContent = formatDuration(displayDuration, finalWidthPercent < 10);"
        "    } else {"
        "        timelineBar.style.width = `${minWidthPercent}%`;"
        "        timelineBar.textContent = '';" /* Hide text if bar is too small */
        "    }"

        "    document.getElementById('sparkline-timestamp-range').style.display = 'flex';"
        "    const tempData = getColumnData('temperature_c');"
        "    const pressData = getColumnData('pressure_kpa');"
        "    const diffPressData = getColumnData('diff_pressure_pa');"
        "    const battData = getColumnData('battery_voltage');"
        "    const maxAbsDiffPress = diffPressData.length > 0 ? diffPressData.reduce((max, v) => Math.max(max, Math.abs(v)), 0) : 1;"
        "    const maxBatt = battData.length > 0 ? battData.reduce((max, v) => Math.max(max, v), 0) : 4.2;"
        "    resizeCanvases();"
        "    document.getElementById('sparkline-container').style.display = 'grid';"
        "    drawSparkline('spark-temp', tempData, '#03dac6', 'C');"
        "    drawSparkline('spark-press', pressData, '#bb86fc', 'kPa');"
        "    drawSparkline('spark-diff-press', diffPressData, '#cf6679', 'Pa', -maxAbsDiffPress, maxAbsDiffPress);"
        "    drawSparkline('spark-batt', battData, '#03dac6', 'V', 3.0, Math.max(4.2, maxBatt));"
        "}"
        "startSensorRefresh();"
        "const loadingIndicator = document.getElementById('loading-indicator');"
        "const previewTable = document.getElementById('preview-table');" /* Keep this reference*/
        "function formatSize(bytes) {"
        "if (bytes === 0) return '0 B';"
        "const k = 1024;"
        "const sizes = ['B', 'KB', 'MB', 'GB'];"
        "const i = Math.floor(Math.log(bytes) / Math.log(k));"
        "return parseFloat((bytes / Math.pow(k, i)).toFixed(2)) + ' ' + sizes[i];"
        "};"
        "window.fetchChunk = function(file, start, count, timestamp, duration, direction, callback) {"
        "if (isLoading) return;"
        "isLoading = true;"
        "loadingIndicator.style.display = 'flex';"
        "let url = `/api/preview?file=${encodeURIComponent(file)}&count=${maxRows}&duration=${duration}`;"
        "if (timestamp !== null) { url += '&timestamp=' + timestamp; }"
        "else if (start !== null) { url += '&start=' + start; }"
        "if (direction) { url += '&direction=' + direction; }"
        "fetch(url).then(function(res) {"
        "const contentType = res.headers.get('content-type');"
        "if (contentType && contentType.includes('application/json')) {"
        "return res.json().then(function(data) { return {isCsv: true, data: data}; });"
        "} else {"
        "return res.text().then(function(data) { return {isCsv: false, data: data}; });"
        "}"
        "})"
        ".then(function(response) {"
        "isLoading = false; loadingIndicator.style.display = 'none'; callback(response);"
        "}).catch(function(err) {"
        "console.error('Fetch error:', err); isLoading = false; loadingIndicator.style.display = 'none';"
        "});};"
        "function renderRows(data, chunkStartLine) {"
        "    const lines = data.lines;" /* Assumes header is already rendered*/
        "if (!lines || lines.length === 0) return;"
        "previewBox.style.display = '';" /* Show the table container now that we have data */
        "const fragment = document.createDocumentFragment();"
        "lastChunkInfo.start_line = chunkStartLine;"
        "/* The header is now rendered separately by renderHeader()*/"
        "for (let i = 0; i < lines.length; i++) {"
        "var line = lines[i];"
        "if (!line) continue;"
        "const row = document.createElement('tr');"
        "const cells = line.split(',');"
        "row.dataset.line = (chunkStartLine + i).toString();"
        "for (var j = 0; j < cells.length; j++) {"
        "if (j === 0) { "
        "    row.dataset.timestamp = cells[j]; "
        "    if (i === 0) { lastChunkInfo.start_ts = parseInt(cells[j], 10); } "
        "}"
        "const td = document.createElement('td');"
        "td.textContent = cells[j];"
        "row.appendChild(td);"
        "}"
        "fragment.appendChild(row);"
        "}"
        "tbody.innerHTML = '';"
        "tbody.appendChild(fragment);"
        "updateSparklines();"
        "document.querySelectorAll('#preview-buttons button, #scale-buttons button').forEach(b => b.style.display = '');"
        "}"
        "function renderHeader(headerString) {"
        "    if (!headerString) return;"
        "    previewTable.querySelector('thead').innerHTML = '<tr>' + headerString.split(',').map(function(h) { return '<th>' + h + '</th>'; }).join('') + '</tr>';"
        "}"
        "const rawTextDisplay = document.createElement('pre');"
        "rawTextDisplay.id = 'raw-text-preview';"
        "rawTextDisplay.style.whiteSpace = 'pre-wrap';"
        "rawTextDisplay.style.wordBreak = 'break-all';"
        "rawTextDisplay.style.display = 'none';"
        "previewBox.appendChild(rawTextDisplay);"
        "function setActiveFile(filename) {"
        "    document.querySelectorAll('#file-list li').forEach(li => li.classList.remove('active-preview'));"
        "    if (filename) { const el = document.querySelector(`li[data-filename=\"${filename}\"]`); if (el) el.classList.add('active-preview'); }"
        "}"
        "function updateScaleButtons() {"
        "    document.querySelectorAll('#scale-buttons button').forEach(b => {"
        "        b.classList.toggle('active', parseInt(b.dataset.scale, 10) === currentScale);"
        "    });"
        "}"
        "window.previewFile = async function(file) {"
        "    currentFile = file; previewContainer.style.display = 'block';"
        "    const title = document.getElementById('preview-title');"
        "    title.textContent = 'Preview: ' + file;"
        "    previewBox.style.display = 'none';" /* Hide table container while loading */
        "    tbody.innerHTML = '<tr><td colspan=\"100\" style=\"text-align: center; padding: 20px;\">Loading...</td></tr>';"
        "    document.querySelectorAll('#preview-buttons button, #scale-buttons button').forEach(b => b.style.display = 'none');"
        "    setActiveFile(file);"
        "    updateScaleButtons();"
        "    currentScale = 14400; /* Reset to default 4 hours before fetching info */"
        "    try {"
        "        const fileInfoRes = await fetch(`/api/fileinfo?file=${encodeURIComponent(file)}`);"
        "        if (!fileInfoRes.ok) { throw new Error('Failed to fetch file info'); }"
        "        fileInfo[file] = await fileInfoRes.json();"
        "        const duration = fileInfo[file].last_ts - fileInfo[file].first_ts;"
        "        if (duration < 3600) { currentScale = 1800; } /* < 1hr -> 30min */"
        "        else if (duration < 7200) { currentScale = 3600; } /* < 2hr -> 1hr */"
        "        else if (duration < 14400) { currentScale = 7200; } /* < 4hr -> 2hr */"
        "        else { currentScale = 14400; } /* default 4hr */"
        "        updateScaleButtons();"
        "        /* Now that we have info and scale, fetch the first chunk */"
        "        window.fetchChunk(file, 0, maxRows, null, currentScale, 'forward', function(response) {"
        "            if (!response.isCsv) {"
        "                previewTable.style.display = 'none'; rawTextDisplay.style.display = 'block';"
        "                document.querySelectorAll('#preview-buttons button, #scale-buttons button').forEach(b => b.style.display = 'none');"
        "                title.textContent = 'Preview: ' + file + ' (Raw Text)';"
        "                rawTextDisplay.textContent = response.data;"
        "                return;"
        "            }"
        "            previewTable.style.display = ''; rawTextDisplay.style.display = 'none';"
        "            title.textContent = 'Preview: ' + file + ' (CSV)';"
        "            renderHeader(response.data.header);"
        "            renderRows(response.data, response.data.start_line); /* This will now call updateSparklines */"
        "        });"
        "    } catch (e) {"
        "        console.error('Could not fetch file info or initial chunk', e);"
        "        tbody.innerHTML = '<tr><td colspan=\"100\" style=\"text-align: center; padding: 20px;\">Error loading file.</td></tr>';"
        "        fileInfo[file] = null;"
        "    }"
        "    closePreviewBtn.onclick = closePreview;"
        "};"
        "let lastFilesSignature = null;"
        "function loadFiles() {"
        "if (!lastFilesSignature) { fileList.innerHTML = '<p id=\"file-list-status\">Loading files...</p>'; }"
        "fetch('/api/files')"
        ".then(res => { if (!res.ok) throw new Error('Server responded with status ' + res.status); return res; })"
        ".then(function(response) { return response.json(); })"
        ".then(function(data) {"
        "if (data.files) { data.files.sort(function(a, b) { return b.mtime - a.mtime; }); }"
        "const newSignature = (data.files || []).map(f => `${f.name}:${f.size}:${f.mtime}`).join(',');"
        "if (newSignature === lastFilesSignature) {"
        "    return; /* Data is the same, no need to re-render */"
        "}"
        "lastFilesSignature = newSignature;"
        "document.getElementById('global-status-banner').style.display = 'none';"
        "const newFiles = new Set((data.files || []).map(f => f.name));"
        "const oldFiles = new Set(Object.keys(fileInfo));"
        "/* Remove info for files that no longer exist */"
        "for (const oldFile of oldFiles) {"
        "    if (!newFiles.has(oldFile)) {"
        "        delete fileInfo[oldFile];"
        "    }"
        "} fileList.innerHTML = ''; if (data.files && data.files.length > 0) { data.files.forEach(function(file) {"
        "const li = document.createElement('li');"
        "li.dataset.filename = file.name;"
        "const infoDiv = document.createElement('div');"
        "infoDiv.className = 'file-info';"
        "const a = document.createElement('a');"
        "const existingLi = fileList.querySelector(`li[data-filename=\"${file.name}\"]`);"
        "if (existingLi) {"
        "    /* Just update the metadata, don't recreate the whole element */"
        "    const meta = existingLi.querySelector('.file-metadata');"
        "    if (meta) meta.textContent = formatSize(file.size) + ' - ' + new Date(file.mtime * 1000).toLocaleString(undefined, {dateStyle: 'short', timeStyle: 'medium'});"
        "    return;"
        "}"
        "a.href = '/download?file=' + encodeURIComponent(file.name);"
        "a.textContent = file.name;"
        "const meta = document.createElement('div');"
        "const actionsDiv = document.createElement('div');"
        "actionsDiv.className = 'actions';"
        "const previewBtn = document.createElement('button');"
        "previewBtn.textContent = 'Preview';"
        "previewBtn.className = 'btn preview-btn';"
        "previewBtn.onclick = function() { previewFile(file.name); };"
        "meta.className = 'file-metadata';"
        "const date = new Date(file.mtime * 1000).toLocaleString(undefined, {dateStyle: 'short', timeStyle: 'medium'});"
        "meta.textContent = formatSize(file.size) + ' - ' + date;"
        "const deleteBtn = document.createElement('button');"
        "deleteBtn.textContent = 'Delete';"
        "deleteBtn.className = 'btn delete-btn';"
        "deleteBtn.onclick = function() {"
        "if (confirm('Are you sure you want to delete ' + file.name + '?')) {"
        "fetch('/api/files/' + encodeURIComponent(file.name), { method: 'DELETE' })"
        ".then(function(response) {"
        "if (response.ok) {"
        "if (currentFile === file.name) { "
        "    previewContainer.style.display = 'none'; "
        "    currentFile = ''; "
        "} "
        "li.remove();"
        "} else {"
        "alert('Failed to delete file.');"
        "}"
        "});"
        "}"
        "};"
        "infoDiv.appendChild(a);"
        "infoDiv.appendChild(meta);"
        "actionsDiv.appendChild(previewBtn);"
        "actionsDiv.appendChild(deleteBtn);"
        "li.appendChild(infoDiv);"
        "li.appendChild(actionsDiv);"
        "fileList.appendChild(li);"
        "});"
        "} else {"
        "const li = document.createElement('li');"
        "li.textContent = 'No files found on SD card.';"
        "li.style.padding = '12px 15px';"
        "fileList.appendChild(li);"
        "}"
        "})"
        ".catch(function(error) {"
        "console.error('Error fetching file list:', error);"
        "showGlobalBanner('Failed to load file list. Server may be offline.', loadFiles);"
        "fileList.innerHTML = '<p id=\"file-list-status\">Error loading file list.</p>';"
        "});"
        "}"
        "function fetchAndRender(start, timestamp, duration, direction) {"
        "    /* previewBox.removeEventListener('scroll', scrollHandler) removed as scrollHandler is gone */"
        "    window.fetchChunk(currentFile, start, maxRows, timestamp, duration, direction, function(response) {"
        "        if (!response.isCsv || !response.data.lines || response.data.lines.length === 0) {            "
        "            console.log('Received empty chunk, handling navigation boundary.');"
        "            if (direction === 'backward') {"
        "                jumpTo('top');"
        "            } else { /* forward or not specified */"
        "                jumpTo('bottom');"
        "            }            "
        "            return;"
        "        }"
        "        if (response.data.header) { renderHeader(response.data.header); }"
        "        renderRows(response.data, response.data.start_line);"
        "        setTimeout(() => {"
        "            previewBox.scrollTop = 0;" /* Scroll to top of new chunk */
        /* Infinite scroll handler removed */
        "        });"
        "    });"
        "}"
        "window.jumpTo = function(position) {"
        "    if (!fileInfo[currentFile]) return; /* Ensure file info is loaded*/"
        "    let targetTimestamp = null;"
        "    if (position === 'top') {"
        "        targetTimestamp = fileInfo[currentFile].first_ts;"
        "    } else if (position === 'bottom') {"
        "        /* For 'bottom', we want the chunk to end at fileEndTimestamp, so calculate its start*/"
        "        /* If currentScale is -1 (Max), then just go to fileStartTimestamp*/"
        "        if (currentScale === -1) {"
        "            targetTimestamp = fileInfo[currentFile].first_ts;"
        "        } else {"
        "            targetTimestamp = fileInfo[currentFile].last_ts - currentScale;"
        "            /* Ensure we don't go before the file start*/"
        "            if (targetTimestamp < fileInfo[currentFile].first_ts) {"
        "                targetTimestamp = fileInfo[currentFile].first_ts;"
        "            }"
        "        }"
        "    }"
        "    fetchAndRender(null, targetTimestamp, currentScale, 'forward');"
        "};"
        "window.navigateTime = function(offset) {"
        "    if (!lastChunkInfo.start_ts) {"
        "       console.warn('navigateTime called without lastChunkInfo.start_ts');"
        "       return;"
        "    }"
        "    if (!fileInfo[currentFile]) {"
        "        console.warn('fileInfo missing for', currentFile, 're-fetching.');"
        "        previewFile(currentFile); /* Re-fetch and re-render */"
        "        return;"
        "    }"
        "    const direction = offset < 0 ? 'backward' : 'forward';"
        "    let targetTimestamp = (lastChunkInfo.start_ts || 0) + offset;"
        "    fetchAndRender(null, targetTimestamp, currentScale, direction);"
        "};"
        "/* scrollHandler function removed to disable infinite scroll */"
        "document.querySelectorAll('#scale-buttons button').forEach(button => {"
        "    button.addEventListener('click', function() {"
        "        currentScale = parseInt(this.dataset.scale, 10);"
        "        updateScaleButtons();"
        "        if (currentFile) {"
        "            const firstRow = tbody.rows[0];"
        "            const startTs = firstRow ? parseInt(firstRow.dataset.timestamp, 10) : fileInfo[currentFile].first_ts;"
        "            fetchAndRender(null, startTs, currentScale, 'forward');"
        "        }"
        "    });"
        "});"
        "/* previewBox.addEventListener('scroll', scrollHandler) removed */"
        "loadFiles();"
        "setInterval(loadFiles, 10000);});"
        "</script>"
        "</body>"
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
static esp_err_t api_files_handler(httpd_req_t *req)
{
    DIR *dir = opendir("/sdcard/");
    if (!dir)
    {
        ESP_LOGE(TAG, "Failed to open directory: /sdcard/");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    // Start building the JSON response
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send_chunk(req, "{\"files\":[", 10);

    bool first_entry = true;
    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL)
    {
        if (entry->d_type == DT_REG)
        { // Only list regular files
            char json_entry[350];
            char full_path[ESP_VFS_PATH_MAX + CONFIG_FATFS_MAX_LFN];
            snprintf(full_path, sizeof(full_path), "/sdcard/%s", entry->d_name);
            struct stat st;
            if (stat(full_path, &st) != 0)
            {
                continue; // Skip if we can't get file stats
            }

            if (!first_entry)
            {
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
 * @brief HTTP GET handler for the `/api/fileinfo` endpoint.
 *
 * Returns the first and last timestamps from a CSV file. This is done
 * efficiently by reading only the first data line and a small chunk from
 * the end of the file.
 * @param req The HTTP request.
 * @return ESP_OK on success.
 */
static esp_err_t api_fileinfo_handler(httpd_req_t *req)
{
    char filename[64] = {0};
    char filepath[FILE_PATH_MAX];
    time_t first_ts = 0;
    time_t last_ts = 0;

    size_t query_len = httpd_req_get_url_query_len(req) + 1;
    if (query_len > 1)
    {
        char *query_buf = malloc(query_len);
        if (httpd_req_get_url_query_str(req, query_buf, query_len) == ESP_OK)
        {
            if (httpd_query_key_value(query_buf, "file", filename, sizeof(filename)) == ESP_OK)
            {
                url_decode(filename);
            }
        }
        free(query_buf);
    }

    if (strlen(filename) == 0)
    {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    snprintf(filepath, sizeof(filepath), "/sdcard/%s", filename);
    FILE *f = fopen(filepath, "r");
    if (!f)
    {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    char line_buf[256];
    // Read and discard header
    if (fgets(line_buf, sizeof(line_buf), f) == NULL)
    {
        fclose(f);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    // Read first data line for first_ts
    if (fgets(line_buf, sizeof(line_buf), f) != NULL)
    {
        first_ts = atoll(line_buf);
    }

    // Seek to near the end to find the last line
    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    if (size > 512)
        fseek(f, -512, SEEK_END);
    else
        fseek(f, 0, SEEK_SET);

    char last_line[256] = "";
    while (fgets(line_buf, sizeof(line_buf), f) != NULL)
    {
        strncpy(last_line, line_buf, sizeof(last_line) - 1);
    }
    last_ts = atoll(last_line);

    fclose(f);

    httpd_resp_set_type(req, "application/json");
    char json_resp[128];
    snprintf(json_resp, sizeof(json_resp), "{\"first_ts\":%lld,\"last_ts\":%lld}", (long long)first_ts, (long long)last_ts);
    httpd_resp_send(req, json_resp, HTTPD_RESP_USE_STRLEN);
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
    datalogger_cmd_msg_t msg = {.cmd = DATALOGGER_CMD_DELETE_FILE};
    if (xQueueSend(g_datalogger_cmd_queue, &msg, pdMS_TO_TICKS(100)) != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to send delete command to datalogger queue");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Server busy");
        return ESP_FAIL;
    }

    // 3. Poll for the result
    command_status_t status = CMD_STATUS_PENDING;
    for (int i = 0; i < 100; i++)
    { // Poll for up to 5 seconds (100 * 50ms)
        vTaskDelay(pdMS_TO_TICKS(50));
        if (xSemaphoreTake(g_sensor_buffer_mutex, pdMS_TO_TICKS(50)))
        {
            status = g_sensor_buffer.delete_file_status;
            xSemaphoreGive(g_sensor_buffer_mutex);
        }
        if (status == CMD_STATUS_SUCCESS || status == CMD_STATUS_FAIL)
        {
            break;
        }
    }

    if (status == CMD_STATUS_SUCCESS)
    {
        httpd_resp_set_status(req, "204 No Content");
        httpd_resp_send(req, NULL, 0);
        return ESP_OK;
    }
    else
    { // CMD_STATUS_FAIL or timeout (still PENDING)
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
static esp_err_t download_handler(httpd_req_t *req)
{
    char filepath[FILE_PATH_MAX];
    char filename[64];
    file_server_data_t *server_data = (file_server_data_t *)req->user_ctx;

    // Get the length of the query string
    size_t query_len = httpd_req_get_url_query_len(req) + 1;
    if (query_len <= 1)
    {
        ESP_LOGE(TAG, "No query string found in download request");
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    // Allocate buffer and get the query string

    char *query_str = malloc(query_len);
    if (httpd_req_get_url_query_str(req, query_str, query_len) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to get query string");
        free(query_str);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    // Parse the filename from the query string
    if (httpd_query_key_value(query_str, "file", filename, sizeof(filename)) != ESP_OK)
    {
        ESP_LOGE(TAG, "Filename not found in query string");
        free(query_str);
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }
    free(query_str);

    // Basic security check: prevent path traversal
    if (strstr(filename, ".."))
    {
        ESP_LOGE(TAG, "Path traversal attempt detected: %s", filename);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Bad Request");
        return ESP_FAIL;
    }

    snprintf(filepath, sizeof(filepath), "/sdcard/%s", filename);
    ESP_LOGD(TAG, "Serving file: %s", filepath);

    int fd = open(filepath, O_RDONLY, 0);
    if (fd == -1)
    {
        ESP_LOGE(TAG, "Failed to open file: %s", filepath);
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    // Get file size
    struct stat file_stat;
    if (fstat(fd, &file_stat) == -1)
    {
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
    do
    {
        read_bytes = read(fd, server_data->scratch, SCRATCH_BUFSIZE);
        if (read_bytes > 0)
        {
            if (httpd_resp_send_chunk(req, server_data->scratch, read_bytes) != ESP_OK)
            {
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
 * @brief Finds the file offset corresponding to or after a target timestamp.
 * Assumes the first column of each line is a UNIX timestamp.
 * @param f The file pointer (should be positioned after the header).
 * @param target_timestamp The UNIX timestamp to search for.
 * @return The file offset for the line, or `data_start_offset` if not found.
 * @param direction If > 0, find first line >= timestamp. If <= 0, find first line <= timestamp.
 */
static long find_offset_by_timestamp(FILE *f, time_t target_timestamp, long data_start_offset, int direction)
{
    char line_buf[256]; // Buffer for reading lines
    fseek(f, 0, SEEK_END);
    long high = ftell(f);
    long low = data_start_offset;
    long best_offset = (direction > 0) ? high : data_start_offset;

    // Perform a binary search on the file offsets to quickly find the approximate location.
    while (low <= high) {
        long mid = low + (high - low) / 2;
        fseek(f, mid, SEEK_SET);

        // If not at the start, read until the next newline to align to a line start.
        if (mid > data_start_offset && fgets(line_buf, sizeof(line_buf), f) == NULL) {
            high = mid - 1; // We're at the end, search lower half.
            continue;
        }

        long current_pos = ftell(f);
        if (fgets(line_buf, sizeof(line_buf), f) == NULL) {
            // Reached end of file, the target must be in the lower half.
            high = mid - 1;
            continue;
        }

        time_t line_ts = atoll(line_buf);

        if (line_ts < target_timestamp) {
            if (direction > 0) { // Searching forward, need to look in the upper half.
                low = mid + 1;
            } else { // Searching backward, this is a potential candidate.
                best_offset = current_pos;
                low = mid + 1; // Try to find a later line that's still <= target.
            }
        } else { // line_ts >= target_timestamp
            if (direction > 0) { // Searching forward, this is a potential candidate.
                best_offset = current_pos;
                high = mid - 1; // Try to find an earlier line that's still >= target.
            } else { // Searching backward, need to look in the lower half.
                high = mid - 1;
            }
        }
    }

    // For a backward search, we might need to step back one more line to get the
    // beginning of the context that *includes* the target timestamp.
    if (direction <= 0 && best_offset > data_start_offset) {
        fseek(f, best_offset, SEEK_SET);
        if (fgets(line_buf, sizeof(line_buf), f) != NULL && atoll(line_buf) < target_timestamp) {
            // We landed on a line before our target. To get the full context leading up to it,
            // we need to find the line *before* this one.
            long prev_line_offset = best_offset;
            while (prev_line_offset > data_start_offset) {
                fseek(f, --prev_line_offset, SEEK_SET);
                if (fgetc(f) == '\n') {
                    best_offset = ftell(f);
                    break;
                }
            }
        }
    }

    return best_offset;
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
    int requested_start_line = 0;   /* 0-indexed, after header */
    int line_count = 100;           // Default to 100 lines for chunk size
    int duration_sec = -1;          // Default to no duration limit
    time_t requested_timestamp = 0; // 0 indicates no timestamp search
    char direction_str[16] = "forward"; // Default direction

    char *query_buf = NULL;
    size_t query_len = httpd_req_get_url_query_len(req) + 1;
    if (query_len > 1)
    {
        query_buf = malloc(query_len);
        if (!query_buf)
        {
            ESP_LOGE(TAG, "Failed to allocate query buffer");
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }
        if (httpd_req_get_url_query_str(req, query_buf, query_len) == ESP_OK)
        {
            char param[64];
            if (httpd_query_key_value(query_buf, "file", param, sizeof(param)) == ESP_OK)
            {
                strncpy(filename, param, sizeof(filename) - 1);
                url_decode(filename); // Decode the URL-encoded filename
            }
            if (httpd_query_key_value(query_buf, "start", param, sizeof(param)) == ESP_OK)
            {
                requested_start_line = atoi(param);
            }
            if (httpd_query_key_value(query_buf, "count", param, sizeof(param)) == ESP_OK)
            {
                int val = atoi(param);
                if (val > 0)
                    line_count = val;
            }
            if (httpd_query_key_value(query_buf, "timestamp", param, sizeof(param)) == ESP_OK)
            {
                // ESP_LOGI(TAG, "Preview request with timestamp: %s", param);
                requested_timestamp = atoll(param);
            }
            if (httpd_query_key_value(query_buf, "duration", param, sizeof(param)) == ESP_OK)
            {
                duration_sec = atoi(param);
            }
            if (httpd_query_key_value(query_buf, "direction", param, sizeof(param)) == ESP_OK)
            {
                strncpy(direction_str, param, sizeof(direction_str) - 1);
            }
        }
    }

    if (strlen(filename) == 0)
    {
        ESP_LOGE(TAG, "Filename parameter is missing in preview request");
        if (query_buf) free(query_buf);
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    snprintf(filepath, sizeof(filepath), "/sdcard/%s", filename);
    if (query_buf) free(query_buf); // Free query buffer now that we are done with it

    FILE *f = fopen(filepath, "r");
    if (f == NULL)
    {
        ESP_LOGE(TAG, "Failed to open file for preview: %s", filepath);
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    file_server_data_t *server_data = (file_server_data_t *)req->user_ctx;
    // Use parts of the scratch buffer for local string operations to avoid stack allocation
    char *line = server_data->scratch;
    char *header_line = server_data->scratch + 256;
    memset(header_line, 0, 256);
    long header_pos = ftell(f); // Save position of file start

    // Read the first line to check for the CSV header.
    if (fgets(header_line, 256, f) == NULL)
    {
        fclose(f);
        httpd_resp_send_500(req);
        ESP_LOGE(TAG, "Failed to read header line from %s", filename);
        return ESP_FAIL;
    }
    header_line[strcspn(header_line, "\r\n")] = 0;       // Trim newline characters from fgets
    char *trimmed_header = trim_whitespace(header_line); // Trim leading/trailing spaces

    // Execute the regular expression match
    if (!regex_is_compiled)
    {
        fclose(f);
        ESP_LOGE(TAG, "CSV header regex not compiled!");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    int reti = regexec(&csv_header_regex, trimmed_header, 0, NULL, 0);

    if (reti == REG_NOMATCH)
    { // reti != 0 means no match was found
        // --- Fallback to Plain Text Preview ---
        ESP_LOGI(TAG, "File '%s' does not match generic CSV header format. Sending as plain text.", filename);
        httpd_resp_set_type(req, "text/plain");

        // Rewind to the beginning of the file to send the content from the start
        fseek(f, header_pos, SEEK_SET);

        int lines_sent = 0;
        while (fgets(line, sizeof(line), f) != NULL && lines_sent < line_count)
        { // Use line_count for raw text too
            httpd_resp_send_chunk(req, line, strlen(line));
            lines_sent++;
        }
        if (!feof(f))
        {
            httpd_resp_send_chunk(req, "\n(TRUNCATED)", 12);
        }
        httpd_resp_send_chunk(req, NULL, 0);
        fclose(f);
        return ESP_OK;
    }
    else if (reti != 0)
    {
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
    long current_offset = ftell(f);
    const long data_start_offset = current_offset;
    if (requested_timestamp != 0)
    {
        int direction = (strcmp(direction_str, "backward") == 0) ? -1 : 1;
        long found_offset = find_offset_by_timestamp(f, requested_timestamp, data_start_offset, direction);
        fseek(f, found_offset, SEEK_SET);
        actual_start_line_idx = -1; // Line index is not used when seeking by timestamp
    }
    else if (requested_start_line == -1)
    {
        // Efficiently seek to near the end of the file to read the last chunk
        fseek(f, 0, SEEK_END);
        long file_size = ftell(f);
        long seek_pos = (file_size > SCRATCH_BUFSIZE) ? (file_size - SCRATCH_BUFSIZE) : 0;
        fseek(f, seek_pos, SEEK_SET);
        if (seek_pos > 0) { // If not at the start, find the beginning of the next line
            if (fgets(line, sizeof(line), f) == NULL) { // Read and discard partial line
                 fseek(f, file_size, SEEK_SET); // Go to very end if read fails
            }
        }
        current_offset = ftell(f);
        actual_start_line_idx = -1; // Line index is not used when seeking to the end
    }
    else
    {
        // Request for specific start line (or default 0)
        fseek(f, data_start_offset, SEEK_SET);
        current_offset = data_start_offset;
        actual_start_line_idx = requested_start_line;
    }

    httpd_resp_set_type(req, "application/json");
    char json_start[512];
    snprintf(json_start, sizeof(json_start), "{\"header\":\"%s\",\"start_line\":%d,\"lines\":[", header_line, actual_start_line_idx);
    httpd_resp_send_chunk(req, json_start, strlen(json_start));

    char *buf = server_data->scratch;
    time_t first_line_ts = 0;
    time_t duration_end_ts = 0;
    char *line_start; // Declare here to be used across the loop

    bool is_first_line_in_json = true;
    int lines_sent = 0;
    size_t leftover_len = 0;

    while (lines_sent < 10000)
    {
        // Read new data into the buffer, after any leftover partial line
        size_t bytes_to_read = SCRATCH_BUFSIZE - leftover_len - 1;
        size_t bytes_read = fread(buf + leftover_len, 1, bytes_to_read, f);

        if (duration_sec > 0 && first_line_ts > 0 && duration_end_ts == 0)
        {
            duration_end_ts = first_line_ts + duration_sec;
        }

        if (bytes_read == 0 && leftover_len == 0) {
            break; // No more data to read and no leftovers
        }

        size_t total_len = leftover_len + bytes_read;
        line_start = buf; // Reset line_start to the beginning of the buffer for each new chunk
        char *buf_end = buf + total_len;

        while (line_start < buf_end) {
            char *next_newline = memchr(line_start, '\n', buf_end - line_start);
            if (!next_newline) {
                // No more complete lines in this buffer, save the rest for the next read
                leftover_len = buf_end - line_start;
                memmove(buf, line_start, leftover_len);
                break;
            }
            leftover_len = 0; // We found a line, so reset leftovers for this iteration

            size_t line_len = next_newline - line_start;
            if (line_len > 0 && line_start[line_len - 1] == '\r') {
                line_len--; // Trim CR
            }

            if (line_len > 0) {
                // Manually parse timestamp to handle potential null characters
                time_t current_line_ts = 0;
                char ts_str[21] = {0};
                char *comma = memchr(line_start, ',', line_len);
                if (comma) {
                    size_t ts_len = comma - line_start;
                    if (ts_len < sizeof(ts_str)) {
                        memcpy(ts_str, line_start, ts_len);
                        current_line_ts = atoll(ts_str);
                    }
                }

                if (is_first_line_in_json) {
                    first_line_ts = current_line_ts;
                }

                if (!is_first_line_in_json) httpd_resp_send_chunk(req, ",", 1);
                httpd_resp_send_chunk(req, "\"", 1);
                httpd_resp_send_chunk(req, line_start, line_len);
                httpd_resp_send_chunk(req, "\"", 1);
                is_first_line_in_json = false;
                lines_sent++;

                // Check duration limit after sending
                if (duration_end_ts > 0 && current_line_ts >= duration_end_ts) {
                    lines_sent = 10001; // Force exit from both loops
                    break;
                }

            }

            line_start = next_newline + 1; // Move to the start of the next line
            if (lines_sent >= 10000) break;
        }

        if (lines_sent > 10000) break;

        if (bytes_read == 0 && leftover_len > 0) {
            break; // Exit the main loop, we are done.
        }
        // Yield to other tasks to prevent watchdog timer from triggering on long operations
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    fclose(f);
    httpd_resp_send_chunk(req, "]}", 2); // Close JSON
    httpd_resp_send_chunk(req, NULL, 0); // Finalize
    return ESP_OK;
}

/**
 * @brief HTTP GET handler for the `/api/sensordata` endpoint.
 *
 * Triggers a sensor data refresh and returns the latest data from the
 * global buffer as a JSON object.
 * @param req The HTTP request.
 * @return ESP_OK on success.
 */
static esp_err_t api_sensordata_handler(httpd_req_t *req)
{
    // 1. Request a data refresh from the datalogger task
    if (g_datalogger_cmd_queue != NULL)
    {
        xQueueSend(g_datalogger_cmd_queue, &(datalogger_cmd_msg_t){.cmd = DATALOGGER_CMD_FORCE_REFRESH}, 0);
    }

    // 2. Give a moment for the refresh to potentially happen.
    // The datalogger task itself throttles requests, so this is safe.
    vTaskDelay(pdMS_TO_TICKS(50));

    // 3. Read the latest data from the shared buffer
    sensor_buffer_t data;
    if (xSemaphoreTake(g_sensor_buffer_mutex, pdMS_TO_TICKS(100)))
    {
        data = g_sensor_buffer;
        xSemaphoreGive(g_sensor_buffer_mutex);
    }
    else
    {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    // 4. Format and send the JSON response
    httpd_resp_set_type(req, "application/json");
    char json_buffer[512];
    char local_time_str[32];
    struct tm local_tm;
    convert_gmt_to_cet(data.timestamp, &local_tm);
    strftime(local_time_str, sizeof(local_time_str), "%Y-%m-%d %H:%M:%S", &local_tm);

    // Handle potential NaN values by printing "null" for JSON compatibility.
    char temp_str[16], diff_press_str[16], batt_volt_str[16];
    snprintf(temp_str, sizeof(temp_str), isnan(data.temperature_c) ? "null" : "%.2f", data.temperature_c);
    snprintf(diff_press_str, sizeof(diff_press_str), isnan(data.diff_pressure_pa) ? "null" : "%.2f", data.diff_pressure_pa);
    snprintf(batt_volt_str, sizeof(batt_volt_str), isnan(data.battery_voltage) ? "null" : "%.2f", data.battery_voltage);

    // Convert pressure from Pa (long) to kPa (float) for JSON compatibility
    float pressure_kpa = (data.pressure_pa != 0) ? (float)data.pressure_pa / 1000.0f : 0.0f;

    snprintf(json_buffer, sizeof(json_buffer),
             "{\"timestamp\":%lld,\"datetime_local\":\"%s\",\"temperature_c\":%s,\"pressure_kpa\":%.3f,\"diff_pressure_pa\":%s,\"battery_voltage\":%s,\"battery_percentage\":%d,\"battery_externally_powered\":%s}",
             (long long)data.timestamp,
             local_time_str,
             temp_str,
             pressure_kpa,
             diff_press_str,
             batt_volt_str,
             data.battery_percentage,
             data.battery_externally_powered ? "true" : "false");
    return httpd_resp_send(req, json_buffer, HTTPD_RESP_USE_STRLEN);
}

esp_err_t start_web_server(void)
{
    if (server)
    {
        ESP_LOGI(TAG, "Web server already running");
        return ESP_OK;
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.uri_match_fn = httpd_uri_match_wildcard;
    config.max_uri_handlers = 8;

    file_server_data_t *server_data = calloc(1, sizeof(file_server_data_t));
    if (!server_data)
    {
        ESP_LOGE(TAG, "Failed to allocate memory for server data");
        return ESP_ERR_NO_MEM;
    }
    server_data->scratch = malloc(SCRATCH_BUFSIZE);
    if (!server_data->scratch)
    {
        ESP_LOGE(TAG, "Failed to allocate scratch buffer");
        free(server_data);
        return ESP_ERR_NO_MEM;
    }

    // Compile the regex for CSV header detection at startup
    const char *csv_pattern = "^[a-zA-Z0-9_\\.]+[a-zA-Z0-9_\\.\\s]*(,[a-zA-Z0-9_\\.\\s]+)+";
    int reg_err_code = regcomp(&csv_header_regex, csv_pattern, REG_EXTENDED | REG_NOSUB);
    if (reg_err_code != 0)
    {
        char msgbuf[100];
        regerror(reg_err_code, &csv_header_regex, msgbuf, sizeof(msgbuf));
        ESP_LOGE(TAG, "Could not compile regex for CSV header: %s", msgbuf);
        free(server_data->scratch);
        free(server_data);
        return ESP_FAIL;
    }
    regex_is_compiled = true;

    ESP_LOGI(TAG, "Starting HTTP Server");
    if (httpd_start(&server, &config) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to start httpd");
        free(server_data->scratch);
        free(server_data);
        server = NULL;
        return ESP_FAIL;
    }

    httpd_uri_t file_list_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = index_html_handler,
        .user_ctx = server_data};
    httpd_register_uri_handler(server, &file_list_uri);

    httpd_uri_t api_files_uri = {
        .uri = "/api/files",
        .method = HTTP_GET,
        .handler = api_files_handler,
        .user_ctx = server_data};
    httpd_register_uri_handler(server, &api_files_uri);

    httpd_uri_t api_delete_uri = {
        .uri = "/api/files/*",
        .method = HTTP_DELETE,
        .handler = api_file_delete_handler,
        .user_ctx = server_data};
    httpd_register_uri_handler(server, &api_delete_uri);

    httpd_uri_t api_sensordata_uri = {
        .uri = "/api/sensordata",
        .method = HTTP_GET,
        .handler = api_sensordata_handler,
        .user_ctx = server_data};
    httpd_register_uri_handler(server, &api_sensordata_uri);

    httpd_uri_t api_preview_uri = {
        .uri = "/api/preview",
        .method = HTTP_GET,
        .handler = api_preview_handler,
        .user_ctx = server_data};
    httpd_register_uri_handler(server, &api_preview_uri);

    httpd_uri_t api_fileinfo_uri = {
        .uri = "/api/fileinfo",
        .method = HTTP_GET,
        .handler = api_fileinfo_handler,
        .user_ctx = server_data};
    httpd_register_uri_handler(server, &api_fileinfo_uri);

    httpd_uri_t download_uri = {
        .uri = "/download",
        .method = HTTP_GET,
        .handler = download_handler,
        .user_ctx = server_data};
    httpd_register_uri_handler(server, &download_uri);

    return ESP_OK;
}

void stop_web_server(void)
{
    if (server)
    {
        ESP_LOGI(TAG, "Stopping HTTP Server");
        file_server_data_t *server_data = (file_server_data_t *)httpd_get_global_user_ctx(server);
        if (server_data)
        {
            free(server_data->scratch);
            free(server_data);
        }
        if (regex_is_compiled)
        {
            regfree(&csv_header_regex);
            regex_is_compiled = false;
        }
        httpd_stop(server);
        server = NULL;
    }
}