#include "ui_render.h"
#include "i2c-oled.h"
#include "rotaryencoder.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <string.h>
#include "esp_timer.h"
#include <ui_menudef.h>
#include "esp_log.h"
#include "time_utils.h"
#include "../../src/datalogger_task.h"
#include "../buffers.h"
#include "../../src/config_params.h"
#include <math.h>
#include "qrcode.h"
#include "wifi_manager.h"
#include "web_server.h"

#include "esp_netif.h"
// --- QR Code Caching ---
// A dedicated buffer to store the rendered QR code image to avoid re-generating it.
#define OLED_WIDTH 128
#define OLED_HEIGHT 64
static uint8_t s_qr_code_buffer[OLED_WIDTH * OLED_HEIGHT / 8];
static bool s_qr_code_cached = false;

// --- Web Server QR Code Caching ---
static uint8_t s_web_qr_code_buffer[OLED_WIDTH * OLED_HEIGHT / 8];
static bool s_web_qr_code_cached = false;
static int s_about_page_view = 0; // 0 for QR code, 1 for text link, 2 for build info
static int s_web_page_view = 0;   // 0 for QR code, 1 for text URL

static const char *TAG = "ui_render";

#define UI_QUEUE_LEN 16
#define UI_MENU_STACK_DEPTH 8
#define UI_MENU_VISIBLE_ROWS 6

typedef struct
{
    ui_event_t event;
    float values[8];
    int value_count;
} ui_event_msg_t;

typedef struct
{
    uint8_t page;
    uint8_t item;
} menu_stack_entry_t;

static QueueHandle_t ui_event_queue = NULL;
static menu_stack_entry_t menu_stack[UI_MENU_STACK_DEPTH];
static int menu_stack_pos = 0;

static int s_sensor_page_num = 0; // 0 for page 1, 1 for page 2
static const int SENSOR_PAGE_COUNT = 2;
typedef enum
{
    POST_ACTION_NONE,
    POST_ACTION_GO_BACK,
} post_cmd_action_t;

static i2c_port_t s_oled_i2c_num;
static bool s_oled_initialized = false;

// UI mode: false = page (default to sensor page), true = menu
static bool s_menu_mode = false;
static bool s_sleeping_mode = false;
static bool s_waking_up = false; // Flag to indicate we are waking up and should ignore input
static const ui_page_t *s_current_page = &sensor_page;

// Command feedback UI state
static bool s_cmd_pending_mode = false;
static uint64_t s_cmd_start_time_ms = 0;
static uint32_t s_cmd_timeout_ms = 5000; // Default timeout
static post_cmd_action_t s_post_cmd_action = POST_ACTION_NONE;
static uint64_t s_cmd_feedback_display_start_ms = 0; // For non-blocking feedback

static void ui_config_page_prepare_data(void);
/* --- For Config Page --- */

// Structure to hold a single config item for display.
typedef struct
{
    const char *full_key;
    char value[21]; // Max 20 chars for OLED line + null terminator
} config_item_t;

#define MAX_CONFIG_ITEMS 9 // Correctly define the number of items
static config_item_t s_config_items[MAX_CONFIG_ITEMS];

// Define the actual configuration keys here, in one place.
static const char *s_config_keys[MAX_CONFIG_ITEMS] = {
    "v_div_ratio",
    "inactive_ms",
    "sleep_ms",
    "hf_sleep_ms",
    "log_int_ms",
    "hf_log_int_ms",
    "d6fph_model",
    "wifi_ssid",
    "wifi_password"};
static int s_num_config_items = 0;

static int s_config_current_page_index = 0;
const int ITEMS_PER_PAGE = 3; // Each item takes 2 lines (key + value), so 3 items fit on 6-7 lines

// Data for rendering
static float last_values[8] = {0};
static int last_value_count = 0;

// Menu navigation state
static int8_t current_page = 0;
static int8_t current_item = 0;

// --- Local copy of sensor data for rendering ---
static sensor_buffer_t s_local_sensor_buffer;

// --- Initialization and event send ---
void uiRender_init(i2c_master_bus_handle_t bus_handle, uint8_t oled_i2c_addr)
{
    i2c_oled_bus_init(bus_handle, oled_i2c_addr);
    s_oled_initialized = true;
}

void uiRender_send_event(int event, float *values, int value_count)
{
    if (!ui_event_queue)
        return;
    ui_event_msg_t msg = {.event = event, .value_count = value_count};
    if (values && value_count > 0)
    {
        memcpy(msg.values, values, sizeof(float) * value_count);
    }
    xQueueSend(ui_event_queue, &msg, 0);
}

// --- Helper to write a flicker-free, padded line ---
static void write_padded_line(uint8_t row, const char *text)
{
    if (!s_oled_initialized)
        return;
    char padded_buffer[21]; // 20 chars + null terminator
    snprintf(padded_buffer, sizeof(padded_buffer), "%-20s", text ? text : "");
    i2c_oled_write_text(s_oled_i2c_num, row, 0, padded_buffer);
}

// --- Helper to write a flicker-free, inverted line ---
static void write_inverted_line(uint8_t row, const char *text)
{
    if (!s_oled_initialized)
        return;
    char padded_buffer[21]; // 20 chars + null terminator
    snprintf(padded_buffer, sizeof(padded_buffer), "%-20s", text ? text : "");
    // This new function handles inversion at the pixel level, preventing flicker.
    i2c_oled_write_inverted_text(s_oled_i2c_num, row, 0, padded_buffer);
}

// --- Menu rendering callback ---
void render_menu_callback(void)
{
    if (!s_oled_initialized)
        return;
    // Clear the screen buffer before drawing the menu to prevent artifacts.
    i2c_oled_clear(s_oled_i2c_num);

    static char last_lines[8][21] = {{0}};
    char new_lines[8][21] = {{0}};

    const ui_menu_page_t *page = &ui_menu_tree[current_page];
    int total = page->item_count;
    int win = UI_MENU_VISIBLE_ROWS;
    int start = 0;

    // Clamp current_item to valid range
    if (current_item >= total)
        current_item = total - 1;

    // Calculate window start so current_item is centered if possible
    if (total > win)
    {
        start = current_item - win / 2;
        if (start < 0)
            start = 0;
        if (start > total - win)
            start = total - 1;
    }

    snprintf(new_lines[0], sizeof(new_lines[0]), "%s", page->title);

    // new_lines[1] is intentionally left blank
    for (int i = 0; i < win && (start + i) < total; ++i)
    {
        int idx = start + i;
        const ui_menu_item_t *item = &page->items[idx];
        char *buffer = new_lines[i + 2];
        const size_t buffer_size = sizeof(new_lines[i + 2]);
        char prefix = (idx == current_item) ? '>' : ' ';

        if (item->has_submenu)
        {
            // e.g., "> Label >" needs 4 chars + label + null
            snprintf(buffer, buffer_size, "%c %.*s >", prefix, (int)(buffer_size - 5), item->label);
        }
        else
        {
            // e.g., "> Label" needs 2 chars + label + null
            snprintf(buffer, buffer_size, "%c %.*s", prefix, (int)(buffer_size - 3), item->label);
        }
    }

    write_inverted_line(0, new_lines[0]);
    for (uint8_t row = 1; row < 8; ++row)
    {
        write_padded_line(row, new_lines[row]);
        strncpy(last_lines[row], new_lines[row], sizeof(last_lines[row]));
    }
}

/**
 * @brief Formats uptime in seconds to a D:HH:MM:SS string.
 */
static void format_uptime(uint64_t total_seconds, char *buf, size_t len)
{
    uint32_t days = total_seconds / (24 * 3600);
    total_seconds %= (24 * 3600);
    uint32_t hours = total_seconds / 3600;
    total_seconds %= 3600;
    snprintf(buf, len, "%lu:%02lu:%02llu:%02llu", days, hours, (unsigned long long)total_seconds / 60, (unsigned long long)total_seconds % 60);
}

// Forward declaration for the QR code display callback
static void oled_display_qr_code(esp_qrcode_handle_t qrcode);

// --- About rendering callback ---
void render_about_callback(void)
{
    if (!s_oled_initialized)
        return;

    i2c_oled_clear(s_oled_i2c_num);

    if (s_about_page_view == 0) { // QR Code View
        // If the QR code is not cached, generate it once.
        if (!s_qr_code_cached) {
            esp_qrcode_config_t cfg = ESP_QRCODE_CONFIG_DEFAULT();
            cfg.display_func = oled_display_qr_code; // This will draw to the main buffer
            esp_qrcode_generate(&cfg, PROJECT_GITHUB_URL);
            i2c_oled_get_buffer(s_qr_code_buffer, sizeof(s_qr_code_buffer)); // Copy to cache
            s_qr_code_cached = true;
        }
        // Copy the cached QR code to the main screen buffer for display.
        i2c_oled_load_buffer(s_qr_code_buffer, sizeof(s_qr_code_buffer));
    } else if (s_about_page_view == 1) { // Text URL View
        write_inverted_line(0, "About");
        write_padded_line(2, "Project URL:");
        write_padded_line(4, "github.com/maunope/");
        write_padded_line(5, "DiffPressSensor");
    } else { // Build Info View
        write_inverted_line(0, "About");
        write_padded_line(2, "Build Info");
        write_padded_line(4, "Date: " __DATE__);
        write_padded_line(5, "Time: " __TIME__);
    }
}

// --- Web Server Page Rendering ---
void render_web_server_callback(void)
{
    if (!s_oled_initialized)
        return;

    int status = WEB_SERVER_STOPPED;
    char url[64] = {0};

    if (xSemaphoreTake(g_sensor_buffer_mutex, pdMS_TO_TICKS(50)))
    {
        status = g_sensor_buffer.web_server_status;
        strncpy(url, g_sensor_buffer.web_server_url, sizeof(url) - 1);
        xSemaphoreGive(g_sensor_buffer_mutex);
    }

    i2c_oled_clear(s_oled_i2c_num);

    switch (status)
    {
    case WEB_SERVER_STARTING:
        write_inverted_line(0, "Web Server");
        write_padded_line(2, "Starting...");
        write_padded_line(3, "Please wait.");
        break;

    case WEB_SERVER_RUNNING:
        if (s_web_page_view == 0)
        { // QR Code View
            if (!s_web_qr_code_cached)
            {
                esp_qrcode_config_t cfg = ESP_QRCODE_CONFIG_DEFAULT();
                cfg.display_func = oled_display_qr_code;
                esp_qrcode_generate(&cfg, url);
                i2c_oled_get_buffer(s_web_qr_code_buffer, sizeof(s_web_qr_code_buffer));
                s_web_qr_code_cached = true;
            }
            i2c_oled_load_buffer(s_web_qr_code_buffer, sizeof(s_web_qr_code_buffer));
        }
        else
        { // Text URL View
            write_inverted_line(0, "Web Server");
            write_padded_line(2, "Connect to:");
            write_padded_line(3, url);
            write_padded_line(6, "Press btn to exit");
        }
        break;

    case WEB_SERVER_FAILED:
        write_inverted_line(0, "Web Server");
        write_padded_line(2, "Failed to start.");
        write_padded_line(3, "Check WiFi config.");
        write_padded_line(6, "Press btn to exit");
        break;

    case WEB_SERVER_USB_CONNECTED:
        write_inverted_line(0, "Web Server");
        write_padded_line(2, "Blocked by USB");
        write_padded_line(3, "connection.");
        write_padded_line(6, "Press btn to exit");
        break;

    default: // WEB_SERVER_STOPPED or unknown
        write_inverted_line(0, "Web Server");
        write_padded_line(2, "Stopped.");
        break;
    }
}

void page_web_server_on_btn(void)
{
    // This function is called when the button is pressed on the QR code/URL screen.
    // It should stop the server and return to the main menu.
    app_command_t cmd = APP_CMD_STOP_WEB_SERVER;
    xQueueSend(g_app_cmd_queue, &cmd, 0);
    s_web_qr_code_cached = false; // Invalidate cache for next time
    
    // Go back to the main menu.
    // We don't use menu_cancel_on_btn() here because we might not have come from a menu.
    // This ensures we always land on the main menu screen.
    current_page = 0;
    current_item = 0;
    s_menu_mode = true;     // Switch to menu mode
    s_current_page = NULL;  // Let the main loop render the menu
}

/**
 * @brief Callback function to render a QR code to the OLED display.
 *
 * This function is passed to `esp_qrcode_generate`. It iterates through the
 * generated QR code modules and draws them onto the OLED screen buffer.
 *
 * @param qrcode Handle to the generated QR code data.
 */
static void oled_display_qr_code(esp_qrcode_handle_t qrcode)
{
    if (qrcode == NULL)
    {
        return;
    }

    int qr_size = esp_qrcode_get_size(qrcode);
    int module_size = 2; // Each QR module will be 2x2 pixels
    int image_size = qr_size * module_size;
    int x_offset = (128 - image_size) / 2;
    int y_offset = (64 - image_size) / 2;

    for (int y = 0; y < qr_size; y++)
    {
        for (int x = 0; x < qr_size; x++)
        {
            if (esp_qrcode_get_module(qrcode, x, y))
            {
                i2c_oled_fill_rect(s_oled_i2c_num, x_offset + x * module_size, y_offset + y * module_size, module_size, module_size, true);
            }
        }
    }
}

// --- Sleeping screen rendering callback ---
void render_sleeping_screen(void)
{
    if (!s_oled_initialized)
        return;
    // Clear screen once, then do nothing to save power.
    i2c_oled_clear(s_oled_i2c_num);
    write_inverted_line(0, "Sleep mode");
    i2c_oled_update_screen(s_oled_i2c_num);
    i2c_oled_write_text(s_oled_i2c_num, 4, 5, "Zzzzz....");
}

// --- Sensor rendering callback ---
void render_sensor_callback(void)
{
    if (!s_oled_initialized)
        return;
    char line_buf[21];
    char uptime_str[21];

    i2c_oled_clear(s_oled_i2c_num);        // Clear screen buffer before drawing
    write_inverted_line(0, "Sensor Data"); // Set a consistent title

    if (s_sensor_page_num == 0)
    {
        // Page 1: Main sensor data
        // Current Time
        struct tm local_tm;
        convert_gmt_to_cet(s_local_sensor_buffer.timestamp, &local_tm);
        strftime(line_buf, sizeof(line_buf), "%Y-%m-%d %H:%M:%S", &local_tm);
        write_padded_line(2, s_local_sensor_buffer.timestamp > 0 ? line_buf : "Time not set");

        // Temperature
        snprintf(line_buf, sizeof(line_buf), "T: %.2f C", s_local_sensor_buffer.temperature_c);
        write_padded_line(4, isnan(s_local_sensor_buffer.temperature_c) ? "T: N/A" : line_buf);

        // Pressure
        snprintf(line_buf, sizeof(line_buf), "P: %ld Pa", s_local_sensor_buffer.pressure_pa);
        write_padded_line(5, s_local_sensor_buffer.pressure_pa == 0 ? "P: N/A" : line_buf);

        // Differential Pressure
        snprintf(line_buf, sizeof(line_buf), "DP: %.2f Pa", s_local_sensor_buffer.diff_pressure_pa);
        write_padded_line(6, isnan(s_local_sensor_buffer.diff_pressure_pa) ? "DP: N/A" : line_buf);
    }
    else
    { // s_sensor_page_num == 1
        // Page 2: System Info (Uptime, Last Write)
        // Uptime
        format_uptime(s_local_sensor_buffer.uptime_seconds, uptime_str, sizeof(uptime_str));
        snprintf(line_buf, sizeof(line_buf), "Uptime: %.12s", uptime_str);
        write_padded_line(2, line_buf);

        // Last successful write
        if (s_local_sensor_buffer.last_successful_write_ts > 0)
        {
            struct tm local_tm;
            convert_gmt_to_cet(s_local_sensor_buffer.last_successful_write_ts, &local_tm);
            char time_buf[10];
            strftime(time_buf, sizeof(time_buf), "%H:%M:%S", &local_tm);
            snprintf(line_buf, sizeof(line_buf), "Last wr.: %s", time_buf);
        }
        else
        {
            snprintf(line_buf, sizeof(line_buf), "Last wr.: None");
        }
        write_padded_line(3, line_buf);

        if (isnan(s_local_sensor_buffer.battery_voltage))
        {
            snprintf(line_buf, sizeof(line_buf), "Batt: N/A");
        }
        else
        {
            snprintf(line_buf, sizeof(line_buf), "Batt: %.2fV %d%% %s", s_local_sensor_buffer.battery_voltage, s_local_sensor_buffer.battery_percentage, s_local_sensor_buffer.battery_externally_powered == 1 ? "(C)" : "");
        }
        write_padded_line(4, line_buf);
    }

}

void page_fs_stats_render_callback(void)
{
    char new_lines[8][21] = {{0}}; // Initialize all lines to empty
    snprintf(new_lines[0], sizeof(new_lines[0]), "File System Stats");

    int file_count = -2; // Default to loading state
    int free_space_mb = -2;
    file_count = s_local_sensor_buffer.sd_card_file_count;
    free_space_mb = s_local_sensor_buffer.sd_card_free_bytes;
    time_t last_write_ts = s_local_sensor_buffer.last_successful_write_ts;

    // Line 1 is blank

    if (file_count == -2)
    {
        snprintf(new_lines[2], sizeof(new_lines[2]), "Files: Loading...");
    }
    else
    {
        if (file_count >= 0)
        {
            snprintf(new_lines[2], sizeof(new_lines[2]), "Files: %d", file_count);
        }
        else
        {
            snprintf(new_lines[2], sizeof(new_lines[2]), "Files: Error");
        }
    }

    if (free_space_mb == -2)
    {
        snprintf(new_lines[3], sizeof(new_lines[3]), "Free: Loading...");
    }
    else if (free_space_mb >= 0)
    {
        snprintf(new_lines[3], sizeof(new_lines[3]), "Free: %d MB", free_space_mb);
    }
    else
    {
        snprintf(new_lines[3], sizeof(new_lines[3]), "Free: Error");
    }

    // Display last successful write timestamp
    snprintf(new_lines[5], sizeof(new_lines[5]), "Last OK write:");
    if (last_write_ts > 0)
    {
        struct tm local_tm;
        convert_gmt_to_cet(last_write_ts, &local_tm);
        // Format as YYYY-MM-DD HH:MM
        strftime(new_lines[6], sizeof(new_lines[6]), "%Y-%m-%d %H:%M", &local_tm);
    }
    else
    {
        snprintf(new_lines[6], sizeof(new_lines[6]), "None");
    }

    write_inverted_line(0, new_lines[0]);
    for (uint8_t row = 1; row < 8; ++row)
    {
        write_padded_line(row, new_lines[row]);
    }
}
// --- Menu event handlers ---
void menu_on_cw(void)
{
    const ui_menu_page_t *page = &ui_menu_tree[current_page];
    if (current_item + 1 < page->item_count)
    {
        current_item++;
    }
    if (current_item >= page->item_count)
        current_item = page->item_count - 1;
}

void menu_on_ccw(void)
{
    if (current_item > 0)
    {
        current_item--;
    }
}

void menu_on_btn(void)
{
    const ui_menu_item_t *item = &ui_menu_tree[current_page].items[current_item];
    if (item->has_submenu)
    {
        if (menu_stack_pos < UI_MENU_STACK_DEPTH)
        {
            menu_stack[menu_stack_pos].page = current_page;
            menu_stack[menu_stack_pos].item = current_item;
            menu_stack_pos++;
        }
        current_page = item->submenu_page_index;
        current_item = 0;
        s_menu_mode = true;
        s_current_page = NULL;
        // Clamp after page change
        const ui_menu_page_t *page = &ui_menu_tree[current_page];
        if (current_item >= page->item_count)
            current_item = page->item_count - 1;
    }
    else if (item->on_btn)
    {
        item->on_btn();
    }
}

void menu_cancel_on_btn(void)
{
    if (menu_stack_pos > 0)
    {
        menu_stack_pos--;
        current_page = menu_stack[menu_stack_pos].page;
        current_item = menu_stack[menu_stack_pos].item;
        // Clamp after page change
        const ui_menu_page_t *page = &ui_menu_tree[current_page];
        if (current_item >= page->item_count)
            current_item = page->item_count - 1;
    }
    s_menu_mode = true;
    s_current_page = NULL;
}

void render_cmd_feedback_screen(void)
{
    static command_status_t last_rendered_status = CMD_STATUS_IDLE;
    uint64_t now = esp_timer_get_time() / 1000;

    command_status_t current_status = CMD_STATUS_IDLE;
    if (xSemaphoreTake(g_command_status_mutex, pdMS_TO_TICKS(50)) == pdTRUE)
    {
        current_status = g_command_status;
        xSemaphoreGive(g_command_status_mutex);
    }

    // If the status hasn't changed, don't redraw anything.
    if (current_status == last_rendered_status && current_status == CMD_STATUS_PENDING) {
        return;
    }

    // --- Polling and State Transition Logic ---
    if (current_status == CMD_STATUS_PENDING)
    {
        if (now - s_cmd_start_time_ms > s_cmd_timeout_ms)
        { // Use parametric timeout
            write_padded_line(3, "Timeout");
            vTaskDelay(pdMS_TO_TICKS(500));
            s_cmd_pending_mode = false; // Exit loading mode
            if (xSemaphoreTake(g_command_status_mutex, portMAX_DELAY))
            {
                g_command_status = CMD_STATUS_IDLE;
                xSemaphoreGive(g_command_status_mutex);
            }
            s_post_cmd_action = POST_ACTION_NONE; // Clear action on timeout
        }
        else
        {
            write_padded_line(3, "Loading...");
            write_inverted_line(0, " "); // Inverted title bar
        }
    }
    else if (current_status == CMD_STATUS_SUCCESS)
    {
        write_padded_line(3, "Success");
        if (s_cmd_feedback_display_start_ms == 0)
        {
            s_cmd_feedback_display_start_ms = now;
        }
        else if (now - s_cmd_feedback_display_start_ms > 300)
        {                               // Show for 300ms
            if (s_post_cmd_action == POST_ACTION_GO_BACK)
            {
                menu_cancel_on_btn();
            }
            s_post_cmd_action = POST_ACTION_NONE; // Clear action
            s_cmd_pending_mode = false; // Exit loading mode
            if (xSemaphoreTake(g_command_status_mutex, portMAX_DELAY))
            {
                g_command_status = CMD_STATUS_IDLE;
                xSemaphoreGive(g_command_status_mutex);
            }
        }
    }
    else if (current_status == CMD_STATUS_FAIL)
    {
        write_padded_line(3, "Failed");
        write_inverted_line(0, " "); // Inverted title bar
        if (s_cmd_feedback_display_start_ms == 0)
        {
            s_cmd_feedback_display_start_ms = now;
        }
        else if (now - s_cmd_feedback_display_start_ms > 500)
        {                               // Show for 500ms
            if (s_post_cmd_action == POST_ACTION_GO_BACK)
            {
                menu_cancel_on_btn();
            }
            s_post_cmd_action = POST_ACTION_NONE; // Clear action
            s_cmd_pending_mode = false; // Exit loading mode
            if (xSemaphoreTake(g_command_status_mutex, portMAX_DELAY))
            {
                g_command_status = CMD_STATUS_IDLE;
                xSemaphoreGive(g_command_status_mutex);
            }
        }
    }
    else
    {
        // Should not happen, but as a fallback
        s_cmd_pending_mode = false;
        s_post_cmd_action = POST_ACTION_NONE;
    }

    // Clear other lines
    for (int i = 1; i < 8; i++)
    {
        if (i != 3)
        {                             // Don't clear the message line
            write_padded_line(i, ""); // Clear other non-title lines
        }
    }

    last_rendered_status = current_status;
}

/**
 * @brief Draws all status icons (HF Mode, SD Error, Battery) in the top-right corner.
 *
 * This function handles the positioning and rendering of all status icons,
 * ensuring they are correctly aligned and do not overlap.
 */
/*
 * @brief Draws all status icons (HF Mode, SD Error, Battery) in the top-right corner.
 *
 * This function handles the positioning and rendering of all status icons,
 * ensuring they are correctly aligned and do not overlap.
 */
static void draw_status_icons(void)
{
    if (!s_oled_initialized)
        return;

    // --- Get current state from the local buffer ---
    int percentage = s_local_sensor_buffer.battery_percentage;
    bool is_charging = s_local_sensor_buffer.battery_externally_powered;
    bool sd_write_failed = (s_local_sensor_buffer.writeStatus == WRITE_STATUS_FAIL);
    bool hf_mode_enabled = s_local_sensor_buffer.high_freq_mode_enabled;

    // The title bar is inverted on most screens. We render the icon with a non-inverted
    // color (white on black) only for specific full-screen pages that have a black background at the top.
    bool is_fullscreen_no_bar = (s_current_page == &about_page && s_about_page_view == 0) ||
                                (s_current_page == &web_server_page && s_local_sensor_buffer.web_server_status == WEB_SERVER_RUNNING && s_web_page_view == 0);

    bool is_inverted = !is_fullscreen_no_bar;

    // --- Clear the entire status icon area ---
    // This prevents flickering and artifacts from previous frames.
    int clear_start_x = 90; // Start clearing from the left of where any icon could be.
    for (int y = 0; y < 8; ++y)
    {
        for (int x = clear_start_x; x < 128; ++x)
        {
            i2c_oled_draw_pixel(s_oled_i2c_num, x, y, is_inverted);
        }
    }

    // --- Draw Battery Icon (always visible) ---
    {
        const int icon_x = 114;
        const int icon_y = 1;
        const int body_width = 12;
        const int body_height = 6;

        // Draw battery outline and terminal
        i2c_oled_draw_rect(s_oled_i2c_num, icon_x, icon_y, body_width, body_height, !is_inverted);
        i2c_oled_fill_rect(s_oled_i2c_num, icon_x + body_width, icon_y + 2, 1, 2, !is_inverted);

        // Draw fill level
        int fill_width = (body_width - 2) * percentage / 100;
        if (fill_width > 0)
        {
            i2c_oled_fill_rect(s_oled_i2c_num, icon_x + 1, icon_y + 1, fill_width, body_height - 2, !is_inverted);
        }

        // If charging, draw a small lightning bolt symbol inside
        if (is_charging)
        {
            int bolt_x = icon_x + 4;
            int bolt_y = icon_y + 1;
            i2c_oled_draw_pixel(s_oled_i2c_num, bolt_x + 1, bolt_y, is_inverted);
            i2c_oled_draw_pixel(s_oled_i2c_num, bolt_x + 2, bolt_y, is_inverted);
            i2c_oled_draw_pixel(s_oled_i2c_num, bolt_x, bolt_y + 1, is_inverted);
            i2c_oled_draw_pixel(s_oled_i2c_num, bolt_x + 1, bolt_y + 1, is_inverted);
            i2c_oled_draw_pixel(s_oled_i2c_num, bolt_x + 2, bolt_y + 2, is_inverted);
            i2c_oled_draw_pixel(s_oled_i2c_num, bolt_x + 1, bolt_y + 3, is_inverted);
        }
    }

    // --- Draw Conditional Icons (SD Error and HF Mode) ---
    int current_icon_x = 114; // Start from the left edge of the battery icon

    // If SD card write failed, draw "!!"
    if (sd_write_failed)
    {
        current_icon_x -= 8; // Move left to make space
        int warning_y = 1;
        // Draw "!!" using pixel-based rectangles
        i2c_oled_fill_rect(s_oled_i2c_num, current_icon_x, warning_y, 1, 4, !is_inverted);
        i2c_oled_draw_pixel(s_oled_i2c_num, current_icon_x, warning_y + 5, !is_inverted);
        i2c_oled_fill_rect(s_oled_i2c_num, current_icon_x + 2, warning_y, 1, 4, !is_inverted);
        i2c_oled_draw_pixel(s_oled_i2c_num, current_icon_x + 2, warning_y + 5, !is_inverted);
    }

    // If HF mode is enabled, draw ">>"
    if (hf_mode_enabled)
    {
        current_icon_x -= 8; // Move left again
        int hf_icon_y = 1;
        // Draw ">>" using pixels
        i2c_oled_draw_pixel(s_oled_i2c_num, current_icon_x + 1, hf_icon_y, !is_inverted);
        i2c_oled_draw_pixel(s_oled_i2c_num, current_icon_x, hf_icon_y + 1, !is_inverted);
        i2c_oled_draw_pixel(s_oled_i2c_num, current_icon_x + 1, hf_icon_y + 2, !is_inverted);
        i2c_oled_draw_pixel(s_oled_i2c_num, current_icon_x + 1, hf_icon_y + 3, !is_inverted);
        i2c_oled_draw_pixel(s_oled_i2c_num, current_icon_x, hf_icon_y + 4, !is_inverted);
        i2c_oled_draw_pixel(s_oled_i2c_num, current_icon_x + 1, hf_icon_y + 5, !is_inverted);
        // Second '>'
        i2c_oled_draw_pixel(s_oled_i2c_num, current_icon_x + 4, hf_icon_y, !is_inverted);
        i2c_oled_draw_pixel(s_oled_i2c_num, current_icon_x + 3, hf_icon_y + 1, !is_inverted);
        i2c_oled_draw_pixel(s_oled_i2c_num, current_icon_x + 4, hf_icon_y + 2, !is_inverted);
        i2c_oled_draw_pixel(s_oled_i2c_num, current_icon_x + 4, hf_icon_y + 3, !is_inverted);
        i2c_oled_draw_pixel(s_oled_i2c_num, current_icon_x + 3, hf_icon_y + 4, !is_inverted);
        i2c_oled_draw_pixel(s_oled_i2c_num, current_icon_x + 4, hf_icon_y + 5, !is_inverted);
    }
}

/**
 * @brief Enters the command feedback UI mode.
 * 
 * This function switches the UI to a "Loading..." screen and sets up a timeout.
 * It's used for long-running, asynchronous operations like formatting the SD card.
 * @param timeout_ms The timeout duration in milliseconds.
 * @param post_action The action to perform after the command completes (e.g., go back).
 */
static void enter_cmd_pending_mode(uint32_t timeout_ms, post_cmd_action_t post_action)
{
    s_cmd_pending_mode = true;
    s_cmd_start_time_ms = esp_timer_get_time() / 1000;
    s_cmd_timeout_ms = timeout_ms;
    s_post_cmd_action = post_action;
    s_cmd_feedback_display_start_ms = 0; // Reset feedback timer
    if (xSemaphoreTake(g_command_status_mutex, portMAX_DELAY))
    {
        g_command_status = CMD_STATUS_PENDING;
        xSemaphoreGive(g_command_status_mutex);
    }
}



/**
 * @brief The main task for the user interface.
 * 
 * This task runs an infinite loop to process UI events from a queue, update the display state, and call the appropriate rendering functions.
 */
// --- UI task ---
void uiRender_task(void *pvParameters)
{
    ui_event_queue = xQueueCreate(UI_QUEUE_LEN, sizeof(ui_event_msg_t));
   

    uint64_t last_sensor_refresh_ms = 0;
    const uint32_t sensor_refresh_interval_ms = 5000; // 5 seconds

    while (1)
    {
        ui_event_msg_t msg;
        if (xQueueReceive(ui_event_queue, &msg, pdMS_TO_TICKS(100)))
        {
            // The rotary encoder task now sends APP_CMD_ACTIVITY_DETECTED directly.
            if (msg.event == UI_EVENT_PREPARE_SLEEP)
            {
                s_waking_up = false; // Clear waking up flag if we are going to sleep
                s_sleeping_mode = true;
                render_sleeping_screen(); // Draw the sleeping message immediately
            }
            else if (msg.event == UI_EVENT_WAKE_UP)
            {
                s_sleeping_mode = false;
                i2c_oled_clear(s_oled_i2c_num);
                s_waking_up = true;             // Set the waking up flag
                i2c_oled_clear(s_oled_i2c_num); // Clear the screen on wake up
            }
            else if (s_waking_up)
            {
                // While in the "waking up" state, ignore all other inputs.
                // This effectively debounces the wake-up button press.
            }
            else if (s_cmd_pending_mode)
            {
                // Ignore all input while a command is pending
            }
            else if (s_menu_mode)
            {
                if (msg.event == UI_EVENT_CW)
                    menu_on_cw();
                else if (msg.event == UI_EVENT_CCW)
                    menu_on_ccw();
                else if (msg.event == UI_EVENT_BTN_LONG)
                {
                    // Long press returns to sensor data page (home)
                    // ESP_LOGI(TAG, "UI_EVENT_BTN_LONG in menu mode, returning to sensor page. 1");
                    s_menu_mode = false;
                    s_current_page = &sensor_page;
                }
                else if (msg.event == UI_EVENT_BTN)
                    menu_on_btn();
            }
            else if (s_current_page)
            {
                if (msg.event == UI_EVENT_CW && s_current_page->on_cw)
                    s_current_page->on_cw();
                else if (msg.event == UI_EVENT_BTN_LONG)
                {
                    // Long press returns to sensor data page (home)
                    // ESP_LOGI(TAG, "UI_EVENT_BTN_LONG in page mode, returning to sensor page. 2");
                    s_menu_mode = false;
                    s_current_page = &sensor_page;
                }
                else if (msg.event == UI_EVENT_CW && s_current_page->on_cw)
                    s_current_page->on_cw();
                else if (msg.event == UI_EVENT_CCW && s_current_page->on_ccw)
                    s_current_page->on_ccw();
                else if (msg.event == UI_EVENT_BTN && s_current_page->on_btn)
                    s_current_page->on_btn();
            }
            if (msg.value_count > 0)
            {
                memcpy(last_values, msg.values, sizeof(float) * msg.value_count);
                last_value_count = msg.value_count;
            }
        }

        // --- Periodic Refresh Logic ---
        uint64_t current_time_ms = esp_timer_get_time() / 1000;
        if (!s_menu_mode && s_current_page == &sensor_page && (current_time_ms - last_sensor_refresh_ms > sensor_refresh_interval_ms))
        {
            // Send command directly to datalogger to prevent resetting the main inactivity timer.
            if (g_datalogger_cmd_queue != NULL)
            {
                datalogger_command_t cmd = DATALOGGER_CMD_FORCE_REFRESH;
                xQueueSend(g_datalogger_cmd_queue, &cmd, 0);
            }
            last_sensor_refresh_ms = current_time_ms;
        }

        // Centralized buffer copy for rendering
        if (xSemaphoreTake(g_sensor_buffer_mutex, pdMS_TO_TICKS(50)) == pdTRUE)
        {
            s_local_sensor_buffer = g_sensor_buffer;
            xSemaphoreGive(g_sensor_buffer_mutex);
        }

        // Always call the correct render callback
        if (s_sleeping_mode)
        {
            // Do nothing, screen is already showing the sleep message
        }
        else if (s_waking_up)
        {
            // We are in the wake-up debounce period.
            vTaskDelay(pdMS_TO_TICKS(200)); // Wait for the button to be released
            xQueueReset(ui_event_queue);    // Flush any stray input events
            s_waking_up = false;            // Clear the flag to resume normal operation
            // The screen will be re-rendered in the next loop iteration
        }
        else if (s_cmd_pending_mode)
        {
            render_cmd_feedback_screen();
        }
        else if (s_menu_mode)
        {
            render_menu_callback();
        }
        else if (s_current_page == &sensor_page)
        {
            if (s_current_page->render)
            {
                s_current_page->render();
            }
        }
        else if (s_current_page && s_current_page->render)
        {
            s_current_page->render();
        }
        draw_status_icons();
        i2c_oled_update_screen(s_oled_i2c_num);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// --- About event handlers (to be referenced in ui_menudef.h) ---
void menu_about_on_btn(void)
{
    s_qr_code_cached = false;
    s_about_page_view = 0; // Reset to default view (QR code)
    s_menu_mode = false;
    s_current_page = &about_page;
}
void menu_sensor_on_btn(void)
{
    // When entering sensor page, always reset to the first page
    s_sensor_page_num = 0;
    // Also trigger a refresh of SD card stats to ensure page 2 is up-to-date
    app_command_t cmd_file_count = APP_CMD_GET_SD_FILE_COUNT;
    xQueueSend(g_app_cmd_queue, &cmd_file_count, 0);

    s_menu_mode = false;
    // Force a refresh when entering the page to get immediate data.
    // ESP_LOGI(TAG, "menu_sensor_on_btn called, returning to sensor page. 3");
    if (g_app_cmd_queue != NULL)
    {
        app_command_t cmd = APP_CMD_REFRESH_SENSOR_DATA;
        xQueueSend(g_app_cmd_queue, &cmd, 0);
    }
    else
    {
        ESP_LOGE(TAG, "App command queue not initialized!");
    }
    s_current_page = &sensor_page;
}

void menu_fs_stats_on_btn(void)
{
    // Always trigger a new read when entering the stats page
    if (g_app_cmd_queue != NULL)
    {
        // Set buffer to "loading" state before sending commands
        if (xSemaphoreTake(g_sensor_buffer_mutex, portMAX_DELAY))
        {
            g_sensor_buffer.sd_card_file_count = -2; // -2 indicates loading
            g_sensor_buffer.sd_card_free_bytes = -2; // -2 indicates loading
            xSemaphoreGive(g_sensor_buffer_mutex);
        }
        app_command_t cmd_file_count = APP_CMD_GET_SD_FILE_COUNT;
        xQueueSend(g_app_cmd_queue, &cmd_file_count, 0);
        app_command_t cmd_free_space = APP_CMD_GET_SD_FREE_SPACE;
        xQueueSend(g_app_cmd_queue, &cmd_free_space, 0);
    }
    s_menu_mode = false;
    s_current_page = &fs_stats_page;
}

void menu_format_sd_confirm_on_btn(void)
{
    if (g_app_cmd_queue != NULL)
    {
        app_command_t cmd = APP_CMD_FORMAT_SD_CARD;
        xQueueSend(g_app_cmd_queue, &cmd, 0);
        enter_cmd_pending_mode(30000, POST_ACTION_GO_BACK); // 30s timeout, then go back
    }
    else
    {
        ESP_LOGE(TAG, "App command queue not initialized!");
    }
}

void menu_sync_rtc_ntp_on_btn(void)
{
    if (g_app_cmd_queue != NULL)
    {
        app_command_t cmd = APP_CMD_SYNC_RTC_NTP;
        xQueueSend(g_app_cmd_queue, &cmd, 0);
        enter_cmd_pending_mode(45000, POST_ACTION_GO_BACK); // 45s timeout, then go back
    }
    else
    {
        ESP_LOGE(TAG, "App command queue not initialized!");
    }
}

void menu_set_time_on_btn(void)
{
    if (g_app_cmd_queue != NULL)
    {
        app_command_t cmd = APP_CMD_SET_RTC_BUILD_TIME;
        xQueueSend(g_app_cmd_queue, &cmd, 0);
        enter_cmd_pending_mode(5000, POST_ACTION_GO_BACK); // 5s timeout, then go back
    }
    else
    {
        ESP_LOGE(TAG, "App command queue not initialized!");
    }
}

void menu_hf_mode_enable_on_btn(void)
{
    app_command_t cmd = APP_CMD_ENABLE_HF_MODE;
    xQueueSend(g_app_cmd_queue, &cmd, 0);
    // After action, go back to the main menu
    current_page = 0;
    current_item = 0;
    s_menu_mode = true;
    s_current_page = NULL;
}

void menu_hf_mode_disable_on_btn(void)
{
    app_command_t cmd = APP_CMD_DISABLE_HF_MODE;
    xQueueSend(g_app_cmd_queue, &cmd, 0);
    menu_cancel_on_btn(); // Go back to previous menu
}

void page_about_on_btn(void)
{
    s_menu_mode = true;
    s_current_page = NULL;
}
void page_about_on_cw(void)
{
    s_about_page_view = (s_about_page_view + 1) % 3; // Cycle through 0, 1, 2
}
void page_about_on_ccw(void)
{
    s_about_page_view = (s_about_page_view > 0) ? (s_about_page_view - 1) : 2; // Cycle through 2, 1, 0
}
void page_sensor_on_btn(void)
{
    // Button press on sensor page enters the menu
    s_menu_mode = true;
    s_current_page = NULL;
    current_page = 0;
    current_item = 0;
}
void page_sensor_on_cw(void)
{
    // Rotation on sensor page toggles between sub-pages
    s_sensor_page_num = (s_sensor_page_num + 1) % SENSOR_PAGE_COUNT;
}
void page_sensor_on_ccw(void)
{
    // Rotation on sensor page toggles between sub-pages
    s_sensor_page_num = (s_sensor_page_num > 0) ? (s_sensor_page_num - 1) : (SENSOR_PAGE_COUNT - 1);
}

void page_fs_stats_on_cw(void) { /* Do nothing */ }
void page_fs_stats_on_ccw(void) { /* Do nothing */ }

void page_fs_stats_on_btn(void)
{
    s_menu_mode = true;
    s_current_page = NULL;
}

void menu_web_server_on_btn(void)
{
    // Send the start command and enter a pending state.
    // The main_task will handle the state transitions.
    // The UI will show "Loading..." and then automatically switch to the
    // web server page upon success, or show "Failed" and return.
    if (g_app_cmd_queue != NULL)
    {
        app_command_t cmd = APP_CMD_START_WEB_SERVER;
        xQueueSend(g_app_cmd_queue, &cmd, 0);
        // We don't use enter_cmd_pending_mode here because the web server page
        // has its own complex state rendering. We just need to switch to it.
        s_menu_mode = false;
        s_current_page = &web_server_page;
    }
}

void page_web_server_on_cw(void)
{
    s_web_page_view = (s_web_page_view + 1) % 2; // Toggle between 0 and 1
}
void page_web_server_on_ccw(void)
{
    s_web_page_view = (s_web_page_view + 1) % 2; // Toggle between 0 and 1
}

void menu_config_on_btn(void)
{
    s_menu_mode = false;
    s_config_current_page_index = 0;
    ui_config_page_prepare_data();
    s_current_page = &config_page;
}

void page_config_on_btn(void)
{
    s_menu_mode = true;
    s_current_page = NULL;
}

/* --- Config Page Callbacks --- */
void page_config_on_cw(void)
{
    int max_pages = (s_num_config_items + ITEMS_PER_PAGE - 1) / ITEMS_PER_PAGE;
    s_config_current_page_index++;
    if (s_config_current_page_index >= max_pages)
    {
        s_config_current_page_index = 0; // Wrap around to the first page
    }
}
void page_config_on_ccw(void)
{
    s_config_current_page_index--;
    if (s_config_current_page_index < 0)
    {
        int max_pages = (s_num_config_items + ITEMS_PER_PAGE - 1) / ITEMS_PER_PAGE;
        s_config_current_page_index = max_pages - 1; // Wrap around to the last page
    }
}

static void ui_config_page_prepare_data(void)
{
    const config_params_t *params = config_params_get();
    s_num_config_items = MAX_CONFIG_ITEMS;

    // Populate the values based on the current config. The keys are now centrally defined.
    s_config_items[0].full_key = s_config_keys[0];
    snprintf(s_config_items[0].value, sizeof(s_config_items[0].value), "%.4f", params->battery_voltage_divider_ratio);
    s_config_items[1].full_key = s_config_keys[1];
    snprintf(s_config_items[1].value, sizeof(s_config_items[1].value), "%lu", (unsigned long)params->inactivity_timeout_ms);
    s_config_items[2].full_key = s_config_keys[2];
    snprintf(s_config_items[2].value, sizeof(s_config_items[2].value), "%llu", (unsigned long long)params->sleep_duration_ms);
    s_config_items[3].full_key = s_config_keys[3]; // hf_sleep_ms
    snprintf(s_config_items[3].value, sizeof(s_config_items[3].value), "%llu", (unsigned long long)params->hf_sleep_duration_ms);
    s_config_items[4].full_key = s_config_keys[4]; // log_int_ms
    snprintf(s_config_items[4].value, sizeof(s_config_items[4].value), "%lu", (unsigned long)params->log_interval_ms);
    s_config_items[5].full_key = s_config_keys[5]; // hf_log_int_ms
    snprintf(s_config_items[5].value, sizeof(s_config_items[5].value), "%lu", (unsigned long)params->hf_log_interval_ms);

    // D6F-PH Model
    s_config_items[6].full_key = s_config_keys[6];
    const char *model_str = "Unknown";
    if (params->d6fph_model == D6FPH_MODEL_0025AD1)
        model_str = "0025AD1";
    else if (params->d6fph_model == D6FPH_MODEL_0505AD3)
        model_str = "0505AD3";
    else if (params->d6fph_model == D6FPH_MODEL_5050AD4)
        model_str = "5050AD4";
    snprintf(s_config_items[6].value, sizeof(s_config_items[6].value), "%s (%d)", model_str, params->d6fph_model);

    // WiFi SSID
    s_config_items[7].full_key = s_config_keys[7];
    snprintf(s_config_items[7].value, sizeof(s_config_items[7].value), "%.20s", params->wifi_ssid);

    // WiFi Password (masked)
    s_config_items[8].full_key = s_config_keys[8];
    if (strlen(params->wifi_password) > 0)
    {
        snprintf(s_config_items[8].value, sizeof(s_config_items[8].value), "********");
    }
    else
    {
        snprintf(s_config_items[8].value, sizeof(s_config_items[8].value), "(not set)");
    }
}

void render_config_callback(void)
{
    if (!s_oled_initialized)
        return;
    char line_buf[21] = {0};

    snprintf(line_buf, sizeof(line_buf), "View Config");
    write_inverted_line(0, line_buf);

    int start_index = s_config_current_page_index * ITEMS_PER_PAGE;

    for (int i = 0; i < ITEMS_PER_PAGE; ++i)
    {
        int item_index = start_index + i;
        int line_num = 1 + (i * 2); // Start from line 1
        if (item_index >= s_num_config_items)
        {
            // Clear the rest of the lines on the last page
            write_padded_line(line_num, "");
            write_padded_line(line_num + 1, "");
            continue; // Continue to clear remaining lines in the loop
        }

        const char *display_key = s_config_items[item_index].full_key;
        // Display the configuration key
        snprintf(line_buf, sizeof(line_buf), "%.20s", display_key);
        write_padded_line(line_num, line_buf);

        snprintf(line_buf, sizeof(line_buf), " %.19s", s_config_items[item_index].value);
        write_padded_line(line_num + 1, line_buf);
    }
}
