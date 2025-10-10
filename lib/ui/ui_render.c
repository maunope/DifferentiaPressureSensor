#include "ui_render.h"
#include "i2c-oled.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <string.h>
#include "esp_timer.h"
#include <ui_menudef.h>
#include "esp_log.h"
#include "time_utils.h"
#include "../buffers.h"

#define UI_QUEUE_LEN 8
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

typedef enum
{
    POST_ACTION_NONE,
    POST_ACTION_GO_BACK,
} post_cmd_action_t;

static i2c_port_t s_oled_i2c_num;
static gpio_num_t s_oled_sda, s_oled_scl;
static bool s_oled_initialized = false;

// UI mode: true = menu, false = page
static bool s_menu_mode = true;
static bool s_sleeping_mode = false;
static bool s_waking_up = false; // Flag to indicate we are waking up and should ignore input
static const ui_page_t *s_current_page = NULL;

// Command feedback UI state
static bool s_cmd_pending_mode = false;
static uint64_t s_cmd_start_time_ms = 0;
static uint32_t s_cmd_timeout_ms = 5000; // Default timeout
static post_cmd_action_t s_post_cmd_action = POST_ACTION_NONE;

// Data for rendering
static float last_values[8] = {0};
static int last_value_count = 0;

// Menu navigation state
static uint8_t current_page = 0;
static uint8_t current_item = 0;

// --- Initialization and event send ---
void uiRender_init(i2c_port_t oled_i2c_num, gpio_num_t sda, gpio_num_t scl)
{
    s_oled_i2c_num = oled_i2c_num;
    s_oled_sda = sda;
    s_oled_scl = scl;
    i2c_oled_init(s_oled_i2c_num, s_oled_sda, s_oled_scl);
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
    static char last_lines[8][21] = {{0}};
    char new_lines[8][21] = {{0}};

    const ui_menu_page_t *page = &ui_menu_tree[current_page];
    int total = page->item_count;
    int win = UI_MENU_VISIBLE_ROWS;
    int start = 0;

    // Clamp current_item to valid range
    if (current_item >= total)
        current_item = total - 1;
    if (current_item < 0)
        current_item = 0;

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

// --- About rendering callback ---
void render_about_callback(void)
{
    if (!s_oled_initialized)
        return;
    static char last_lines[8][21] = {{0}};
    char new_lines[8][21] = {{0}};
    snprintf(new_lines[0], sizeof(new_lines[0]), "About");
    snprintf(new_lines[2], sizeof(new_lines[2]), "Differential");
    snprintf(new_lines[3], sizeof(new_lines[3]), "Pressure Sensor");
    snprintf(new_lines[4], sizeof(new_lines[4]), "V1.0");
    // The rest remain empty

    write_inverted_line(0, new_lines[0]);
    write_padded_line(1, ""); // Blank line
    for (uint8_t row = 2; row < 8; ++row)
    {
        write_padded_line(row, new_lines[row]);
        strncpy(last_lines[row], new_lines[row], sizeof(last_lines[row]));
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
    i2c_oled_write_text(s_oled_i2c_num, 4, 5, "Zzzzz....");
}

// --- Sensor rendering callback ---
void render_sensor_callback(void)
{
    if (!s_oled_initialized)
        return;
    static char last_lines[8][21] = {{0}};
    char new_lines[8][21] = {{0}};

    sensor_buffer_t local_buffer;

    if (g_sensor_buffer_mutex && xSemaphoreTake(g_sensor_buffer_mutex, pdMS_TO_TICKS(50)) == pdTRUE)
    {
        local_buffer = g_sensor_buffer;
        xSemaphoreGive(g_sensor_buffer_mutex);

        struct tm local_tm;
        convert_gmt_to_cet(local_buffer.timestamp, &local_tm);
        char time_str[20];
        strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", &local_tm);

        // new_lines[1] is intentionally left blank
        snprintf(new_lines[0], sizeof(new_lines[0]), "Stato Letture");
        snprintf(new_lines[2], sizeof(new_lines[2]), "%s", time_str);
        snprintf(new_lines[3], sizeof(new_lines[3]), "T: %.2f C", local_buffer.temperature_c);
        snprintf(new_lines[4], sizeof(new_lines[4]), "P: %ld Pa", local_buffer.pressure_pa);
        snprintf(new_lines[5], sizeof(new_lines[5]), "File write: %s", local_buffer.writeStatus == 0 ? "OK" : "KO");
        snprintf(new_lines[6], sizeof(new_lines[6]), "Batt: %.2fV-%d%% %s", local_buffer.battery_voltage, local_buffer.battery_percentage, local_buffer.battery_externally_powered == 1 ? "(C)" : "");
    }
    else
    {
        return; // Don't update display if we can't get the data
    }

    write_inverted_line(0, new_lines[0]);
    for (uint8_t row = 1; row < 8; ++row)
    {
        write_padded_line(row, new_lines[row]);
        strncpy(last_lines[row], new_lines[row], sizeof(last_lines[row]));
    }
}

void page_fs_stats_render_callback(void)
{
    char new_lines[8][21] = {{0}}; // Initialize all lines to empty
    snprintf(new_lines[0], sizeof(new_lines[0]), "File System Stats");

    int file_count = -2; // Default to loading state
    int free_space_mb = -2;
    if (g_sensor_buffer_mutex && xSemaphoreTake(g_sensor_buffer_mutex, pdMS_TO_TICKS(50)) == pdTRUE)
    {
        file_count = g_sensor_buffer.sd_card_file_count;
        free_space_mb = g_sensor_buffer.sd_card_free_bytes;
        xSemaphoreGive(g_sensor_buffer_mutex);
    }

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
    if (current_item < 0)
        current_item = 0;
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
        if (current_item < 0)
            current_item = 0;
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
        if (current_item < 0)
            current_item = 0;
    }
    s_menu_mode = true;
    s_current_page = NULL;
}

void render_cmd_feedback_screen(void)
{
    write_inverted_line(0, " "); // Inverted title bar
    uint64_t now = esp_timer_get_time() / 1000;

    command_status_t current_status = CMD_STATUS_IDLE;
    if (xSemaphoreTake(g_command_status_mutex, pdMS_TO_TICKS(50)) == pdTRUE)
    {
        current_status = g_command_status;
        xSemaphoreGive(g_command_status_mutex);
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
        }
    }
    else if (current_status == CMD_STATUS_SUCCESS)
    {
        write_padded_line(3, "Success");
        vTaskDelay(pdMS_TO_TICKS(300));
        s_cmd_pending_mode = false; // Exit loading mode
        if (xSemaphoreTake(g_command_status_mutex, portMAX_DELAY))
        {
            g_command_status = CMD_STATUS_IDLE;
            xSemaphoreGive(g_command_status_mutex);
        }
        if (s_post_cmd_action == POST_ACTION_GO_BACK)
        {
            menu_cancel_on_btn();
        }
        s_post_cmd_action = POST_ACTION_NONE; // Clear action
    }
    else if (current_status == CMD_STATUS_FAIL)
    {
        write_padded_line(3, "Failed");
        vTaskDelay(pdMS_TO_TICKS(500));
        s_cmd_pending_mode = false; // Exit loading mode
        if (xSemaphoreTake(g_command_status_mutex, portMAX_DELAY))
        {
            g_command_status = CMD_STATUS_IDLE;
            xSemaphoreGive(g_command_status_mutex);
        }
        s_post_cmd_action = POST_ACTION_NONE; // Clear action
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
}

void uiRender_reset_activity_timer(void)
{
    app_command_t cmd = APP_CMD_ACTIVITY_DETECTED;
    xQueueSend(g_app_cmd_queue, &cmd, 0);
}

// --- UI task ---
void uiRender_task(void *pvParameters)
{
    ui_event_queue = xQueueCreate(UI_QUEUE_LEN, sizeof(ui_event_msg_t));
    uiRender_reset_activity_timer();
    
    uint64_t last_sensor_refresh_ms = 0;
    const uint32_t sensor_refresh_interval_ms = 5000; // 5 seconds

    uint64_t last_fs_stats_refresh_ms = 0;
    const uint32_t fs_stats_refresh_interval_ms = 5000; // 5 seconds

    while (1)
    {
        ui_event_msg_t msg;
        if (xQueueReceive(ui_event_queue, &msg, pdMS_TO_TICKS(100)))
        {

            if (msg.event != UI_EVENT_PREPARE_SLEEP)
            {
                uiRender_reset_activity_timer();
            }

            if (msg.event == UI_EVENT_PREPARE_SLEEP)
            {
                s_waking_up = false; // Clear waking up flag if we are going to sleep
                s_sleeping_mode = true;
                render_sleeping_screen(); // Draw the sleeping message immediately
            }
            else if (msg.event == UI_EVENT_WAKE_UP)
            {
                s_sleeping_mode = false;
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
                else if (msg.event == UI_EVENT_BTN)
                    menu_on_btn();
            }
            else if (s_current_page)
            {
                if (msg.event == UI_EVENT_CW && s_current_page->on_cw)
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
            uint64_t current_time_ms = esp_timer_get_time() / 1000;
            if (current_time_ms - last_sensor_refresh_ms >= sensor_refresh_interval_ms) {
                last_sensor_refresh_ms = current_time_ms;
                if (g_app_cmd_queue != NULL) {
                    app_command_t cmd = APP_CMD_REFRESH_SENSOR_DATA;
                    xQueueSend(g_app_cmd_queue, &cmd, 0);
                }
            }
            if (s_current_page->render) {
                s_current_page->render();
            }
        }
        else if (s_current_page == &fs_stats_page)
        {
            uint64_t current_time_ms = esp_timer_get_time() / 1000;
            if (current_time_ms - last_fs_stats_refresh_ms >= fs_stats_refresh_interval_ms) {
                last_fs_stats_refresh_ms = current_time_ms;
                if (g_app_cmd_queue != NULL) {
                    app_command_t cmd_file_count = APP_CMD_GET_SD_FILE_COUNT;
                    xQueueSend(g_app_cmd_queue, &cmd_file_count, 0);
                    app_command_t cmd_free_space = APP_CMD_GET_SD_FREE_SPACE;
                    xQueueSend(g_app_cmd_queue, &cmd_free_space, 0);
                }
            }
            if (s_current_page->render) {
                s_current_page->render();
            }
        }
        else if (s_current_page && s_current_page->render)
        {
            s_current_page->render();
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// --- About event handlers (to be referenced in ui_menudef.h) ---
void menu_about_on_btn(void)
{
    s_menu_mode = false;
    s_current_page = &about_page;
}
void menu_sensor_on_btn(void)
{
    s_menu_mode = false;
    if (g_app_cmd_queue != NULL)
    {
        app_command_t cmd = APP_CMD_REFRESH_SENSOR_DATA;
        xQueueSend(g_app_cmd_queue, &cmd, 0);
    }
    else
    {
        ESP_LOGE("UI", "App command queue not initialized!");
    }
    
    s_current_page = &sensor_page;
}

void menu_fs_stats_on_btn(void)
{
    // Always trigger a new read when entering the stats page
    if (g_app_cmd_queue != NULL)
    {
        // Set buffer to "loading" state
        if (xSemaphoreTake(g_sensor_buffer_mutex, portMAX_DELAY))
        {
            g_sensor_buffer.sd_card_file_count = -2; // -2 indicates loading for file count
            g_sensor_buffer.sd_card_free_bytes = -2; // -2 indicates loading for free space
            xSemaphoreGive(g_sensor_buffer_mutex);
        }
        // Send commands to get both stats
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
        s_cmd_pending_mode = true; // Enter loading screen mode
        s_cmd_start_time_ms = esp_timer_get_time() / 1000;
        s_cmd_timeout_ms = 30000; // 30-second timeout for format
        s_post_cmd_action = POST_ACTION_GO_BACK;
        if (xSemaphoreTake(g_command_status_mutex, portMAX_DELAY))
        {
            g_command_status = CMD_STATUS_PENDING; // Set initial status for UI
            xSemaphoreGive(g_command_status_mutex);
        }
    }
    else
    {
        ESP_LOGE("UI", "App command queue not initialized!");
    }
}

void menu_set_time_on_btn(void)
{
    if (g_app_cmd_queue != NULL)
    {
        app_command_t cmd = APP_CMD_SET_RTC_BUILD_TIME;
        xQueueSend(g_app_cmd_queue, &cmd, 0);
        s_cmd_pending_mode = true; // Enter loading screen mode
        s_cmd_start_time_ms = esp_timer_get_time() / 1000;
        s_post_cmd_action = POST_ACTION_NONE;
        s_cmd_timeout_ms = 5000; // 5-second timeout for RTC set
        if (xSemaphoreTake(g_command_status_mutex, portMAX_DELAY))
        {
            g_command_status = CMD_STATUS_PENDING; // Set initial status for UI
            xSemaphoreGive(g_command_status_mutex);
        }
    }
    else
    {
        ESP_LOGE("UI", "App command queue not initialized!");
    }
}
void page_about_on_btn(void)
{
    s_menu_mode = true;
    s_current_page = NULL;
}
void page_about_on_cw(void) { /* Do nothing */ }
void page_about_on_ccw(void) { /* Do nothing */ }
void page_sensor_on_btn(void)
{
    s_menu_mode = true;
    s_current_page = NULL;
}
void page_sensor_on_cw(void) { /* Do nothing */ }
void page_sensor_on_ccw(void) { /* Do nothing */ }

void page_fs_stats_on_btn(void)
{
    s_menu_mode = true;
    s_current_page = NULL;
}
void page_fs_stats_on_cw(void) { /* Do nothing */ }
void page_fs_stats_on_ccw(void) { /* Do nothing */ }