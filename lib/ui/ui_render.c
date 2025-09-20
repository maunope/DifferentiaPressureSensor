#include "ui_render.h"
#include "i2c-oled.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <string.h>
#include <ui_menudef.h>
#include "esp_log.h"
#include "../buffers.h"

#define UI_QUEUE_LEN 8
#define UI_MENU_STACK_DEPTH 4
#define UI_MENU_VISIBLE_ROWS 6

typedef struct {
    ui_event_t event;
    float values[8];
    int value_count;
} ui_event_msg_t;

typedef struct {
    uint8_t page;
    uint8_t item;
} menu_stack_entry_t;

static QueueHandle_t ui_event_queue = NULL;
static menu_stack_entry_t menu_stack[UI_MENU_STACK_DEPTH];
static int menu_stack_pos = 0;

static i2c_port_t s_oled_i2c_num;
static gpio_num_t s_oled_sda, s_oled_scl;
static bool s_oled_initialized = false;

// UI mode: true = menu, false = page
static bool s_menu_mode = true;
static const ui_page_t* s_current_page = NULL;

// Data for rendering
static float last_values[8] = {0};
static int last_value_count = 0;

// Menu navigation state
static uint8_t current_page = 0;
static uint8_t current_item = 0;


// --- Initialization and event send ---
void uiRender_init(i2c_port_t oled_i2c_num, gpio_num_t sda, gpio_num_t scl) {
    s_oled_i2c_num = oled_i2c_num;
    s_oled_sda = sda;
    s_oled_scl = scl;
    i2c_oled_init(s_oled_i2c_num, s_oled_sda, s_oled_scl);
    s_oled_initialized = true;
}

void uiRender_send_event(int event, float *values, int value_count) {
    if (!ui_event_queue) return;
    ui_event_msg_t msg = { .event = event, .value_count = value_count };
    if (values && value_count > 0) {
        memcpy(msg.values, values, sizeof(float) * value_count);
    }
    xQueueSend(ui_event_queue, &msg, 0);
}

// --- Menu rendering callback ---
void render_menu_callback(void) {
    if (!s_oled_initialized) return;
    static char last_lines[8][20] = {{0}};
    char new_lines[8][20] = {{0}};

    const ui_menu_page_t *page = &ui_menu_tree[current_page];
    int total = page->item_count;
    int win = UI_MENU_VISIBLE_ROWS;
    int start = 0;
   
    // Clamp current_item to valid range
    if (current_item >= total) current_item = total - 1;
    if (current_item < 0) current_item = 0;

    // Calculate window start so current_item is centered if possible
    if (total > win) {
        start = current_item - win / 2;
        if (start < 0) start = 0;
        if (start > total - win) start = total - win;
    }

    snprintf(new_lines[0], sizeof(new_lines[0]), "%-16s", page->title);

    for (int i = 0; i < win && (start + i) < total; ++i) {
        int idx = start + i;
        const ui_menu_item_t *item = &page->items[idx];
        // Left arrow for selected, right arrow for submenu
        if (item->has_submenu) {
            snprintf(new_lines[i + 1], sizeof(new_lines[i + 1]), "%c%-13s >", (idx == current_item) ? '>' : ' ', item->label);
        } else {
            snprintf(new_lines[i + 1], sizeof(new_lines[i + 1]), "%c%-15s", (idx == current_item) ? '>' : ' ', item->label);
        }
    }

    // Clear remaining lines
    for (int i = win + 1; i < 8; ++i) {
        new_lines[i][0] = '\0';
    }

    for (uint8_t row = 0; row < 8; ++row) {
        if (new_lines[row][0] != '\0') {
            i2c_oled_write_text(s_oled_i2c_num, row, 0, new_lines[row]);
        } else {
            i2c_oled_write_text(s_oled_i2c_num, row, 0, "                ");
        }
        strncpy(last_lines[row], new_lines[row], sizeof(last_lines[row]));
    }
}

// --- About rendering callback ---
void render_about_callback(void) {
    if (!s_oled_initialized) return;
    static char last_lines[8][20] = {{0}};
    char new_lines[8][20] = {{0}};
    snprintf(new_lines[0], sizeof(new_lines[0]), "%-16s", "About");
    snprintf(new_lines[1], sizeof(new_lines[1]), "%-16s", "Differential");
    snprintf(new_lines[2], sizeof(new_lines[2]), "%-16s", "Pressure Sensor");
    snprintf(new_lines[3], sizeof(new_lines[3]), "%-16s", "V1.0");
    // The rest remain empty

    for (uint8_t row = 0; row < 8; ++row) {
        if (new_lines[row][0] != '\0') {
            i2c_oled_write_text(s_oled_i2c_num, row, 0, new_lines[row]);
        } else {
            i2c_oled_write_text(s_oled_i2c_num, row, 0, "                ");
        }
        strncpy(last_lines[row], new_lines[row], sizeof(last_lines[row]));
    }
}

// --- Sensor rendering callback ---
void render_sensor_callback(void) {
    if (!s_oled_initialized) return;
    static char last_lines[8][20] = {{0}};
    char new_lines[8][20] = {{0}};

    // Copy shared buffer under mutex
    float temperature_c = 0.0f, pressure_pa = 0.0f;
    int writeStatus = -1;
    if (g_sensor_buffer_mutex && xSemaphoreTake(g_sensor_buffer_mutex, pdMS_TO_TICKS(200)) == pdTRUE) {
        temperature_c = g_sensor_buffer.temperature_c;
        pressure_pa = g_sensor_buffer.pressure_pa;
        writeStatus = g_sensor_buffer.writeStatus;
        xSemaphoreGive(g_sensor_buffer_mutex);
    }

    snprintf(new_lines[0], sizeof(new_lines[0]), "%-16s", "Stato letture");
    snprintf(new_lines[1], sizeof(new_lines[1]), "T: %.2f C    ", temperature_c);
    snprintf(new_lines[2], sizeof(new_lines[2]), "P: %.2f Pa   ", pressure_pa);
    snprintf(new_lines[3], sizeof(new_lines[3]), "File write: %s", writeStatus == 0 ? "OK" : "KO");

    for (uint8_t row = 0; row < 8; ++row) {
        if (new_lines[row][0] != '\0') {
            i2c_oled_write_text(s_oled_i2c_num, row, 0, new_lines[row]);
        } else {
            i2c_oled_write_text(s_oled_i2c_num, row, 0, "                ");
        }
        strncpy(last_lines[row], new_lines[row], sizeof(last_lines[row]));
    }
}

// --- Settings rendering callback ---
void render_settings_callback(void) {
    if (!s_oled_initialized) return;
    static char last_lines[8][20] = {{0}};
    char new_lines[8][20] = {{0}};
    snprintf(new_lines[0], sizeof(new_lines[0]), "%-16s", "Settings Page");
    // Fill in more as needed

    for (uint8_t row = 0; row < 8; ++row) {
        if (new_lines[row][0] != '\0') {
            i2c_oled_write_text(s_oled_i2c_num, row, 0, new_lines[row]);
        } else {
            i2c_oled_write_text(s_oled_i2c_num, row, 0, "                ");
        }
        strncpy(last_lines[row], new_lines[row], sizeof(last_lines[row]));
    }
}

// --- Menu event handlers ---
void menu_on_cw(void) {
    const ui_menu_page_t *page = &ui_menu_tree[current_page];
    if (current_item + 1 < page->item_count) {
        current_item++;
    }
    if (current_item >= page->item_count) current_item = page->item_count - 1;
}

void menu_on_ccw(void) {
    if (current_item > 0) {
        current_item--;
    }
    if (current_item < 0) current_item = 0;
}

void menu_on_btn(void) {
    const ui_menu_item_t *item = &ui_menu_tree[current_page].items[current_item];
    if (item->has_submenu) {
        if (menu_stack_pos < UI_MENU_STACK_DEPTH) {
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
        if (current_item >= page->item_count) current_item = page->item_count - 1;
        if (current_item < 0) current_item = 0;
    } else if (item->on_btn) {
        item->on_btn();
    }
}

void menu_cancel_on_btn(void) {
    if (menu_stack_pos > 0) {
        menu_stack_pos--;
        current_page = menu_stack[menu_stack_pos].page;
        current_item = menu_stack[menu_stack_pos].item;
        // Clamp after page change
        const ui_menu_page_t *page = &ui_menu_tree[current_page];
        if (current_item >= page->item_count) current_item = page->item_count - 1;
        if (current_item < 0) current_item = 0;
    }
    s_menu_mode = true;
    s_current_page = NULL;
}

// --- UI task ---
void uiRender_task(void *pvParameters) {
    ui_event_queue = xQueueCreate(UI_QUEUE_LEN, sizeof(ui_event_msg_t));

    while (1) {
        ui_event_msg_t msg;
        if (xQueueReceive(ui_event_queue, &msg, pdMS_TO_TICKS(100))) {
            if (s_menu_mode) {
                if (msg.event == UI_EVENT_CW) menu_on_cw();
                else if (msg.event == UI_EVENT_CCW) menu_on_ccw();
                else if (msg.event == UI_EVENT_BTN) menu_on_btn();
            } else if (s_current_page) {
                if (msg.event == UI_EVENT_CW && s_current_page->on_cw) s_current_page->on_cw();
                else if (msg.event == UI_EVENT_CCW && s_current_page->on_ccw) s_current_page->on_ccw();
                else if (msg.event == UI_EVENT_BTN && s_current_page->on_btn) s_current_page->on_btn();
            }
            if (msg.value_count > 0) {
                memcpy(last_values, msg.values, sizeof(float) * msg.value_count);
                last_value_count = msg.value_count;
            }
        }
        // Always call the correct render callback
        if (s_menu_mode) {
            render_menu_callback();
        } else if (s_current_page && s_current_page->render) {
            s_current_page->render();
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// --- About event handlers (to be referenced in ui_menudef.h) ---
void menu_about_on_btn(void) {
    s_menu_mode = false;
    s_current_page = &about_page;
}
void menu_sensor_on_btn(void) {
    s_menu_mode = false;
    s_current_page = &sensor_page;
}
void menu_settings_on_btn(void) {
    s_menu_mode = false;
    s_current_page = &settings_page;
}
void page_about_on_btn(void) { 
    s_menu_mode = true;
    s_current_page = NULL;
}
void page_settings_on_btn(void) {
    s_menu_mode = true;
    s_current_page = NULL;
}
void page_about_on_cw(void) { /* Do nothing */ }
void page_about_on_ccw(void) { /* Do nothing */ }
void page_sensor_on_btn(void) {
    s_menu_mode = true;
    s_current_page = NULL;
}
void page_sensor_on_cw(void) { /* Do nothing */ }
void page_sensor_on_ccw(void) { /* Do nothing */ }
void page_settings_on_cw(void) { /* Do nothing */ }
void page_settings_on_ccw(void) { /* Do nothing */ }