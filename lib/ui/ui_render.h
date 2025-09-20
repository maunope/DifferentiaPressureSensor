#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "i2c-oled.h"


#define UI_MAX_MENU_ITEMS 20
#define UI_MAX_LABEL_LEN  16
#define UI_MAX_PAGES      4

typedef void (*ui_action_callback_t)(void);

typedef struct {
    char label[UI_MAX_LABEL_LEN];
    bool has_submenu;
    uint8_t submenu_page_index; // Index in ui_menu_tree if submenu
    ui_action_callback_t on_cw;
    ui_action_callback_t on_ccw;
    ui_action_callback_t on_btn;
} ui_menu_item_t;

typedef struct {
    char title[UI_MAX_LABEL_LEN];
    ui_menu_item_t items[UI_MAX_MENU_ITEMS];
    uint8_t item_count;
} ui_menu_page_t;

// --- Page definition struct for standalone pages ---
typedef struct {
    void (*on_cw)(void);
    void (*on_ccw)(void);
    void (*on_btn)(void);
    void (*render)(void);
} ui_page_t;

// Externally defined menu pages and standalone pages
extern const ui_menu_page_t ui_menu_pages[];
extern const uint8_t ui_menu_page_count;
extern const ui_page_t about_page;
extern const ui_page_t sensor_page;
extern const ui_page_t settings_page;

// UI task API
void uiRender_init(i2c_port_t oled_i2c_num, gpio_num_t sda, gpio_num_t scl);
void uiRender_task(void *pvParameters);
void uiRender_send_event(int event, float *values, int value_count);

typedef enum {
    UI_EVENT_CW,
    UI_EVENT_CCW,
    UI_EVENT_BTN
} ui_event_t;