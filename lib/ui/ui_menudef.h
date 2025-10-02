#pragma once
#include "ui_render.h"

// Forward declarations for page event handlers
void menu_set_time_on_btn(void);
void menu_about_on_btn(void);
void menu_sensor_on_btn(void);
void menu_cancel_on_btn(void);
void page_about_on_btn(void);
void page_about_on_cw(void);
void page_about_on_ccw(void);
void page_sensor_on_btn(void);
void page_sensor_on_cw(void);
void page_sensor_on_ccw(void);
void render_about_callback(void);
void render_sensor_callback(void);

#define MENU_BACK_ITEM { \
    .label = "Back", \
    .has_submenu = false, \
    .on_cw = NULL, \
    .on_ccw = NULL, \
    .on_btn = menu_cancel_on_btn \
}

const ui_menu_page_t ui_menu_tree[] = {
    {
        .title = "Main Menu",
        .items = {
            { .label = "Sensor data", .has_submenu = false, .on_btn = menu_sensor_on_btn },
            { .label = "Settings", .has_submenu = true, .submenu_page_index = 1, .on_btn = NULL },
            { .label = "About", .has_submenu = false, .on_btn = menu_about_on_btn }
           
        },
        .item_count = 3
    },
    {
        .title = "Settings",
        .items = {
            { .label = "Build ts to RTC", .has_submenu = false, .on_btn = menu_set_time_on_btn },
            MENU_BACK_ITEM
        },
        .item_count = 2
    }
    ,
    {
        .title = "Ovf Sub-Menu",
        .items = {
            { .label = "Sub-Item 1", .has_submenu = false, .on_btn = NULL },
            { .label = "Sub-Item 2", .has_submenu = false, .on_btn = NULL },
            { .label = "Sub-Item 3", .has_submenu = false, .on_btn = NULL },

            MENU_BACK_ITEM
        },
        .item_count = 4
    }
};
const uint8_t ui_menu_page_count = sizeof(ui_menu_tree) / sizeof(ui_menu_tree[0]);

// Standalone pages
const ui_page_t about_page = {
    .on_cw = page_about_on_cw,
    .on_ccw = page_about_on_ccw,
    .on_btn = page_about_on_btn,
    .render = render_about_callback
};
const ui_page_t sensor_page = {
    .on_cw = page_sensor_on_cw,
    .on_ccw = page_sensor_on_ccw,
    .on_btn = page_sensor_on_btn,
    .render = render_sensor_callback
};