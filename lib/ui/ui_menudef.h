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
void menu_fs_stats_on_btn(void);
void page_fs_stats_on_cw(void);
void page_fs_stats_on_ccw(void);
void page_fs_stats_on_btn(void);
void menu_format_sd_on_btn(void);
void menu_format_sd_confirm_on_btn(void);
void render_about_callback(void);
void render_sensor_callback(void);
void page_fs_stats_render_callback(void);

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
            { .label = "Real Time Clock", .has_submenu = true, .submenu_page_index = 2, .on_btn = NULL },
            { .label = "File System", .has_submenu = true, .submenu_page_index = 3, .on_btn = NULL },
            MENU_BACK_ITEM
        },
        .item_count = 3
    },
    {
        .title = "Real Time Clock",
        .items = {
            { .label = "Build ts to RTC", .has_submenu = false, .on_btn = menu_set_time_on_btn },
            MENU_BACK_ITEM
        },
        .item_count = 2
    },
    {
        .title = "File System",
        .items = {
            { .label = "Stats", .has_submenu = false, .on_btn = menu_fs_stats_on_btn },
            { .label = "Format SD Card", .has_submenu = true, .submenu_page_index = 4, .on_btn = NULL },
            MENU_BACK_ITEM
        },
        .item_count = 3
    },
    {
        .title = "Format SD Card?",
        .items = {
            { .label = "Cancel", .has_submenu = false, .on_btn = menu_cancel_on_btn },
            { .label = "Confirm", .has_submenu = false, .on_btn = menu_format_sd_confirm_on_btn }
        },
        .item_count = 2
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

const ui_page_t fs_stats_page = {
    .on_cw = page_fs_stats_on_cw,
    .on_ccw = page_fs_stats_on_ccw,
    .on_btn = page_fs_stats_on_btn,
    .render = page_fs_stats_render_callback
};