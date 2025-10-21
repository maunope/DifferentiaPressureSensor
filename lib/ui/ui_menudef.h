#pragma once
#include "ui_render.h"

// --- Forward declarations for page event handlers ---
void menu_sync_rtc_ntp_on_btn(void);
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

void menu_config_on_btn(void);
void page_config_on_btn(void);
void render_config_callback(void);
void page_config_on_cw(void);
void page_config_on_ccw(void);

void page_fs_stats_render_callback(void);

void menu_web_server_on_btn(void);
void page_web_server_on_cw(void);
void page_web_server_on_ccw(void);
void page_web_server_on_btn(void);
void menu_hf_mode_enable_on_btn(void);
void menu_hf_mode_disable_on_btn(void);
void render_web_server_callback(void);

#define MENU_BACK_ITEM { \
    .label = "Back", \
    .has_submenu = false, \
    .on_cw = NULL, \
    .on_ccw = NULL, \
    .on_btn = menu_cancel_on_btn \
}

#define CONFIRM_MENU_PAGE(page_title, confirm_callback) \
    { \
        .title = page_title, \
        .items = { \
            { .label = "Cancel", .has_submenu = false, .on_btn = menu_cancel_on_btn }, \
            { .label = "Confirm", .has_submenu = false, .on_btn = confirm_callback } \
        }, \
        .item_count = 2 \
    }

const ui_menu_page_t ui_menu_tree[] = {
    {
        .title = "Main Menu",
        .items = {
            { .label = "Sensor data", .has_submenu = false, .on_btn = menu_sensor_on_btn },
            { .label = "High Freq. Mode", .has_submenu = true, .submenu_page_index = 8, .on_btn = NULL },
            { .label = "Web Server", .has_submenu = true, .submenu_page_index = 7, .on_btn = NULL },
            { .label = "Options", .has_submenu = true, .submenu_page_index = 1 },
            { .label = "About", .has_submenu = false, .on_btn = menu_about_on_btn },
           
        },
        .item_count = 5,
    },
    {
        .title = "Options",
        .items = {
            { .label = "Real Time Clock", .has_submenu = true, .submenu_page_index = 2, .on_btn = NULL },
            { .label = "File System", .has_submenu = true, .submenu_page_index = 3, .on_btn = NULL },
            { .label = "View Config", .has_submenu = false, .on_btn = menu_config_on_btn },
            MENU_BACK_ITEM
        },
        .item_count = 4
    },
    {
        .title = "Real Time Clock",
        .items = {
            { .label = "Sync RTC w/ NTP", .has_submenu = true, .submenu_page_index = 4, .on_btn =  NULL},
            { .label = "Build ts to RTC", .has_submenu = true, .submenu_page_index = 5, .on_btn = NULL },
            MENU_BACK_ITEM
        },
        .item_count = 3
    },
    {
        .title = "File System",
        .items = {
            { .label = "Stats", .has_submenu = false, .on_btn = menu_fs_stats_on_btn },
            { .label = "Format SD Card", .has_submenu = true, .submenu_page_index = 6, .on_btn = NULL },
            MENU_BACK_ITEM
        },
        .item_count = 3
    },
    // Confirmation Pages
    CONFIRM_MENU_PAGE("Sync RTC w/ NTP?", menu_sync_rtc_ntp_on_btn),       // Page 4
    CONFIRM_MENU_PAGE("Build ts to RTC?", menu_set_time_on_btn),        // Page 5
    CONFIRM_MENU_PAGE("Format SD Card?", menu_format_sd_confirm_on_btn),  // Page 6
    CONFIRM_MENU_PAGE("Start Web Server?", menu_web_server_on_btn),       // Page 7
    { // Page 8: High Freq. Mode
        .title = "High Freq. Mode",
        .items = {
            { .label = "Enable", .has_submenu = false, .on_btn = menu_hf_mode_enable_on_btn },
            { .label = "Disable", .has_submenu = false, .on_btn = menu_hf_mode_disable_on_btn }
        },
        .item_count = 2
    },
}; // ui_menu_tree
const uint8_t ui_menu_page_count = sizeof(ui_menu_tree) / sizeof(ui_menu_tree[0]); // This count updates automatically

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

const ui_page_t config_page = {
    .on_cw = page_config_on_cw,
    .on_ccw = page_config_on_ccw,
    .on_btn = page_config_on_btn,
    .render = render_config_callback
};

const ui_page_t web_server_page = {
    .on_cw = page_web_server_on_cw,
    .on_ccw = page_web_server_on_ccw,
    .on_btn = page_web_server_on_btn,
    .render = render_web_server_callback
};