# UI Rendering Engine

This library is a complete UI rendering engine for the project, designed to work with an I2C OLED display and a rotary encoder. It manages the menu system, data display pages, and user interaction.

## Features

*   **Menu System**: A hierarchical menu system defined in `ui_menudef.h`.
*   **Page Rendering**: Callbacks for rendering different pages (e.g., sensor data, "About" screen, configuration viewer).
*   **Event-Driven**: Uses a FreeRTOS queue to process events from the rotary encoder (rotate, press, long press) and other system tasks.
*   **Asynchronous Commands**: Handles long-running commands (like formatting the SD card or NTP sync) by displaying a "Loading..." screen and showing a success/fail status.
*   **Power Management**: Responds to sleep/wake events to power down the display and debounce wake-up inputs.
*   **Flicker-Free Updates**: Uses line-by-line updates and padded strings to prevent screen flicker.

## Key Components

*   `ui_render.c`: The core task and logic for handling events and rendering.
*   `ui_menudef.h`: Defines the structure of the menu tree and the actions associated with each item.
*   `time_utils.h`: Helper functions for time conversion (e.g., GMT to CET).

## How It Works

The `uiRender_task` runs in a loop, waiting for events on a queue. Based on the event and the current UI state (menu mode, page mode, sleeping), it calls the appropriate handler functions to update the UI state and trigger a re-render of the display. It sends commands to the main application task (`main_task`) for execution and monitors a global status variable for feedback.

## How to Modify the Menu

The entire menu structure and the actions associated with it are defined in `ui_menudef.h`. To add or change menu items, you'll primarily be editing this file and `ui_render.c`.

### Menu Structure

The menu is defined by an array of `ui_menu_page_t` structs called `ui_menu_tree`. Each element in this array represents a screen with a title and a list of items.

```c
// From ui_menudef.h
const ui_menu_page_t ui_menu_tree[] = {
    { // Page 0: Main Menu
        .title = "Main Menu",
        .items = {
            { .label = "Sensor data", .has_submenu = false, .on_btn = menu_sensor_on_btn },
            { .label = "Options", .has_submenu = true, .submenu_page_index = 1 },
            // ...
        },
        .item_count = 3,
    },
    { // Page 1: Options
        .title = "Options",
        .items = { ... },
        .item_count = 4
    },
    // ... more pages
};
```

### Adding a New Menu Item

Let's say you want to add a new "Reboot Device" option to the "Options" menu (page index 1).

1.  **Define the Action Callback**: In `ui_render.c`, create a function that will be called when the item is selected. This function will send a command to the main task.

    ```c
    // In ui_render.c
    void menu_reboot_confirm_on_btn(void) {
        // This would be a new function to handle the final confirmation
        app_command_t cmd = APP_CMD_REBOOT_DEVICE; // Assuming you add this to your app_command_t enum
        xQueueSend(g_app_cmd_queue, &cmd, 0);
        // ... set up command feedback UI ...
    }
    ```

2.  **Create a Confirmation Page (Good Practice)**: It's best to have a confirmation screen for actions like rebooting. Add a new page to `ui_menu_tree` in `ui_menudef.h`.

    ```c
    // In ui_menudef.h, add a new page to the ui_menu_tree array
    { // Let's say this becomes page 7
        .title = "Reboot Device?",
        .items = {
            { .label = "Cancel", .has_submenu = false, .on_btn = menu_cancel_on_btn },
            { .label = "Confirm", .has_submenu = false, .on_btn = menu_reboot_confirm_on_btn }
        },
        .item_count = 2
    }
    ```

3.  **Add the Item to the "Options" Menu**: Now, edit the "Options" page (index 1) to include the new item. Make sure it points to your new confirmation page (index 7).

    ```c
    // In ui_menudef.h, inside ui_menu_tree
    { // Page 1: Options
        .title = "Options",
        .items = {
            { .label = "Real Time Clock", .has_submenu = true, .submenu_page_index = 2 },
            { .label = "File System", .has_submenu = true, .submenu_page_index = 3 },
            { .label = "View Config", .has_submenu = false, .on_btn = menu_config_on_btn },
            { .label = "Reboot Device", .has_submenu = true, .submenu_page_index = 7 }, // New Item
            MENU_BACK_ITEM
        },
        .item_count = 5 // IMPORTANT: Increment the item count!
    },
    ```

### Adding a New Standalone Page

Standalone pages (like the sensor data or config viewer) are for displaying information rather than just navigating a menu.

1.  **Define Callbacks**: In `ui_render.c`, create a render function and input handlers for your new page.

    ```c
    // In ui_render.c
    void render_my_new_page_callback(void) { /* ... drawing logic ... */ }
    void page_my_new_page_on_btn(void) { /* ... logic to exit page ... */ }
    ```

2.  **Define the Page Struct**: In `ui_menudef.h`, define a `ui_page_t` constant for your new page.

    ```c
    // In ui_menudef.h
    const ui_page_t my_new_page = {
        .on_cw = NULL, // No action on clockwise rotation
        .on_ccw = NULL,
        .on_btn = page_my_new_page_on_btn, // Action on button press
        .render = render_my_new_page_callback
    };
    ```

3.  **Create a Menu Item to Enter the Page**: In `ui_render.c`, create the `on_btn` handler for the menu item that will switch to your new page.

    ```c
    // In ui_render.c
    void menu_my_new_page_on_btn(void) {
        s_menu_mode = false; // Switch from menu mode to page mode
        s_current_page = &my_new_page; // Set the current page
    }
    ```

4.  **Add the Item to a Menu**: Finally, add an item to a menu in `ui_menudef.h` that uses `menu_my_new_page_on_btn` as its `on_btn` callback.