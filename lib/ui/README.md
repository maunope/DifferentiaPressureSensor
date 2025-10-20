# User Interface (UI) Manager

This library is the core of the device's user interface. It runs as a dedicated FreeRTOS task, processing input events and rendering all screens and menus.

## Features

- Renders all pages, including the main sensor data screen and the menu system.
- Processes input events (button presses, rotation) from the `rotaryencoder` task.
- Manages UI state, including menu navigation, page transitions, and command feedback screens (e.g., "Loading...").
- Displays persistent status icons for battery and SD card status.