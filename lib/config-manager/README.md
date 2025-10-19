# Configuration Manager

This library provides a simple INI file parser and NVS (Non-Volatile Storage) manager for ESP-IDF projects.

## Features

*   **Load from SD Card**: On startup, it can load key-value pairs from a specified INI file on an SD card.
*   **Store to NVS**: It stores the loaded configuration into the NVS flash partition.
*   **Type-safe Getters**: Provides getter functions (`config_get_int`, `config_get_float`, `config_get_string`) to retrieve configuration values from NVS with default fallbacks.
*   **Simple INI Parsing**:
    *   Parses `key = value` pairs.
    *   Ignores comments (lines starting with `;` or `#`).
    *   Ignores section headers (lines in `[]`).
    *   Trims whitespace from keys and values.

## Usage

1.  Call `config_init()` at startup to initialize the NVS backend.
2.  Call `config_load_from_sdcard_to_flash()` to parse an INI file and store its contents in NVS.
3.  Use the `config_get_*` functions throughout the application to retrieve configuration values.