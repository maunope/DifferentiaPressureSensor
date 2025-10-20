# SPI SD Card Manager

This library provides a high-level interface for interacting with an SD card over the SPI bus. It also integrates with TinyUSB to expose the SD card as a USB Mass Storage Device (MSC).

## Features

- Initializes the SD card and FAT filesystem.
- Handles writing data logs to CSV files, with automatic file rotation.
- Provides functions to format the SD card and retrieve statistics (file count, free space).
- Manages the USB MSC stack for accessing the SD card from a computer.