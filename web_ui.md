# Web User Interface Guide

This document explains the features and functionality of the web-based user interface, which can be accessed when the device's web server is active.

## Overview

The web UI provides a convenient way to monitor the device, view live data, and manage log files directly from a web browser on a computer or smartphone connected to the same Wi-Fi network.

The interface is divided into three main sections:
1.  **Live Sensor Data**: A real-time dashboard of sensor readings.
2.  **File Browser**: A list of all data log files stored on the SD card.
3.  **File Previewer**: An interactive tool for viewing the contents of log files without downloading them.

## Live Sensor Data

The panel at the top of the page displays the most recent data read from the device's sensors. This data is automatically refreshed every 5 seconds and includes:

*   **Timestamp**: The current date and time from the device's RTC.
*   **Temperature**: Ambient temperature in Celsius.
*   **Pressure**: Atmospheric pressure in kilopascals (kPa).
*   **Diff. Pressure**: Differential pressure in Pascals (Pa).
*   **Battery**: Voltage, percentage, and charging status.

If the web page fails to fetch data, a banner will appear with a "Retry" button to re-establish the connection.

## File Browser

This section lists all files found on the SD card, sorted by the most recently modified. Each entry provides the following information and actions:

*   **Filename**: The name of the log file. Clicking this link will download the file directly to your computer.
*   **File Metadata**: The file size and the date it was last modified.
*   **Preview Button**: Opens the interactive File Previewer for that file.
*   **Delete Button**: Permanently deletes the file from the SD card after a confirmation prompt.

## File Previewer

The File Previewer is a powerful tool for inspecting log data directly on the device.

### CSV vs. Raw Text Files

The previewer automatically detects the file type:
*   **CSV Files**: If the file is recognized as a CSV log, it will be displayed in a scrollable table with a header. At the top, sparkline graphs provide a quick visual summary of the data currently loaded in the table.
*   **Non-CSV Files**: If the file is not a standard log file (e.g., `config.ini`), its contents will be displayed as plain, raw text.

### Navigating CSV Data

The previewer is designed to handle very large files efficiently by only loading small chunks of data at a time.

#### Navigation Buttons (`<<< head`, `>>> tail`, Time Jumps)

The buttons below the preview table (`<<< head`, `< Hour`, `Day >>`, etc.) are used for **windowed navigation**. When you click one of these buttons:

*   The currently displayed data is **replaced** with a new chunk of data from a different part of the file.
*   This allows you to quickly "jump" to the beginning, end, or a specific time in the log without loading all the data in between.
*   The number of visible rows is reset to the default chunk size (e.g., 100 lines).

This is the fastest way to move through large files.

#### Scrolling with the Mouse

When you scroll up or down within the preview table, the behavior is different:

*   The previewer will automatically fetch and **append (or prepend)** new lines of data to the table.
*   This allows you to create a longer, continuous view of the data, extending the number of visible points up to a maximum limit (e.g., 1000 rows).

#### Combining Navigation Methods

You can use both methods together. For example, you can scroll to load a few hundred lines of data for detailed inspection. If you then click a navigation button like `>>> tail`, the view will reset, clearing the extended list and showing only the last chunk of data from the end of thefile.