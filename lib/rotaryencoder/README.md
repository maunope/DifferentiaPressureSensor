# Rotary Encoder Driver

This library provides a robust driver for a rotary encoder with a built-in push-button, designed for user interface control.

## Features

- Runs as a dedicated high-priority FreeRTOS task to ensure responsiveness.
- Decodes rotational events (clockwise and counter-clockwise).
- Detects both short and long button presses with debouncing.
- Uses callbacks to notify the main application of input events.