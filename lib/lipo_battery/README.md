# LiPo Battery Monitor

This library provides functions to monitor the status of a single-cell LiPo battery.

## Features

- Reads the battery voltage via an ADC pin connected to a voltage divider.
- Estimates the remaining battery percentage based on the voltage.
- Detects if the device is being powered externally (e.g., via USB).