# D6F-PH Differential Pressure Sensor Driver

This library provides a driver for the Omron D6F-PH series of I2C differential pressure sensors. It is designed for the ESP-IDF framework.

## Features

*   Initialize the sensor with a specific model type.
*   Trigger and read pressure measurements.
*   Calculates the final pressure in Pascals (Pa) based on the sensor's model-specific formula.

## Supported Models

*   `D6FPH_MODEL_0025AD1` (+/- 250 Pa)
*   `D6FPH_MODEL_0505AD3` (+/- 50 Pa)
*   `D6FPH_MODEL_5050AD4` (+/- 500 Pa)

## Usage

1.  Initialize the sensor using `d6fph_init()`, providing an I2C port, the sensor's I2C address, and the specific model.
2.  Call `d6fph_read_pressure()` to get the differential pressure in Pascals.