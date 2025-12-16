#pragma once

#include <stdint.h>

/**
 * @brief Structure to hold the state of a single Kalman filter.
 *
 * This structure is designed to be statically allocated to avoid heap fragmentation.
 * It contains all the necessary state variables for the filter.
 */
typedef struct {
    float q; // Process noise covariance
    float r; // Measurement noise covariance
    float p; // Estimation error covariance
    float x; // Value
    float k; // Kalman gain
} kalman_filter_t;

/**
 * @brief Initializes a Kalman filter with given parameters.
 *
 * @param kf Pointer to the Kalman filter structure to initialize.
 * @param q Process noise covariance. A smaller value indicates a more stable process.
 * @param r Measurement noise covariance. A larger value indicates less trust in the measurement.
 * @param initial_value The initial value to seed the filter with.
 */
void kalman_init(kalman_filter_t *kf, float q, float r, float initial_value);

/**
 * @brief Updates the Kalman filter with a new measurement.
 *
 * @param kf Pointer to the Kalman filter structure.
 * @param measurement The new raw measurement from the sensor.
 * @return The new filtered value.
 */
float kalman_update(kalman_filter_t *kf, float measurement);