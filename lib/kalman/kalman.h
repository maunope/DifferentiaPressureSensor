#pragma once

#include <stdint.h>

/**
 * @brief Structure to hold the state of a single Kalman filter.
 *
 * This structure is designed to be statically allocated to avoid heap fragmentation.
 * It contains all the necessary state variables for the filter.
 */
typedef struct {
    double p; /**< Estimation error covariance */
    double x; /**< Current value */
    double k; /**< Kalman gain */
    double q; /**< Process noise covariance */
    double r; /**< Measurement noise covariance */
} kalman_filter_t;

/**
 * @brief Initializes a Kalman filter with given parameters.
 *
 * @param kf Pointer to the Kalman filter structure to initialize.
 * @param q Process noise covariance. A smaller value indicates a more stable process.
 * @param r Measurement noise covariance. A larger value indicates less trust in the measurement. This is also used as the initial estimation error covariance `p`.
 * @param initial_value The initial value to seed the filter with.
 */
void kalman_init(kalman_filter_t *kf, double q, double r, double initial_value);

/**
 * @brief Updates the Kalman filter with a new measurement.
 *
 * @param kf Pointer to the Kalman filter structure.
 * @param measurement The new raw measurement from the sensor.
 * @return The new filtered value.
 */
double kalman_update(kalman_filter_t *kf, double measurement);