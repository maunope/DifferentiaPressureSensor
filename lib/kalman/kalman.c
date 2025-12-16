#include "kalman.h"
#include <math.h>

/**
 * @brief Initializes a Kalman filter with given parameters.
 *
 * @param kf Pointer to the Kalman filter structure to initialize.
 * @param q Process noise covariance. A smaller value indicates a more stable process.
 * @param r Measurement noise covariance. A larger value indicates less trust in the measurement.
 * @param initial_value The initial value to seed the filter with.
 */
void kalman_init(kalman_filter_t *kf, float q, float r, float initial_value) {
    kf->q = q;
    kf->r = r;
    kf->p = 0.0f;
    kf->x = initial_value;
    kf->k = 0.0f;
}

/**
 * @brief Updates the Kalman filter with a new measurement.
 *
 * @param kf Pointer to the Kalman filter structure.
 * @param measurement The new raw measurement from the sensor.
 * @return The new filtered value.
 */
float kalman_update(kalman_filter_t *kf, float measurement) {
    // Prediction update
    kf->p = kf->p + kf->q;

    // Measurement update
    kf->k = kf->p / (kf->p + kf->r);
    kf->x = kf->x + kf->k * (measurement - kf->x);
    kf->p = (1.0f - kf->k) * kf->p;

    return kf->x;
}