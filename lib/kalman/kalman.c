#include "kalman.h"
#include <math.h>

/**
 * @brief Initializes a Kalman filter with given parameters.
 *
 * @param kf Pointer to the Kalman filter structure to initialize.
 * @param q Process noise covariance. A smaller value indicates a more stable process.
 * @param r Measurement noise covariance. A larger value indicates less trust in the measurement.l
 */
void kalman_init(kalman_filter_t *kf, double q, double r, double initial_value) {
    kf->q = q;
    kf->r = r;
    // Initialize the estimation error covariance to be equal to the measurement noise.
    // This is a common and effective practice, as it assumes the initial state's
    // uncertainty is on the same scale as the sensor's measurement uncertainty.
    kf->p = r;
    kf->x = initial_value;
    kf->k = 0.0;
}

/**
 * @brief Updates the Kalman filter with a new measurement.
 *
 * @param kf Pointer to the Kalman filter structure.
 * @param measurement The new raw measurement from the sensor.
 * @return The new filtered value.
 */
double kalman_update(kalman_filter_t *kf, double measurement) {
    // Prediction update (now using double precision)
    kf->p = kf->p + kf->q;

    // Measurement update
    kf->k = kf->p / (kf->p + kf->r);
    kf->x = kf->x + kf->k * (measurement - kf->x);
    kf->p = (1.0 - kf->k) * kf->p;

    return kf->x;
}