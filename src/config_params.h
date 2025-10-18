#ifndef CONFIG_PARAMS_H
#define CONFIG_PARAMS_H

#include <stdint.h>

/**
 * @brief Structure to hold all runtime configuration parameters.
 */
typedef struct {
    float battery_voltage_divider_ratio;
    uint32_t inactivity_timeout_ms;
    uint64_t sleep_duration_ms;
    uint32_t log_interval_ms; 
} config_params_t;

/**
 * @brief Initializes the configuration parameters by loading them from NVS.
 *
 * This function should be called once at startup after the config-manager
 * has potentially loaded new values from the SD card.
 */
void config_params_init(void);

/**
 * @brief Gets a pointer to the global configuration parameters structure.
 *
 * @return A const pointer to the config_params_t structure.
 */
const config_params_t* config_params_get(void);


#endif // CONFIG_PARAMS_H