#include "rotary_encoder.h"
#include "esp_log.h"
#include "freertos/task.h"
#include "esp_sleep.h"
static const char *TAG = "ROTARY_ENCODER";

rotaryencoder_config_t g_encoder_cfg;
static QueueHandle_t gpio_evt_queue = NULL;
static volatile encoder_direction_t g_encoder_direction = ENC_DIR_NONE;
static volatile encoder_button_state_t g_encoder_button_state = ENC_BTN_IDLE;

// For state-machine based decoding
static volatile uint8_t g_encoder_state = 0;

// Table for full-step quadrature decoding, moved to file scope.
// Index is (old_state << 2) | new_state. Values: 0=no change, 1=CW, -1=CCW.
// static makes it private to this file. const places it in flash.
#define LONG_PRESS_DURATION_MS 2000
static const int8_t KNOB_STATES[] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

/**
 * @brief GPIO ISR handler that sends the triggered GPIO number to a queue.
 *
 * This function is marked with IRAM_ATTR to be placed in IRAM for faster execution.
 * @param arg The GPIO number that triggered the interrupt.
 */
void IRAM_ATTR gpio_isr_handler(void *arg)
{
    // This is an optional but recommended FreeRTOS feature.
    // It allows for an immediate context switch if the message unblocks a higher-priority task.
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    uint32_t gpio_num = (uint32_t)arg;
    // Send the GPIO number to the queue, and check if it woke a higher priority task.
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, &xHigherPriorityTaskWoken);
}

/**
 * @brief Initializes the rotary encoder GPIOs and ISR.
 *
 * Configures the GPIO pins for the encoder and its button, and sets up the
 * interrupt service routine to handle rotation and press events.
 * @param cfg Pointer to the rotary encoder configuration struct.
 */
void rotaryencoder_init(const rotaryencoder_config_t *cfg)
{
    g_encoder_cfg = *cfg; // Copy config

    // Configure encoder pins A and B
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.pin_bit_mask = (1ULL << g_encoder_cfg.pin_a) | (1ULL << g_encoder_cfg.pin_b);
    io_conf.mode = GPIO_MODE_INPUT; 
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    // Configure button pin
    io_conf.intr_type = GPIO_INTR_ANYEDGE; // Trigger on both press and release
    io_conf.pin_bit_mask = (1ULL << g_encoder_cfg.button_pin);
    // mode and pull_up_en are already set from before
    gpio_config(&io_conf);

    gpio_evt_queue = xQueueCreate(64, sizeof(uint32_t));
    gpio_install_isr_service(0);
    gpio_isr_handler_add(g_encoder_cfg.pin_a, gpio_isr_handler, (void *)g_encoder_cfg.pin_a);
    gpio_isr_handler_add(g_encoder_cfg.pin_b, gpio_isr_handler, (void *)g_encoder_cfg.pin_b);
    gpio_isr_handler_add(g_encoder_cfg.button_pin, gpio_isr_handler, (void *)g_encoder_cfg.button_pin);
}

/**
 * @brief Configures the rotary encoder's button pin as a wake-up source.
 *
 * This is called before entering light sleep.
 */
void rotaryencoder_enable_wakeup_source(void)
{
    ESP_LOGI(TAG, "Enabling deep sleep wakeup on GPIO %d", g_encoder_cfg.button_pin);
    // Use ext0 wakeup for deep sleep. It can only use one pin.
    // The pin must be an RTC GPIO. Wake on low level (0).
    esp_err_t err = esp_sleep_enable_ext0_wakeup(g_encoder_cfg.button_pin, 0);
    if (err != ESP_OK) ESP_LOGE(TAG, "Failed to enable ext0 wakeup for button pin: %s", esp_err_to_name(err));
}

/**
 * @brief The main task for processing rotary encoder events.
 *
 * This task waits for events from the ISR queue, debounces them, decodes rotation,
 * and calls the appropriate callbacks.
 * @param arg Unused.
 */
static void rotary_encoder_task(void *arg)
{
    uint32_t gpio_num;
    TickType_t last_rotation_time = 0;
    TickType_t button_down_time = 0; // Time when button was pressed
    bool long_press_fired = false;   // Flag to prevent short press after a long press
    int32_t encoder_position = 0; // Track absolute position

    while (1)
    {
        // Wait for a GPIO event or timeout after 200ms to check for long press
        if (xQueueReceive(gpio_evt_queue, &gpio_num, pdMS_TO_TICKS(200)))
        {
            if (gpio_num == g_encoder_cfg.button_pin)
            {
                if (gpio_get_level(g_encoder_cfg.button_pin) == 0)
                {
                    // --- Button is PRESSED (falling edge) ---
                    if (button_down_time == 0) { // Only register if not already pressed
                        // Debounce the initial press
                        vTaskDelay(pdMS_TO_TICKS(g_encoder_cfg.button_debounce_ms));
                        if (gpio_get_level(g_encoder_cfg.button_pin) == 0) {
                            button_down_time = xTaskGetTickCount();
                            long_press_fired = false; // Reset flag on new press
                        }
                    }
                }
                else
                {
                    // --- Button is RELEASED (rising edge) ---
                    if (button_down_time > 0) { // Only process release if a press was registered
                        // Only fire short press if a long press hasn't already been fired
                        if (!long_press_fired)
                        {
                            if (g_encoder_cfg.on_button_press) g_encoder_cfg.on_button_press();
                        }
                    }
                    button_down_time = 0; // Reset press time
                }
            }
            else if (gpio_num == g_encoder_cfg.pin_a || gpio_num == g_encoder_cfg.pin_b)
            {
                TickType_t current_time = xTaskGetTickCount();
                if ((current_time - last_rotation_time) * portTICK_PERIOD_MS > 2) // 2ms debounce
                {
                    // Read the new state of the encoder pins
                    uint8_t new_state = (gpio_get_level(g_encoder_cfg.pin_a) << 1) | gpio_get_level(g_encoder_cfg.pin_b);
                    
                    // Use the state table to find the direction
                    int8_t direction = KNOB_STATES[(g_encoder_state << 2) | new_state];

                    if (direction != 0) {
                        last_rotation_time = current_time; // Update time on valid rotation
                        if (direction == 1) { // Clockwise
                            encoder_position++;
                            g_encoder_direction = ENC_DIR_CW;
                            if (g_encoder_cfg.on_rotate_cw)
                            {
                                // Send event only on even positions (full step)
                                if ((encoder_position % 2) == 0) g_encoder_cfg.on_rotate_cw();
                            }
                        } else { // Counter-Clockwise (direction == -1)
                            encoder_position--;
                            g_encoder_direction = ENC_DIR_CCW;
                            if (g_encoder_cfg.on_rotate_ccw)
                            {
                                // Send event only on even positions (full step)
                                if ((encoder_position % 2) == 0) g_encoder_cfg.on_rotate_ccw();
                            }
                        }
                    }
                    // Update the state for the next transition
                    g_encoder_state = new_state;
                }
            }
        }
        else
        {
            // --- Queue receive timed out, check for long press ---
            if (button_down_time > 0 && !long_press_fired)
            {
                TickType_t current_time = xTaskGetTickCount();
                if ((current_time - button_down_time) * portTICK_PERIOD_MS >= LONG_PRESS_DURATION_MS)
                {
                    // Before firing, double-check if the button is still physically pressed.
                    // This prevents a spurious long press if the release event was dropped.
                    if (gpio_get_level(g_encoder_cfg.button_pin) == 0) {
                        if (g_encoder_cfg.on_button_long_press) g_encoder_cfg.on_button_long_press();
                        long_press_fired = true; // Mark that long press has been handled
                    } else {
                        // The button is up, but we missed the release event. Reset state.
                        //ESP_LOGD(TAG, "Missed button release event, resetting long press state.");
                        button_down_time = 0;
                    }
                }
            }
        }
    }
}

/**
 * @brief Starts the rotary encoder processing task.
 */
void rotaryencoder_start_task(void)
{
    xTaskCreate(&rotary_encoder_task, "rotary_encoder_task", 2048, NULL, 7, NULL);
}

/**
 * @brief Gets the last detected rotation direction and resets it.
 *
 * @return encoder_direction_t The direction of the last rotation (CW, CCW, or NONE).
 */
encoder_direction_t rotaryencoder_get_direction(void)
{
    encoder_direction_t dir = g_encoder_direction;
    g_encoder_direction = ENC_DIR_NONE; // Reset after reading
    return dir;
}

/**
 * @brief Gets the last detected button state and resets it.
 *
 * @return encoder_button_state_t The state of the button (PRESSED or IDLE).
 */
encoder_button_state_t rotaryencoder_get_button_state(void)
{
    encoder_button_state_t state = g_encoder_button_state;
    g_encoder_button_state = ENC_BTN_IDLE; // Reset after reading
    return state;
}

/**
 * @brief Gets the handle of the internal GPIO event queue.
 *
 * @return QueueHandle_t The queue handle.
 */
QueueHandle_t rotaryencoder_get_queue_handle(void)
{
    return gpio_evt_queue;
}
