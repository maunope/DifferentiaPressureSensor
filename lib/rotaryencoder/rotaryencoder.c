#include "rotaryencoder.h"
#include "esp_log.h"
#include "freertos/task.h"
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
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.pin_bit_mask = (1ULL << g_encoder_cfg.button_pin);
    // mode and pull_up_en are already set from before
    gpio_config(&io_conf);

    gpio_evt_queue = xQueueCreate(16, sizeof(uint32_t));
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
    // --- Configure wake-up sources ---
    // Wake on button press (HIGH to LOW). This is the only reliable GPIO wakeup source
    // on non-RTC pins.
    esp_err_t err = gpio_wakeup_enable(g_encoder_cfg.button_pin, GPIO_INTR_LOW_LEVEL);
    if (err != ESP_OK) ESP_LOGE(TAG, "Failed to enable wakeup for button pin: %s", esp_err_to_name(err));
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
    TickType_t last_button_press_time = 0;
    TickType_t last_rotation_time = 0;
    int32_t encoder_position = 0; // Track absolute position

    while (1)
    {
        
         // DO NOT log  in this loop, unless it's for debug
         // ESP_LOGI is stack-heavy and can stack overflow this task when many events come in.
         // OR, figure some clever way to log in a more memory-efficient manner.
        if (xQueueReceive(gpio_evt_queue, &gpio_num, portMAX_DELAY) )
        {
            if (gpio_num == g_encoder_cfg.button_pin)
            {
                TickType_t current_time = xTaskGetTickCount();
                if ((current_time - last_button_press_time) * portTICK_PERIOD_MS > g_encoder_cfg.button_debounce_ms)
                {
                    last_button_press_time = current_time;
                    vTaskDelay(pdMS_TO_TICKS(20)); // Debounce delay

                    uint32_t press_duration_ms = 0;
                    while (gpio_get_level(g_encoder_cfg.button_pin) == 0)
                    {
                        vTaskDelay(pdMS_TO_TICKS(50));
                        press_duration_ms += 50;
                        if (press_duration_ms >= LONG_PRESS_DURATION_MS)
                        {
                            // --- Long Press Detected ---
                            if (g_encoder_cfg.on_button_long_press)
                            {
                                g_encoder_cfg.on_button_long_press();
                            }
                            // Wait for release before continuing to avoid re-triggering
                            while (gpio_get_level(g_encoder_cfg.button_pin) == 0) {
                                vTaskDelay(pdMS_TO_TICKS(10));
                            }
                            goto button_event_handled; // Skip short press logic
                        }
                    }
                    // --- Short Press Detected ---
                    // If the while loop finished before the long press duration, it was a short press.
                    if (g_encoder_cfg.on_button_press) g_encoder_cfg.on_button_press();
                }
            button_event_handled:;
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
    }
}

/**
 * @brief Starts the rotary encoder processing task.
 */
void rotaryencoder_start_task(void)
{
    xTaskCreate(&rotary_encoder_task, "rotary_encoder_task", 2048, NULL, 5, NULL);
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
