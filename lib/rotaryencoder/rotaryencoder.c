#include "rotaryencoder.h"
#include "esp_log.h"
#include "freertos/task.h"
#include "ui_render.h"

static const char *TAG = "ROTARY_ENCODER";

static rotaryencoder_config_t encoder_cfg;
static QueueHandle_t gpio_evt_queue = NULL;
static volatile encoder_direction_t g_encoder_direction = ENC_DIR_NONE;
static volatile encoder_button_state_t g_encoder_button_state = ENC_BTN_IDLE;

// For state-machine based decoding
static volatile uint8_t g_encoder_state = 0;

// Table for full-step quadrature decoding, moved to file scope.
// Index is (old_state << 2) | new_state. Values: 0=no change, 1=CW, -1=CCW.
// static makes it private to this file. const places it in flash.
static const int8_t KNOB_STATES[] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    // This is an optional but recommended FreeRTOS feature.
    // It allows for an immediate context switch if the message unblocks a higher-priority task.
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    uint32_t gpio_num = (uint32_t)arg;
    // Send the GPIO number to the queue, and check if it woke a higher priority task.
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, &xHigherPriorityTaskWoken);
}

void rotaryencoder_init(const rotaryencoder_config_t *cfg)
{
    encoder_cfg = *cfg;

    // Configure encoder pins A and B
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.pin_bit_mask = (1ULL << encoder_cfg.pin_a) | (1ULL << encoder_cfg.pin_b);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    // Configure button pin
    io_conf.intr_type = GPIO_INTR_NEGEDGE; // Trigger only on press (falling edge)
    io_conf.pin_bit_mask = (1ULL << encoder_cfg.button_pin);
    // mode and pull_up_en are already set from before
    gpio_config(&io_conf);

    gpio_evt_queue = xQueueCreate(16, sizeof(uint32_t));
    gpio_install_isr_service(0);
    gpio_isr_handler_add(encoder_cfg.pin_a, gpio_isr_handler, (void *)encoder_cfg.pin_a);
    gpio_isr_handler_add(encoder_cfg.pin_b, gpio_isr_handler, (void *)encoder_cfg.pin_b);
    gpio_isr_handler_add(encoder_cfg.button_pin, gpio_isr_handler, (void *)encoder_cfg.button_pin);
}

static void rotary_encoder_task(void *arg)
{
    uint32_t gpio_num;
    TickType_t last_button_press_time = 0;
    int32_t encoder_position = 0; // Track absolute position

    while (1)
    {
         // DO NOT log  in this loop, unless it's for debug
         // ESP_LOGI is stack-heavy and can stack overflow this task when many events come in.
         // OR, figure some clever way to log in a more memory-efficient manner.
        if (xQueueReceive(gpio_evt_queue, &gpio_num, portMAX_DELAY))
        {
            if (gpio_num == encoder_cfg.button_pin)
            {
                TickType_t current_time = xTaskGetTickCount();
                if ((current_time - last_button_press_time) * portTICK_PERIOD_MS > encoder_cfg.button_debounce_ms)
                {
                    // Add a small delay to ensure the button state is stable after the interrupt
                    vTaskDelay(pdMS_TO_TICKS(10));
                    if (gpio_get_level(encoder_cfg.button_pin) == 0)
                    {
                     
                      //  ESP_LOGI(TAG, "Rotary Encoder Button Pressed!");
                        g_encoder_button_state = ENC_BTN_PRESSED;
                        // Send button event to UI
                        uiRender_send_event(UI_EVENT_BTN, NULL, 0);
                    }
                    last_button_press_time = current_time;
                }
            }
            else if (gpio_num == encoder_cfg.pin_a || gpio_num == encoder_cfg.pin_b)
            {
                // Read the new state of the encoder pins
                uint8_t new_state = (gpio_get_level(encoder_cfg.pin_a) << 1) | gpio_get_level(encoder_cfg.pin_b);
                
                // Use the state table to find the direction
                int8_t direction = KNOB_STATES[(g_encoder_state << 2) | new_state];

                if (direction == 1) { // Clockwise
                    encoder_position++;
                   // ESP_LOGI(TAG, "Rotary Encoder: Clockwise, pos=%ld", encoder_position);
                    g_encoder_direction = ENC_DIR_CW;
                    // Send event only on even positions (0, 2, 4, ...)
                    if ((encoder_position % 2) == 0) {
                        uiRender_send_event(UI_EVENT_CW, NULL, 0);
                    }
                } else if (direction == -1) { // Counter-Clockwise
                    encoder_position--;
                  //  ESP_LOGI(TAG, "Rotary Encoder: Counter-Clockwise, pos=%ld", encoder_position);
                    g_encoder_direction = ENC_DIR_CCW;
                    // Send event only on even positions (0, -2, -4, ...)
                    if ((encoder_position % 2) == 0) {
                        uiRender_send_event(UI_EVENT_CCW, NULL, 0);
                    }
                }
                // else: no change or invalid state

                // Update the state for the next transition
                g_encoder_state = new_state;
              
            }
        }
    }
}

void rotaryencoder_start_task(void)
{
    xTaskCreate(&rotary_encoder_task, "rotary_encoder_task", 2048, NULL, 5, NULL);
}

encoder_direction_t rotaryencoder_get_direction(void)
{
    encoder_direction_t dir = g_encoder_direction;
    g_encoder_direction = ENC_DIR_NONE; // Reset after reading
    return dir;
}

encoder_button_state_t rotaryencoder_get_button_state(void)
{
    encoder_button_state_t state = g_encoder_button_state;
    g_encoder_button_state = ENC_BTN_IDLE; // Reset after reading
    return state;
}
