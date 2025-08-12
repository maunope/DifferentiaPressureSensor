#include "rotaryencoder.h"
#include "esp_log.h"
#include "freertos/task.h"

static const char *TAG = "ROTARY_ENCODER";

static rotaryencoder_config_t encoder_cfg;
static QueueHandle_t gpio_evt_queue = NULL;
static volatile encoder_direction_t g_encoder_direction = ENC_DIR_NONE;
static volatile encoder_button_state_t g_encoder_button_state = ENC_BTN_IDLE;

static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t)arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

void rotaryencoder_init(const rotaryencoder_config_t *cfg)
{
    encoder_cfg = *cfg;

    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.pin_bit_mask = (1ULL << encoder_cfg.pin_a) | (1ULL << encoder_cfg.pin_b);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.pin_bit_mask = (1ULL << encoder_cfg.button_pin);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    gpio_install_isr_service(0);
    gpio_isr_handler_add(encoder_cfg.pin_a, gpio_isr_handler, (void *)encoder_cfg.pin_a);
    gpio_isr_handler_add(encoder_cfg.pin_b, gpio_isr_handler, (void *)encoder_cfg.pin_b);
    gpio_isr_handler_add(encoder_cfg.button_pin, gpio_isr_handler, (void *)encoder_cfg.button_pin);
}

//todo refactor taking the task part out of here
static void rotary_encoder_task(void *arg)
{
    uint32_t gpio_num;
    int last_state_A = gpio_get_level(encoder_cfg.pin_a);
    int last_state_B = gpio_get_level(encoder_cfg.pin_b);
    int last_button_state = gpio_get_level(encoder_cfg.button_pin);
    TickType_t last_button_press_time = 0;

    while (1)
    {
        if (xQueueReceive(gpio_evt_queue, &gpio_num, portMAX_DELAY))
        {
            vTaskDelay(pdMS_TO_TICKS(10));

            if (gpio_num == encoder_cfg.button_pin)
            {
                int current_button_state = gpio_get_level(encoder_cfg.button_pin);
                TickType_t current_time = xTaskGetTickCount();

                if (current_button_state != last_button_state &&
                    (current_time - last_button_press_time) * portTICK_PERIOD_MS > encoder_cfg.button_debounce_ms)
                {
                    if (current_button_state == 0)
                    {
                        ESP_LOGI(TAG, "Rotary Encoder Button Pressed!");
                        g_encoder_button_state = ENC_BTN_PRESSED;
                    }
                    else
                    {
                        g_encoder_button_state = ENC_BTN_IDLE;
                    }
                    last_button_state = current_button_state;
                    last_button_press_time = current_time;
                }
            }
            else if (gpio_num == encoder_cfg.pin_a || gpio_num == encoder_cfg.pin_b)
            {
                int current_state_A = gpio_get_level(encoder_cfg.pin_a);
                int current_state_B = gpio_get_level(encoder_cfg.pin_b);

                if (current_state_A != last_state_A)
                {
                    if (current_state_A == 1)
                    {
                        if (current_state_B == 0)
                        {
                            ESP_LOGI(TAG, "Rotary Encoder: Clockwise");
                            g_encoder_direction = ENC_DIR_CW;
                        }
                        else
                        {
                            ESP_LOGI(TAG, "Rotary Encoder: Counter-Clockwise");
                            g_encoder_direction = ENC_DIR_CCW;
                        }
                    }
                    else
                    {
                        if (current_state_B == 1)
                        {
                            ESP_LOGI(TAG, "Rotary Encoder: Clockwise");
                            g_encoder_direction = ENC_DIR_CW;
                        }
                        else
                        {
                            ESP_LOGI(TAG, "Rotary Encoder: Counter-Clockwise");
                            g_encoder_direction = ENC_DIR_CCW;
                        }
                    }
                }
                else if (current_state_B != last_state_B)
                {
                    if (current_state_B == 1)
                    {
                        if (current_state_A == 1)
                        {
                            ESP_LOGI(TAG, "Rotary Encoder: Clockwise");
                            g_encoder_direction = ENC_DIR_CW;
                        }
                        else
                        {
                            ESP_LOGI(TAG, "Rotary Encoder: Counter-Clockwise");
                            g_encoder_direction = ENC_DIR_CCW;
                        }
                    }
                    else
                    {
                        if (current_state_A == 0)
                        {
                            ESP_LOGI(TAG, "Rotary Encoder: Clockwise");
                            g_encoder_direction = ENC_DIR_CW;
                        }
                        else
                        {
                            ESP_LOGI(TAG, "Rotary Encoder: Counter-Clockwise");
                            g_encoder_direction = ENC_DIR_CCW;
                        }
                    }
                }

                last_state_A = current_state_A;
                last_state_B = current_state_B;
            }
        }
    }
}

void rotaryencoder_start_task(void)
{
    xTaskCreate(&rotary_encoder_task, "rotary_encoder_task", 2048, NULL, 10, NULL);
}

encoder_direction_t rotaryencoder_get_direction(void)
{
    return g_encoder_direction;
}

encoder_button_state_t rotaryencoder_get_button_state(void)
{
    return g_encoder_button_state;
}
