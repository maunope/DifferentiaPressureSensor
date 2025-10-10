#pragma once

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

typedef enum
{
    ENC_DIR_NONE,
    ENC_DIR_CW,
    ENC_DIR_CCW
} encoder_direction_t;

typedef enum
{
    ENC_BTN_IDLE,
    ENC_BTN_PRESSED
} encoder_button_state_t;

typedef struct
{
    gpio_num_t pin_a;
    gpio_num_t pin_b;
    gpio_num_t button_pin;
    uint32_t button_debounce_ms;
} rotaryencoder_config_t;

void rotaryencoder_init(const rotaryencoder_config_t *cfg);
void rotaryencoder_start_task(void);
encoder_direction_t rotaryencoder_get_direction(void);
encoder_button_state_t rotaryencoder_get_button_state(void);
void rotaryencoder_enable_wakeup_source(void);
#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif
void IRAM_ATTR gpio_isr_handler(void *arg);
QueueHandle_t rotaryencoder_get_queue_handle(void);