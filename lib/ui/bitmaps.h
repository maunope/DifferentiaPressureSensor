#pragma once
#include <stdint.h>

//* built with buffers.xlsx

static const uint8_t usb_icon_l[8] = {
    0x00, // Row 0
    0x03, // Row 1
    0x04, // Row 2
    0x3C, // Row 3
    0x3C, / Row 4
    0x04, // Row 5
    0x03, // Row 6
    0x00, // Row 7
};

static const uint8_t usb_icon_r[8] = {
    0x00, // Row 0
    0xE0, // Row 1
    0x3C, // Row 2
    0xA4, // Row 3
    0xA4, // Row 4
    0x3C, // Row 5
    0xE0, // Row 6
    0x00, // Row 7
};

// Differential pressure Icon
const uint8_t diff_press_icon[8] = {
    0X40,
    0XEF,
    0X41,
    0X01,
    0X81,
    0X80,
    0XF7,
    0X00};

// Pressure Icon
const uint8_t press_icon[8] = {
    0X3C,
    0X42,
    0XA1,
    0X91,
    0X81,
    0X81,
    0X42,
    0X3C};

// Temperature Icon
const uint8_t temp_icon[8] = {
    0X23,
    0X50,
    0X51,
    0X50,
    0X53,
    0X88,
    0X89,
    0X70};

// Logging Pause Icon
const uint8_t logging_pause_icon[8] = {
    0X00,
    0X3C,
    0X22,
    0X22,
    0X3C,
    0X20,
    0X20,
    0X00};

// Logging Error Icon
const uint8_t logging_error_icon[8] = {
    0X00,
    0X24,
    0X24,
    0X24,
    0X24,
    0X00,
    0X24,
    0X00};

// Logging hf Icon
const uint8_t logging_hf_icon[8] = {
    0X00,
    0X44,
    0X22,
    0X11,
    0X11,
    0X22,
    0X44,
    0X00};
