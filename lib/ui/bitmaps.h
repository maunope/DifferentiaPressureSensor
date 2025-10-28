#pragma once

#include <stdint.h>

/**
 * @file bitmaps.h
 * @brief Contains definitions for 8x8 bitmaps used in the UI.
 *
 * Each bitmap is an array of 8 bytes, where each byte represents a row of 8 pixels.
 */

// An 8x8 "X" icon.
const uint8_t bitmap_icon_x[8] = {
    0b10000001, // B10000001
    0b01000010, // B01000010
    0b00100100, // B00100100
    0b00011000, // B00011000
    0b00011000, // B00011000
    0b00100100, // B00100100
    0b01000010, // B01000010
    0b10000001, // B10000001
};