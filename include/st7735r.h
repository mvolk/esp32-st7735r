/*
 * MIT License
 *
 * Copyright (c) 2020 Michael Volk
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#ifndef ST7735R_H
#define ST7735R_H

#include "freertos/FreeRTOS.h"

// TODO: the labels here are just guesses
enum uint8_t {
  NONE = 0,
  ROTATED_LEFT = 1,
  INVERTED = 2,
  ROTATED_RIGHT = 3
} rotation_t;

enum st7735_variant_t {
  ST7735B,
  ST7735R
};

typedef struct {
  gpio_num_t cs;    // <-- Chip Select
  gpio_num_t dc;    // <-- Data/Command Select
  gpio_num_t mosi;  // <-- SPI MOSI (MISO not used)
  gpio_num_t sclk;  // <-- SPI Clock
  gpio_num_t rst;   // <-- Reset
  gpio_num_t blt;   // <-- Backlight
  uint16_t width;   // <-- Width in pixels
  uint16_t height;  // <-- Height in pixels
  rotation_t rotation; // <-- Rotation
  st7735_variant_t variant;
  uint8_t options;
} ST7735_Info;


#endif // ST7735R
