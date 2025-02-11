/*
 * SPDX-FileCopyrightText: 2021-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: LicenseRef-Included
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Espressif Systems
 *    integrated circuit in a product or a software update for such product,
 *    must reproduce the above copyright notice, this list of conditions and
 *    the following disclaimer in the documentation and/or other materials
 *    provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * 4. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "esp_log.h"
#include "led_strip.h"
#include "light_driver.h"
#include <math.h>

static led_strip_handle_t s_led_strip;
static uint8_t s_red = 255, s_green = 255, s_blue = 255;

void light_driver_set_power(bool power)
{
    ESP_ERROR_CHECK(led_strip_set_pixel(s_led_strip, 0, s_red * power, s_green * power, s_blue * power));
    ESP_ERROR_CHECK(led_strip_refresh(s_led_strip));
}

void light_driver_set_color(uint32_t r, uint32_t g, uint32_t b)
{
  s_red = r;
  s_green = g;
  s_blue = b;
  ESP_ERROR_CHECK(led_strip_set_pixel(s_led_strip, 0, r, g, b));
  ESP_ERROR_CHECK(led_strip_refresh(s_led_strip));
}

void light_driver_set_hue_and_saturation(uint8_t hue, uint8_t sat)
{
  // Placeholder: set greenish for now.
  s_red = sat;
  s_green = 255;
  s_blue = sat;
  ESP_ERROR_CHECK(led_strip_set_pixel(s_led_strip, 0, s_red, s_green, s_blue));
  ESP_ERROR_CHECK(led_strip_refresh(s_led_strip));
}

void light_driver_set_brightness(uint8_t brightness)
{
  // Placeholder: set brightness but only for white.
  s_red = brightness;
  s_green = brightness;
  s_blue = brightness;
  ESP_ERROR_CHECK(led_strip_set_pixel(s_led_strip, 0, s_red, s_green, s_blue));
  ESP_ERROR_CHECK(led_strip_refresh(s_led_strip));
}

#include <stdio.h>

// Convert CIE 1931 (X, Y; 16-bit unsigned) to RGB (8-bit)
void cie1931_to_rgb(uint8_t rgb[3], uint16_t X, uint16_t Y) {
    // Scale X and Y from 0-65535 to 0.0-1.0
    float X_float = (float)X / 65535.0f;
    float Y_float = (float)Y / 65535.0f;

    // CIE 1931 to XYZ conversion matrix
    float Z = 1.0f - X_float - Y_float;

    // Convert from XYZ to RGB (using the sRGB color space matrix)
    float R_linear = X_float * 3.2406f - Y_float * 1.5372f - Z * 0.4986f;
    float G_linear = -X_float * 0.9689f + Y_float * 1.8758f + Z * 0.0415f;
    float B_linear = X_float * 0.0556f - Y_float * 0.2040f + Z * 1.0572f;

    // Apply gamma correction (sRGB)
    float R = (R_linear > 0.0031308f) ? (1.055f * pow(R_linear, 1.0f / 2.4f) - 0.055f) : (12.92f * R_linear);
    float G = (G_linear > 0.0031308f) ? (1.055f * pow(G_linear, 1.0f / 2.4f) - 0.055f) : (12.92f * G_linear);
    float B = (B_linear > 0.0031308f) ? (1.055f * pow(B_linear, 1.0f / 2.4f) - 0.055f) : (12.92f * B_linear);

    // Clamp RGB values to [0, 1]
    R = (R > 1.0f) ? 1.0f : (R < 0.0f) ? 0.0f : R;
    G = (G > 1.0f) ? 1.0f : (G < 0.0f) ? 0.0f : G;
    B = (B > 1.0f) ? 1.0f : (B < 0.0f) ? 0.0f : B;

    // Convert to 8-bit values and store in the RGB array
    rgb[0] = (uint8_t)(R * 255);
    rgb[1] = (uint8_t)(G * 255);
    rgb[2] = (uint8_t)(B * 255);
}


void light_driver_init(bool power)
{
    led_strip_config_t led_strip_conf = {
        .max_leds = CONFIG_EXAMPLE_STRIP_LED_NUMBER,
        .strip_gpio_num = CONFIG_EXAMPLE_STRIP_LED_GPIO,
    };
    led_strip_rmt_config_t rmt_conf = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&led_strip_conf, &rmt_conf, &s_led_strip));
    light_driver_set_power(power);
}
