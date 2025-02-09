#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "led_strip.h"
#include "driver/rmt_tx.h"
#include "esp_err.h"

#define LED_STRIP_GPIO 8  // GPIO8 is the onboard LED on ESP32-C6
#define LED_STRIP_RESOLUTION_HZ 10000000  // 10MHz resolution
#define NUM_LEDS 1  // Only 1 LED onboard

#define EXT_LED_GPIO 5

led_strip_handle_t led_strip;

// Function to initialize the onboard LED
void setup_led()
{
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_STRIP_GPIO,
        .max_leds = NUM_LEDS,  
    };

    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = LED_STRIP_RESOLUTION_HZ,
    };

    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
}

// Function to set LED color
void set_led_color(uint8_t red, uint8_t green, uint8_t blue)
{
    led_strip_set_pixel(led_strip, 0, red, green, blue);
    led_strip_refresh(led_strip);
}


// Main function to cycle colors
void app_main()
{
    setup_led();

    while (1)
    {
        // Cycle Red → Green → Blue with fading effect
        for (int i = 0; i < 256; i += 10) {
            set_led_color(i, 0, 0);  // Red increasing
            vTaskDelay(50 / portTICK_PERIOD_MS);
        }
        for (int i = 0; i < 256; i += 10) {
            set_led_color(0, i, 0);  // Green increasing
            vTaskDelay(50 / portTICK_PERIOD_MS);
        }
        for (int i = 0; i < 256; i += 10) {
            set_led_color(0, 0, i);  // Blue increasing
            vTaskDelay(50 / portTICK_PERIOD_MS);
        }
    }
}
