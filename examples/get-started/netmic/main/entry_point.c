#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s_std.h"
#include "esp_log.h"

#define I2S_WS   1  // LRCLK
#define I2S_SCK  3  // BCLK
#define I2S_SD  14  // DIN (Data In)

void app_main(void)
{
    ESP_LOGI("MAIN", "Starting I2S clock test...");

    i2s_chan_handle_t rx_handle;
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_SLAVE);  // RX as slave

    ESP_LOGI("MAIN", "Creating I2S channel...");
    esp_err_t err = i2s_new_channel(&chan_cfg, &rx_handle, NULL);
    if (err != ESP_OK) {
        ESP_LOGE("MAIN", "Failed to create I2S channel, error: %s", esp_err_to_name(err));
        return;
    }

    i2s_std_config_t std_cfg = {
        .clk_cfg = {
            .sample_rate_hz = 16000,
            .clk_src = I2S_CLK_SRC_DEFAULT,
            .mclk_multiple = I2S_MCLK_MULTIPLE_256,
        },
        .slot_cfg = {
            .data_bit_width = I2S_DATA_BIT_WIDTH_16BIT,
            .slot_bit_width = I2S_SLOT_BIT_WIDTH_16BIT,
            .slot_mode = I2S_SLOT_MODE_MONO,
            .slot_mask = I2S_STD_SLOT_LEFT,
            .ws_width = 16,
        },
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_SCK,  // BCLK
            .ws = I2S_WS,
            .dout = I2S_GPIO_UNUSED,
            .din = I2S_SD,  // Data input (DIN)
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };

    ESP_LOGI("MAIN", "Initializing I2S in standard mode...");
    err = i2s_channel_init_std_mode(rx_handle, &std_cfg);
    if (err != ESP_OK) {
        ESP_LOGE("MAIN", "I2S initialization failed, error: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI("MAIN", "Enabling I2S channel...");
    err = i2s_channel_enable(rx_handle);
    if (err != ESP_OK) {
        ESP_LOGE("MAIN", "Failed to enable I2S channel, error: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI("MAIN", "I2S clock config complete. BCLK on GPIO %d, WS on GPIO %d, DIN on GPIO %d.", I2S_SCK, I2S_WS, I2S_SD);

    while (true) {
        ESP_LOGI("MAIN", "I2S test running...");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
