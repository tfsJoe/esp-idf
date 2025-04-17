#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/i2s_std.h"

static const char *TAG = "NetMic";

#define I2S_WS   18  // LRCLK
#define I2S_SCK  19  // BCLK
#define I2S_SD   21  // DIN 
#define I2S_OUT  20  // Speaker output (not attached yet)

#define READ_LEN 1024
#define SAMPLE_RATE 16000

static i2s_chan_handle_t rx_chan = NULL;
static i2s_chan_handle_t tx_chan = NULL;

void configure_gpio_pins() {
    gpio_set_direction(I2S_WS, GPIO_MODE_OUTPUT);
    gpio_set_level(I2S_WS, 0);

    gpio_set_direction(I2S_SCK, GPIO_MODE_OUTPUT);
    gpio_set_level(I2S_SCK, 0);
    
    gpio_set_direction(I2S_OUT, GPIO_MODE_OUTPUT);
    gpio_set_level(I2S_OUT, 0);
    
    gpio_set_direction(I2S_SD, GPIO_MODE_INPUT);
    gpio_pullup_en(I2S_SD);
}

static void init_i2s(void) {
    esp_err_t err;

    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true;

    err = i2s_new_channel(&chan_cfg, &tx_chan, &rx_chan);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2S channel: %s", esp_err_to_name(err));
        return;
    }
    bool handle_fail = false;
    if (!rx_chan) {
        ESP_LOGE(TAG, "Channel initialization failed: NULL RX handle");
        handle_fail = true;
    }
    if (!tx_chan) {
        ESP_LOGE(TAG, "Channel initialization failed: NULL TX handle");
        handle_fail = true;
    }
    if (handle_fail) {
        return;
    }

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = {
            .data_bit_width = I2S_DATA_BIT_WIDTH_24BIT,
            .slot_bit_width = I2S_SLOT_BIT_WIDTH_32BIT,
            .slot_mode = I2S_SLOT_MODE_MONO,
            .slot_mask = I2S_STD_SLOT_LEFT,
            .ws_width = 16,
            .ws_pol = false,
            .bit_shift = false,
            .left_align = false,
            .big_endian = false,
        },
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_SCK,
            .ws = I2S_WS,
            .dout = I2S_OUT,
            .din = I2S_SD,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };

    err = i2s_channel_init_std_mode(rx_chan, &std_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init I2S standard mode: %s", esp_err_to_name(err));
        return;
    }

    err = i2s_channel_enable(rx_chan);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable I2S channel: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "I2S initialized successfully");
}

void read_i2s_data() {
    uint8_t *i2s_read_buff = (uint8_t *)calloc(READ_LEN, sizeof(char));
    size_t bytes_read;

    if (!i2s_read_buff) {
        ESP_LOGE("I2S", "Failed to allocate read buffer");
        return;
    }

    ESP_LOGI("I2S", "Reading data...");
    esp_err_t ret = i2s_channel_read(rx_chan, i2s_read_buff, READ_LEN, &bytes_read, pdMS_TO_TICKS(1000));
    if (ret == ESP_OK && bytes_read > 0) {
        ESP_LOGI("I2S", "Read %d bytes", (int)bytes_read);

        // Log first few sample values (assuming 16-bit samples)
        for (int i = 0; i < 10 && i < bytes_read / 2; i++) {
            int16_t sample = ((int16_t *)i2s_read_buff)[i];
            ESP_LOGI("I2S", "Sample %d: %d", i, sample);
        }
    } else {
        ESP_LOGW("I2S", "i2s_channel_read failed or timed out: %s", esp_err_to_name(ret));
    }

    free(i2s_read_buff);
}

void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(3000));
    ESP_LOGI(TAG, "Starting NetMic");
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    ESP_LOGI(TAG, "\tConfiguring GPIO pins");
    configure_gpio_pins();
    
    ESP_LOGI(TAG, "\tInitializing I2S");
    init_i2s();
    
    while (1) {
        read_i2s_data();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
