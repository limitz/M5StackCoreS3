/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/rmt_tx.h"
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"
#include "led_strip_encoder.h"

#define RMT_LED_STRIP_RESOLUTION_HZ 10000000 // 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)
#define RMT_LED_STRIP_GPIO_NUM      27
#define EXAMPLE_LED_NUMBERS         48
#define EXAMPLE_CHASE_SPEED_MS      10

#define PCNT_GPIO_NUM 13

static const char *TAG = "example";
static uint8_t led_strip_pixels[EXAMPLE_LED_NUMBERS * 3];

void app_main(void)
{

    ESP_LOGI(TAG, "Create RMT TX channel");
    rmt_channel_handle_t led_chan = NULL;
    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT, // select source clock
        .gpio_num = RMT_LED_STRIP_GPIO_NUM,
        .mem_block_symbols = 64, // increase the block size can make the LED less flickering
        .resolution_hz = RMT_LED_STRIP_RESOLUTION_HZ,
        .trans_queue_depth = 4, // set the number of transactions that can be pending in the background
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &led_chan));

    ESP_LOGI(TAG, "Install led strip encoder");
    rmt_encoder_handle_t led_encoder = NULL;
    led_strip_encoder_config_t encoder_config = {
        .resolution = RMT_LED_STRIP_RESOLUTION_HZ,
    };
    ESP_ERROR_CHECK(rmt_new_led_strip_encoder(&encoder_config, &led_encoder));

    ESP_LOGI(TAG, "Enable RMT TX channel");
    ESP_ERROR_CHECK(rmt_enable(led_chan));

    ESP_LOGI(TAG, "Start LED rainbow chase");
    rmt_transmit_config_t tx_config = {
        .loop_count = 0, // no transfer loop
    };
    
    pcnt_unit_config_t unit_config = {
        .high_limit = 1000,
        .low_limit = -1000,
    };
    pcnt_unit_handle_t pcnt_unit = NULL;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));

    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));

    pcnt_chan_config_t chan_config = {
        .edge_gpio_num = PCNT_GPIO_NUM,
    };
    pcnt_channel_handle_t pcnt_chan = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_config, &pcnt_chan));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD));


    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));

    int fans = 3;
    int blades = 3;
    int offset = 0;
    int speed = 8;
    int num_pulses = 0;
    int time = 0;

    uint8_t cos_values[128*8];
    for (int i=0; i<sizeof(cos_values); i++) 
    {
        float v = powf(cos(i * M_PI / sizeof(cos_values)), 4);
        cos_values[i] = (uint8_t)(v * 255);
    }

    while (1) 
    {
        for (int fan = 0; fan < fans; fan++)
        {
            for (int led = 0; led < 16; led++) 
            {        
                bool is_ring = led >= 4;
                int ring_index = led - 4;
                int j = fan * 16 + led;
                if (is_ring)
                {
                    int cos_index = (offset + (ring_index * sizeof(cos_values) * blades) / 12); 
                    uint8_t cos_value = cos_values[cos_index & (sizeof(cos_values)-1)];
                    led_strip_pixels[j * 3 + 0] = cos_value >> 3;
                    led_strip_pixels[j * 3 + 1] = 0;
                    led_strip_pixels[j * 3 + 2] = cos_value;
            
                }
                else
                {
                    int cos_index = (offset*3 + ((led+0) * sizeof(cos_values) * blades) / 4); 
                    uint8_t cos_value = cos_values[cos_index & (sizeof(cos_values)-1)];
                    led_strip_pixels[j * 3 + 0] = 32;
                    led_strip_pixels[j * 3 + 1] = 255;
                    led_strip_pixels[j * 3 + 2] = 0;
                }
            }
        }
        // Flush RGB values to LEDs
        ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
        ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
        vTaskDelay(pdMS_TO_TICKS(EXAMPLE_CHASE_SPEED_MS));
    
        time += 1;
        if (time == 100)
        {
            time = 0;
            ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit, &num_pulses));
            ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
            ESP_LOGI(TAG, "Pulses: %d", num_pulses);
        }
        offset += speed;
    }
}
