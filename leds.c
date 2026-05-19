/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#include <stdbool.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_strip.h"
#include "esp_log.h"
#include "esp_err.h"

// GPIO assignment
#define LED_STRIP_BLINK_GPIO  12
// Numbers of the LED in the strip
#define LED_STRIP_LED_NUMBERS 7

static const char *TAG = "LEDs";
static led_strip_handle_t led_strip;

static bool is_led_strip_initialized = false;

typedef struct {
    uint8_t r_color;
    uint8_t g_color;
    uint8_t b_color;
} led_rgb;

led_rgb led_array[LED_STRIP_LED_NUMBERS];
void led_task(void*);
    
void configure_led(void)
{
    // LED strip general initialization, according to your led board design
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_STRIP_BLINK_GPIO,   // The GPIO that connected to the LED strip's data line
        .max_leds = LED_STRIP_LED_NUMBERS,        // The number of LEDs in the strip,
        .led_pixel_format = LED_PIXEL_FORMAT_GRB, // Pixel format of your LED strip
        .led_model = LED_MODEL_WS2812,            // LED strip model
        .flags.invert_out = true,                // whether to invert the output signal
    };

    // LED strip backend configuration: SPI
    led_strip_spi_config_t spi_config = {
        .clk_src = SPI_CLK_SRC_DEFAULT, // different clock source can lead to different power consumption
        .flags.with_dma = true,         // Using DMA can improve performance and help drive more LEDs
        .spi_bus = SPI2_HOST,           // SPI bus ID
    };

    // LED Strip object handle
    ESP_ERROR_CHECK(led_strip_new_spi_device(&strip_config, &spi_config, &led_strip));
    ESP_LOGI(TAG, "Created LED strip object with SPI backend");
}

void led_init()
{
    if (is_led_strip_initialized)
	return;
    
    configure_led();

    for(int i=0; i<LED_STRIP_LED_NUMBERS; i++){
	ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, 255, 0, 255));	
    }
    ESP_ERROR_CHECK(led_strip_refresh(led_strip));
    vTaskDelay(pdMS_TO_TICKS(500));
    ESP_ERROR_CHECK(led_strip_clear(led_strip));
    vTaskDelay(pdMS_TO_TICKS(1));

    is_led_strip_initialized = true;
    
    xTaskCreatePinnedToCore(led_task, "led_task", 8192, NULL, 5, NULL, 0);
}

void led_task(void *parameters)
{
    while (true)
    {
	for(int i=0; i<LED_STRIP_LED_NUMBERS; i++){
	    ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, led_array[i].r_color, led_array[i].g_color, led_array[i].b_color));
	}
	ESP_ERROR_CHECK(led_strip_refresh(led_strip));
	vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void led_set_on(int local_led_no, int local_r, int local_g, int local_b)
{
    led_init();
    
    if (local_r > 255 || local_r < 0 || local_g > 255 || local_g < 0 || local_b > 255 || local_b < 0) {
        ESP_LOGE(TAG, "RGB out of bound !!!");
        return;
    }
    if(local_led_no < 0 || local_led_no >= LED_STRIP_LED_NUMBERS){
	ESP_LOGE(TAG, "LED out of bound !!!");
    }
    
    led_array[local_led_no].r_color = local_r;
    led_array[local_led_no].g_color = local_g;
    led_array[local_led_no].b_color = local_b;
    
    //    ESP_LOGE(TAG,"LED_ON");
}

void led_set_off(int led_no)
{
    led_init();
    
    if(led_no < 0 || led_no >= LED_STRIP_LED_NUMBERS){
	ESP_LOGE(TAG, "LED out of bound !!!");
    }
    
    led_array[led_no].r_color = 0;
    led_array[led_no].g_color = 0;
    led_array[led_no].b_color = 0;
    //   ESP_LOGE(TAG,"LED_OFF");
}

void led_test()
{
    while(true)
    {
	for(int i=0;i<7;i++){
	    led_set_on(i, 255/2 , 255/3, 0/2);
	    vTaskDelay(pdMS_TO_TICKS(100));
	}
    
	for(int i=0;i<7;i++){
	    led_set_off(i);
	    vTaskDelay(pdMS_TO_TICKS(100));
	}
	vTaskDelay(pdMS_TO_TICKS(1));
    }
}
