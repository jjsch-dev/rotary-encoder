/*
 * Copyright (c) 2021 Juan Schiavoni
 *
 * This program provides an example using the esp32-c3 rotary encoder component.
 * There are three ways to process events: the first is to use an event 
 * queue, the second is to use a callback function, and the third is to 
 * polling the state.
 *
 * rotary-encoder is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * rotary-encoder is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with rotary-encoder.  If not, see <https://www.gnu.org/licenses/>.
 */
#include <stdbool.h>

#include "esp_system.h"
#include "esp_log.h"

#include "rotary_encoder.h"

/**
 *  NOTE: This component is part of the espressif project esp-rainmaker.
 *        https://github.com/espressif/esp-rainmaker
 * 
 *        The led pin is configured with the macro: CONFIG_WS2812_LED_GPIO
 *        please use idf.py menuconfig. 
 */
#ifdef CONFIG_IDF_TARGET_ESP32C3
    #include <ws2812_led.h>
    #define DEFAULT_SATURATION  100
    #define DEFAULT_BRIGHTNESS  50
#endif

#define TAG "app"

#if CONFIG_PUSH_BUTTON
static void button_callback(void* arg)
{
    rotenc_handle_t * handle = (rotenc_handle_t*) arg;
    ESP_LOGI(TAG, "Reset rotary encoder");
    ESP_ERROR_CHECK(rotenc_reset(handle));

#ifdef CONFIG_IDF_TARGET_ESP32C3
    ws2812_led_clear();
#endif
}
#endif

static void event_callback(rotenc_event_t event)
{
    ESP_LOGI(TAG, "Event: position %d, direction %s", event.position,
                  event.direction ? (event.direction == ROTENC_CW ? "CW" : "CCW") : "NOT_SET");

#ifdef CONFIG_IDF_TARGET_ESP32C3
    uint16_t g_hue = (uint16_t) (event.position * 10);
    ws2812_led_set_hsv(g_hue, DEFAULT_SATURATION, DEFAULT_BRIGHTNESS);
#endif
}

void app_main()
{
    // Verify that the GPIO ISR service is installed, before initializing the driver.
    ESP_ERROR_CHECK(gpio_install_isr_service(0));

#ifdef CONFIG_IDF_TARGET_ESP32C3
    esp_err_t err = ws2812_led_init();
    if  (err == ESP_OK) {
        ws2812_led_clear();
    }
    ESP_LOGI(TAG, "ws2812_led_init: %d\n", err);
#endif

    // Initialize the handle instance of the rotary device, 
    // by default it uses 1 mS for the debounce time.
    rotenc_handle_t handle = { 0 };
    ESP_ERROR_CHECK(rotenc_init(&handle, 
                                CONFIG_ROT_ENC_CLK_GPIO, 
                                CONFIG_ROT_ENC_DTA_GPIO, 
                                CONFIG_ROT_ENC_DEBOUNCE));
#if CONFIG_FLIP_DIRECTION
    ESP_ERROR_CHECK(rotenc_flip_direction(&handle));
#endif

#if CONFIG_PUSH_BUTTON
    ESP_ERROR_CHECK(rotenc_init_button(&handle, 
                                       CONFIG_ROT_ENC_BUTTON_GPIO, 
                                       CONFIG_ROT_ENC_BUTTON_DEBOUNCE, 
                                       button_callback));
#endif

#if CONFIG_REPORT_MODE_QUEUE
    ESP_LOGI(TAG, "Report mode by freertos queue");
    ESP_ERROR_CHECK(rotenc_set_event_queue(&handle, 1000));
#elif CONFIG_REPORT_MODE_CALLBACK
    ESP_LOGI(TAG, "Report mode by function callback");
    ESP_ERROR_CHECK(rotenc_set_event_callback(&handle, event_callback));
#elif CONFIG_REPORT_MODE_POLLING
    ESP_LOGI(TAG, "Report mode by polling");
#endif

    while (1) {
#if CONFIG_REPORT_MODE_QUEUE
        rotenc_event_t event = { 0 };
        if (rotenc_wait_event(&handle, &event) == ESP_OK) {
            event_callback(event);
        }
#elif CONFIG_REPORT_MODE_CALLBACK
        vTaskDelay(1000 / portTICK_PERIOD_MS);
#elif CONFIG_REPORT_MODE_POLLING
        vTaskDelay(100 / portTICK_PERIOD_MS);
        rotenc_event_t event = { 0 };
        ESP_ERROR_CHECK(rotenc_polling(&handle, &event));
        event_callback(event);
#endif
    }
}

