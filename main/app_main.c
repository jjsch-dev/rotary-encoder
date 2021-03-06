/*
 * MIT License
 * 
 * Copyright (c) 2021 Juan Schiavoni
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

