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

#define TAG "app"

#if CONFIG_PUSH_BUTTON
static void button_callback(void* arg)
{
    rotenc_handle_t * handle = (rotenc_handle_t*) arg;
    ESP_LOGI(TAG, "Reset rotary encoder");
    ESP_ERROR_CHECK(rotenc_reset(handle));
}
#endif

static void log_event(rotenc_event_t event)
{
    ESP_LOGI(TAG, "Event: position %d, direction %s", event.position,
                  event.direction ? (event.direction == ROTENC_CW ? "CW" : "CCW") : "NOT_SET");
}

void app_main()
{
    // Verify that the GPIO ISR service is installed, before initializing the driver.
    ESP_ERROR_CHECK(gpio_install_isr_service(0));

    // Handle instance of the rotary device, by default 1 mS for debounce time.
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
    ESP_ERROR_CHECK(rotenc_set_event_callback(&handle, log_event));
#elif CONFIG_REPORT_MODE_POLLING
    ESP_LOGI(TAG, "Report mode by polling");
#endif

    while (1) {
#if CONFIG_REPORT_MODE_QUEUE
        rotenc_event_t event = { 0 };
        if (rotenc_wait_event(&handle, &event) == ESP_OK) {
            log_event(event);
        }
#elif CONFIG_REPORT_MODE_CALLBACK
        vTaskDelay(1000 / portTICK_PERIOD_MS);
#elif CONFIG_REPORT_MODE_POLLING
        vTaskDelay(100 / portTICK_PERIOD_MS);
        rotenc_event_t event = { 0 };
        ESP_ERROR_CHECK(rotenc_polling(&handle, &event));
        log_event(event);
#endif
    }
}

