/*
 * Copyright (c) 2021 Juan Schiavoni
 *
 * This file is part of the esp32-c3 rotary-encoder component.
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

/**
 * @file rotary_encoder.c
 * @brief Driver implementation Incremental Rotary Encoder component for ESP32-C3.
 */

#include "rotary_encoder.h"

#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_timer.h"

#define TAG "rotenc"

/**
 * @brief Toggle test pin to debug irqs events.
 * @param[in] void
 * @return void
 */
static void rotenc_toggle_test_pin(void)
{
#if CONFIG_TEST_PIN_ENABLE    
    gpio_set_level(CONFIG_ROT_ENC_TEST_PIN_GPIO, !gpio_get_level(CONFIG_ROT_ENC_TEST_PIN_GPIO));
#endif
}

/**
 * @brief Enable clock IRQ and disable data IRQ.
 * @param[in] arg Pointer to initialised rotary encoder info structure.
 * @return void
 */
static void rotenc_enable_clk_irq(rotenc_info_t * info)
{
    gpio_set_intr_type(info->pin_dta, GPIO_INTR_DISABLE);
    gpio_set_intr_type(info->pin_clk, GPIO_INTR_NEGEDGE);
}

/**
 * @brief Disable clock IRQ and enable data IRQ.
 * @param[in] arg Pointer to initialised rotary encoder info structure.
 * @return void
 */
static void rotenc_disable_clk_irq(rotenc_info_t * info)
{
    gpio_set_intr_type(info->pin_clk, GPIO_INTR_DISABLE);
    gpio_set_intr_type(info->pin_dta, GPIO_INTR_ANYEDGE);
}

/**
 * @brief When the anti-bounce timer expires, check that the sequence 
 *        (pin levels) is valid to update the rotary enconder status.
 * @param[in] arg Pointer to initialised rotary encoder info structure.
 * @return void
 */
static void rotenc_debounce_callback(void *arg)
{
    rotenc_info_t * info = (rotenc_info_t*) arg;

    // Updates the rotation state, when the clock pin is low and 
    // the data pin level match the start state (when irq occurred).
    if (!gpio_get_level(info->pin_clk) && 
       (info->irq_data_level == gpio_get_level(info->pin_dta))) {
        
        rotenc_toggle_test_pin();
        
        // Reverses rotation direction when flip is enabled.
        bool data_level = info->irq_data_level ? true : false;
        if (data_level == !info->flip_direction) {
            ++info->state.position;
            info->state.direction = ROTENC_CW;
        } else {
            --info->state.position;
            info->state.direction = ROTENC_CCW;
        }

        rotenc_event_t event = {
            .position = info->state.position,
            .direction = info->state.direction,
        };
        
        if (info->q_event.queue) {
            xQueueOverwrite(info->q_event.queue, &event);
        } else if (info->event_callback) {
            info->event_callback(event);
        }
    } else { 
        rotenc_enable_clk_irq(info);
    }
}

/**
 * @brief Re-enable clock pin irq.
 * @param[in] args Pointer to initialised rotary encoder info structure.
 * @return void
 */
static void rotenc_isr_dta(void * args)
{
    rotenc_info_t * info = (rotenc_info_t *)args;
    rotenc_enable_clk_irq(info);
    esp_timer_stop(info->debounce_timer);
    rotenc_toggle_test_pin();
}

/**
 * @brief It sets the anti-bounce timer each time it detects a falling edge.
 * @param[in] args Pointer to initialised rotary encoder info structure.
 * @return void
 */
static void rotenc_isr_clk(void * args)
{
    rotenc_info_t * info = (rotenc_info_t *)args;
    rotenc_disable_clk_irq(info);

    rotenc_toggle_test_pin();
  
    info->irq_data_level = gpio_get_level(info->pin_dta);

    esp_timer_stop(info->debounce_timer);
    ESP_ERROR_CHECK(esp_timer_start_once(info->debounce_timer, info->debounce_us));       
}

/**
 * @brief When the anti-bounce timer expires, check if the button's callback function 
 * has already been invoked, and if the button is released to reactivate the irq.
 * @param[in] arg Pointer to initialised rotary encoder info structure.
 * @return void
 */
static void rotenc_button_timer_callback(void *arg)
{
    rotenc_info_t * info = (rotenc_info_t*) arg;

    if( !info->button.cb_invoked) {
        info->button.cb_invoked = true;
        info->button.callback(arg);
    } else if (gpio_get_level(info->button.pin)) {
        gpio_set_intr_type(info->button.pin, GPIO_INTR_NEGEDGE);
        return;
    }

    ESP_ERROR_CHECK(esp_timer_start_once(info->button.timer, info->button.debounce_us));
}

/**
 * @brief Set the anti-bounce timer to 10 uS and stop the IRQ. 
 *        NOTE: the button's callback function is invoked in the 
 *        timer to quickly release the IRQ.
 * @param[in] args Pointer to initialised rotary encoder info structure.
 * @return void
 */
static void rotenc_button_isr(void * args)
{
    rotenc_info_t * info = (rotenc_info_t *)args;
    gpio_set_intr_type(info->button.pin, GPIO_INTR_DISABLE);

    info->button.cb_invoked = false;
    esp_timer_stop(info->button.timer);
    ESP_ERROR_CHECK(esp_timer_start_once(info->button.timer, 10)); 
}

esp_err_t rotenc_init(rotenc_info_t * info, 
                      gpio_num_t pin_clk, 
                      gpio_num_t pin_dta, uint32_t debounce_us)
{
    esp_err_t err = ESP_OK;
    if (info) {
        info->pin_clk = pin_clk;
        info->pin_dta = pin_dta;
        info->state.position = 0;
        info->state.direction = ROTENC_NOT_SET;
        info->debounce_us = debounce_us;
        info->flip_direction = false;
        info->event_callback = NULL;
        info->q_event.queue = NULL;
 
        const esp_timer_create_args_t debounce_timer_args = {
            .callback = &rotenc_debounce_callback,
            .arg = (void *)info,
            .name = "debounce-timer"};
        
        ESP_ERROR_CHECK(esp_timer_create(&debounce_timer_args, &info->debounce_timer));

        // configure GPIOs
        gpio_reset_pin(info->pin_clk);
        gpio_pad_select_gpio(info->pin_clk);
        gpio_set_pull_mode(info->pin_clk, GPIO_PULLUP_ONLY);
        gpio_set_direction(info->pin_clk, GPIO_MODE_INPUT);
        gpio_set_intr_type(info->pin_clk, GPIO_INTR_NEGEDGE);

        gpio_reset_pin(info->pin_dta);
        gpio_pad_select_gpio(info->pin_dta);
        gpio_set_pull_mode(info->pin_dta, GPIO_PULLUP_ONLY);
        gpio_set_direction(info->pin_dta, GPIO_MODE_INPUT);
        gpio_set_intr_type(info->pin_dta, GPIO_INTR_ANYEDGE);

        // install interrupt handlers
        gpio_isr_handler_add(info->pin_clk, rotenc_isr_clk, info);
        gpio_isr_handler_add(info->pin_dta, rotenc_isr_dta, info);

#if CONFIG_TEST_PIN_ENABLE
        gpio_pad_select_gpio(CONFIG_ROT_ENC_TEST_PIN_GPIO);
        gpio_set_direction(CONFIG_ROT_ENC_TEST_PIN_GPIO, GPIO_MODE_INPUT_OUTPUT);
#endif
    } else {
        ESP_LOGE(TAG, "info is NULL");
        err = ESP_ERR_INVALID_ARG;
    }
    return err;
}

esp_err_t rotenc_init_button(rotenc_info_t * info, 
                             gpio_num_t pin, uint32_t debounce_us, 
                             rotenc_button_cb_t callback)
{
    esp_err_t err = ESP_OK;
    if (info) {
        if(!info->button.callback){
            info->button.pin = pin;
            info->button.callback = callback;
            info->button.debounce_us = debounce_us;

            const esp_timer_create_args_t button_timer_args = {
                .callback = &rotenc_button_timer_callback,
                .arg = (void *)info,
                .name = " button-timer"};
        
            ESP_ERROR_CHECK(esp_timer_create(&button_timer_args, &info->button.timer));

            gpio_reset_pin(info->button.pin);
            gpio_pad_select_gpio(info->button.pin);
            gpio_set_pull_mode(info->button.pin, GPIO_PULLUP_ONLY);
            gpio_set_direction(info->button.pin, GPIO_MODE_INPUT);
            gpio_set_intr_type(info->button.pin, GPIO_INTR_NEGEDGE);

            // install interrupt handler
            gpio_isr_handler_add(info->button.pin, rotenc_button_isr, info);
        } else {
            ESP_LOGE(TAG, "push button already inited");
            err = ESP_ERR_INVALID_STATE;
        }
    }else {
        ESP_LOGE(TAG, "info is NULL");
        err = ESP_ERR_INVALID_ARG;
    }
    return err;
}

esp_err_t rotenc_flip_direction(rotenc_info_t * info)
{
    esp_err_t err = ESP_OK;
    if (info) {
        info->flip_direction = !info->flip_direction;
    } else {
        ESP_LOGE(TAG, "info is NULL");
        err = ESP_ERR_INVALID_ARG;
    }
    return err;
}

esp_err_t rotenc_uninit(rotenc_info_t * info)
{
    esp_err_t err = ESP_OK;
    if (info) {
        gpio_isr_handler_remove(info->pin_clk);
        gpio_isr_handler_remove(info->pin_dta);

        if (info->q_event.queue) {
            vQueueDelete(info->q_event.queue);
            info->q_event.queue = NULL;
        }
        
        gpio_reset_pin(info->pin_clk);
        gpio_reset_pin(info->pin_dta);

        info->event_callback = NULL;

        if(info->button.callback){
            gpio_isr_handler_remove(info->button.pin);
            gpio_reset_pin(info->button.pin);
            info->button.callback = NULL;
        }
    } else {
        ESP_LOGE(TAG, "info is NULL");
        err = ESP_ERR_INVALID_ARG;
    }
    return err;
}

esp_err_t rotenc_set_event_callback(rotenc_info_t * info, rotenc_event_cb_t callback)
{
esp_err_t err = ESP_OK;

    if (info) {
        if (!info->q_event.queue) {
            info->event_callback = callback;
        } else {
            ESP_LOGE(TAG, "could not be created because there is a queue created.");
            err = ESP_ERR_INVALID_STATE;
        }
    } else {
        ESP_LOGE(TAG, "info is NULL");
        err = ESP_ERR_INVALID_ARG;
    }
    return err;
}

esp_err_t rotenc_set_event_queue(rotenc_info_t * info, uint32_t wait_time_ms)
{
    esp_err_t err = ESP_OK;

    if (info) {
        if (!info->event_callback) {   
            info->q_event.queue = xQueueCreate(1, sizeof(rotenc_event_t));

            info->q_event.wait_ms = wait_time_ms;

            if (!info->q_event.queue) {
                ESP_LOGE(TAG, "queue could not be created");
                err = ESP_ERR_NO_MEM;
            }
        } else {
            ESP_LOGE(TAG, "could not be created because there is a callback created.");
            err = ESP_ERR_INVALID_STATE;
        }
    } else {
        ESP_LOGE(TAG, "info is NULL");
        err = ESP_ERR_INVALID_ARG;
    }
    return err;
}

esp_err_t rotenc_wait_event(rotenc_info_t * info, rotenc_event_t* event)
{
    esp_err_t err = ESP_OK;
    if (info && info->q_event.queue && event) {
        if (xQueueReceive(info->q_event.queue, event, 
                          info->q_event.wait_ms / portTICK_PERIOD_MS) != pdTRUE) {
            err = ESP_ERR_TIMEOUT;
        }
    } else {
        ESP_LOGE(TAG, "info and/or queue, event is NULL");
        err = ESP_ERR_INVALID_ARG;
    }
    return err;
}

esp_err_t rotenc_polling(const rotenc_info_t * info, rotenc_event_t * event)
{
    esp_err_t err = ESP_OK;
    if (info && event) {
        event->position = info->state.position;
        event->direction = info->state.direction;
    } else {
        ESP_LOGE(TAG, "info and/or state is NULL");
        err = ESP_ERR_INVALID_ARG;
    }
    return err;
}

esp_err_t rotenc_reset(rotenc_info_t * info)
{
    esp_err_t err = ESP_OK;
    if (info) {
        info->state.position = 0;
        info->state.direction = ROTENC_NOT_SET;
    } else {
        ESP_LOGE(TAG, "info is NULL");
        err = ESP_ERR_INVALID_ARG;
    }
    return err;
}
