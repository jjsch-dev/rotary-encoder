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

/**
 * @file rotary_encoder.c
 * @brief Driver implementation Incremental Rotary Encoder component for ESP32-C3.
 */

#include "rotary_encoder.h"

#include "esp_log.h"

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
 * @param[in] arg Pointer to allocated rotary encoder instance.
 * @return void
 */
static void rotenc_enable_clk_irq(rotenc_handle_t * handle)
{
   gpio_intr_disable(handle->pin_dta);
   gpio_intr_enable(handle->pin_clk);
}

/**
 * @brief Disable clock IRQ and enable data IRQ.
 * @param[in] arg Pointer to allocated rotary encoder instance.
 * @return void
 */
static void rotenc_disable_clk_irq(rotenc_handle_t * handle)
{
    gpio_intr_disable(handle->pin_clk);
    gpio_intr_enable(handle->pin_dta);
}

/**
 * @brief When the anti-bounce timer expires, check that the sequence 
 *        (pin levels) is valid to update the rotary enconder status.
 * @param[in] arg Pointer to allocated rotary encoder instance.
 * @return void
 */
static void IRAM_ATTR rotenc_debounce_callback(void *arg)
{
    rotenc_handle_t * handle = (rotenc_handle_t*) arg;

    // Updates the rotation state, when the clock pin is low and 
    // the data pin level match the start state (when irq occurred).
    if (!gpio_get_level(handle->pin_clk) && 
       (handle->irq_data_level == gpio_get_level(handle->pin_dta))) {
        
        rotenc_toggle_test_pin();
        
        // Reverses rotation direction when flip is enabled.
        bool data_level = handle->irq_data_level ? true : false;
        if (data_level == !handle->flip_direction) {
            ++handle->state.position;
            handle->state.direction = ROTENC_CW;
        } else {
            --handle->state.position;
            handle->state.direction = ROTENC_CCW;
        }

        rotenc_event_t event = {
            .position = handle->state.position,
            .direction = handle->state.direction,
        };
        
        if (handle->q_event.queue) {
            xQueueOverwrite(handle->q_event.queue, &event);
        } else if (handle->event_callback) {
            handle->event_callback(event);
        }
    } else { 
        rotenc_enable_clk_irq(handle);
    }
}

/**
 * @brief Re-enable clock pin irq.
 * @param[in] args Pointer to allocated rotary encoder instance.
 * @return void
 */
static void IRAM_ATTR rotenc_isr_dta(void * args)
{
    rotenc_handle_t * handle = (rotenc_handle_t *)args;
    rotenc_enable_clk_irq(handle);
    esp_timer_stop(handle->debounce_timer);
    rotenc_toggle_test_pin();
}

/**
 * @brief It sets the anti-bounce timer each time it detects a falling edge.
 * @param[in] args Pointer to allocated rotary encoder instance.
 * @return void
 */
static void IRAM_ATTR rotenc_isr_clk(void * args)
{
    rotenc_handle_t * handle = (rotenc_handle_t *)args;
    rotenc_disable_clk_irq(handle);

    rotenc_toggle_test_pin();
  
    handle->irq_data_level = gpio_get_level(handle->pin_dta);

    esp_timer_stop(handle->debounce_timer);
    ESP_ERROR_CHECK(esp_timer_start_once(handle->debounce_timer, handle->debounce_us));    
}

/**
 * @brief When the anti-bounce timer expires, check if the button's callback function 
 * has already been invoked, and if the button is released to reactivate the irq.
 * @param[in] arg Pointer to allocated rotary encoder instance.
 * @return void
 */
static void IRAM_ATTR rotenc_button_timer_callback(void *arg)
{
    rotenc_handle_t * handle = (rotenc_handle_t*) arg;

    if( !handle->button.cb_invoked) {
        handle->button.cb_invoked = true;
        handle->button.callback(arg);
    } else if (gpio_get_level(handle->button.pin)) {
        gpio_set_intr_type(handle->button.pin, GPIO_INTR_NEGEDGE);
        return;
    }

    ESP_ERROR_CHECK(esp_timer_start_once(handle->button.timer, handle->button.debounce_us));
}

/**
 * @brief Set the anti-bounce timer to 10 uS and stop the IRQ. 
 *        NOTE: the button's callback function is invoked in the 
 *        timer to quickly release the IRQ.
 * @param[in] args Pointer to allocated rotary encoder instance.
 * @return void
 */
static void IRAM_ATTR rotenc_button_isr(void * args)
{
    rotenc_handle_t * handle = (rotenc_handle_t *)args;
    gpio_set_intr_type(handle->button.pin, GPIO_INTR_DISABLE);

    handle->button.cb_invoked = false;
    esp_timer_stop(handle->button.timer);
    ESP_ERROR_CHECK(esp_timer_start_once(handle->button.timer, 10)); 
}

esp_err_t rotenc_init(rotenc_handle_t * handle, 
                      gpio_num_t pin_clk, 
                      gpio_num_t pin_dta, uint32_t debounce_us)
{
    esp_err_t err = ESP_OK;
    if (handle) {
        handle->pin_clk = pin_clk;
        handle->pin_dta = pin_dta;
        handle->state.position = 0;
        handle->state.direction = ROTENC_NOT_SET;
        handle->debounce_us = debounce_us;
        handle->flip_direction = false;
        handle->event_callback = NULL;
        handle->q_event.queue = NULL;
 
        const esp_timer_create_args_t debounce_timer_args = {
            .callback = &rotenc_debounce_callback,
            .arg = (void *)handle,
            .name = "debounce-timer"};
        
        ESP_ERROR_CHECK(esp_timer_create(&debounce_timer_args, &handle->debounce_timer));

        // configure GPIOs
        gpio_reset_pin(handle->pin_clk);
        gpio_set_pull_mode(handle->pin_clk, GPIO_PULLUP_ONLY);
        gpio_set_direction(handle->pin_clk, GPIO_MODE_INPUT);
        gpio_set_intr_type(handle->pin_clk, GPIO_INTR_NEGEDGE);


        gpio_reset_pin(handle->pin_dta);
        gpio_set_pull_mode(handle->pin_dta, GPIO_PULLUP_ONLY);
        gpio_set_direction(handle->pin_dta, GPIO_MODE_INPUT);
        gpio_set_intr_type(handle->pin_dta, GPIO_INTR_ANYEDGE);
        
        
        // install interrupt handlers
        gpio_isr_handler_add(handle->pin_clk, rotenc_isr_clk, handle);
        gpio_isr_handler_add(handle->pin_dta, rotenc_isr_dta, handle);
#if CONFIG_TEST_PIN_ENABLE
        gpio_reset_pin(CONFIG_ROT_ENC_TEST_PIN_GPIO);
        gpio_set_direction(CONFIG_ROT_ENC_TEST_PIN_GPIO, GPIO_MODE_INPUT_OUTPUT);
#endif
    } else {
        ESP_LOGE(TAG, "handle is NULL");
        err = ESP_ERR_INVALID_ARG;
    }
    return err;
}

esp_err_t rotenc_init_button(rotenc_handle_t * handle, 
                             gpio_num_t pin, uint32_t debounce_us, 
                             rotenc_button_cb_t callback)
{
    esp_err_t err = ESP_OK;
    if (handle) {
        if(!handle->button.callback){
            handle->button.pin = pin;
            handle->button.callback = callback;
            handle->button.debounce_us = debounce_us;

            const esp_timer_create_args_t button_timer_args = {
                .callback = &rotenc_button_timer_callback,
                .arg = (void *)handle,
                .name = " button-timer"};
        
            ESP_ERROR_CHECK(esp_timer_create(&button_timer_args, &handle->button.timer));

            gpio_reset_pin(handle->button.pin);
            gpio_set_pull_mode(handle->button.pin, GPIO_PULLUP_ONLY);
            gpio_set_direction(handle->button.pin, GPIO_MODE_INPUT);
            gpio_set_intr_type(handle->button.pin, GPIO_INTR_NEGEDGE);

            // install interrupt handler
            gpio_isr_handler_add(handle->button.pin, rotenc_button_isr, handle);
        } else {
            ESP_LOGE(TAG, "push button already inited");
            err = ESP_ERR_INVALID_STATE;
        }
    }else {
        ESP_LOGE(TAG, "handle is NULL");
        err = ESP_ERR_INVALID_ARG;
    }
    return err;
}

esp_err_t rotenc_flip_direction(rotenc_handle_t * handle)
{
    esp_err_t err = ESP_OK;
    if (handle) {
        handle->flip_direction = !handle->flip_direction;
    } else {
        ESP_LOGE(TAG, "handle is NULL");
        err = ESP_ERR_INVALID_ARG;
    }
    return err;
}

esp_err_t rotenc_uninit(rotenc_handle_t * handle)
{
    esp_err_t err = ESP_OK;
    if (handle) {
        gpio_isr_handler_remove(handle->pin_clk);
        gpio_isr_handler_remove(handle->pin_dta);

        if (handle->q_event.queue) {
            vQueueDelete(handle->q_event.queue);
            handle->q_event.queue = NULL;
        }
        
        gpio_reset_pin(handle->pin_clk);
        gpio_reset_pin(handle->pin_dta);

        handle->event_callback = NULL;

        if(handle->button.callback){
            gpio_isr_handler_remove(handle->button.pin);
            gpio_reset_pin(handle->button.pin);
            handle->button.callback = NULL;
        }
    } else {
        ESP_LOGE(TAG, "handle is NULL");
        err = ESP_ERR_INVALID_ARG;
    }
    return err;
}

esp_err_t rotenc_set_event_callback(rotenc_handle_t * handle, rotenc_event_cb_t callback)
{
esp_err_t err = ESP_OK;

    if (handle) {
        if (!handle->q_event.queue) {
            handle->event_callback = callback;
        } else {
            ESP_LOGE(TAG, "could not be created because there is a queue created.");
            err = ESP_ERR_INVALID_STATE;
        }
    } else {
        ESP_LOGE(TAG, "handle is NULL");
        err = ESP_ERR_INVALID_ARG;
    }
    return err;
}

esp_err_t rotenc_set_event_queue(rotenc_handle_t * handle, uint32_t wait_time_ms)
{
    esp_err_t err = ESP_OK;

    if (handle) {
        if (!handle->event_callback) {   
            handle->q_event.queue = xQueueCreate(1, sizeof(rotenc_event_t));

            handle->q_event.wait_ms = wait_time_ms;

            if (!handle->q_event.queue) {
                ESP_LOGE(TAG, "queue could not be created");
                err = ESP_ERR_NO_MEM;
            }
        } else {
            ESP_LOGE(TAG, "could not be created because there is a callback created.");
            err = ESP_ERR_INVALID_STATE;
        }
    } else {
        ESP_LOGE(TAG, "handle is NULL");
        err = ESP_ERR_INVALID_ARG;
    }
    return err;
}

esp_err_t rotenc_wait_event(rotenc_handle_t * handle, rotenc_event_t* event)
{
    esp_err_t err = ESP_OK;
    if (handle && handle->q_event.queue && event) {
        if (xQueueReceive(handle->q_event.queue, event, 
                          handle->q_event.wait_ms / portTICK_PERIOD_MS) != pdTRUE) {
            err = ESP_ERR_TIMEOUT;
        }
    } else {
        ESP_LOGE(TAG, "handle and/or queue, event is NULL");
        err = ESP_ERR_INVALID_ARG;
    }
    return err;
}

esp_err_t rotenc_polling(const rotenc_handle_t * handle, rotenc_event_t * event)
{
    esp_err_t err = ESP_OK;
    if (handle && event) {
        event->position = handle->state.position;
        event->direction = handle->state.direction;
    } else {
        ESP_LOGE(TAG, "handle and/or state is NULL");
        err = ESP_ERR_INVALID_ARG;
    }
    return err;
}

esp_err_t rotenc_reset(rotenc_handle_t * handle)
{
    esp_err_t err = ESP_OK;
    if (handle) {
        handle->state.position = 0;
        handle->state.direction = ROTENC_NOT_SET;
    } else {
        ESP_LOGE(TAG, "handle is NULL");
        err = ESP_ERR_INVALID_ARG;
    }
    return err;
}
