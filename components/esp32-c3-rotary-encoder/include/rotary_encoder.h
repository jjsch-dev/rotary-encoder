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
 * @file rotary_encoder.h
 * @brief API definitions for Incremental Rotary Encoder component for ESP32.
 *
 * This component provides a means to interface with a encoders that produce a 
 * quadrature signal on two outputs, which can be used to track position and 
 * direction as movement occurs.
 *
 * To reduce interruptions, from bounces of the mechanical contact to false 
 * triggers due to the lack of a Schmitt trigger in the ESP32, in the idle 
 * state the clock pin has the irq enabled and the data pin does not, when 
 * it detects an falling edge, it toggle the irq enable and activates a timer 
 * which, when it expires, verifies that the clock pin is low and uses the 
 * data pin to set the direction of rotation. 
 * The irq of the clock becomes active when it detects any edge on the data pin, 
 * or there is an error in the decoding of the direction of rotation in the 
 * timer callback.
 * 
 * The notification can be done in three ways: polling using the rotenc_get_state 
 * function, by callback using the rotenc_set_event_callback function, or by 
 * message queue using the rotenc_wait_event function.
 * 
 */

#ifndef ROTARY_ENCODER_H
#define ROTARY_ENCODER_H

#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "esp_timer.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Enum representing the direction of rotation.
 */
typedef enum
{
    ROTENC_NOT_SET = 0,             ///< Direction unknown.
    ROTENC_CW,                      ///< Clockwise, depends on the flip option.
    ROTENC_CCW,                     ///< Counter clockwise, depends on the flip option.
} rotenc_direction_t;

/**
 * @brief Struct position and direction of last movement of the device.
 */
typedef struct
{
    int32_t position;               ///< Numerical position since reset. 
    rotenc_direction_t direction;   ///< Direction of last movement. Set to NOT_SET on reset.
} rotenc_event_t;

/**
 * @brief Position Event callback function type
 * @param event direction and numerical position. 
 */
typedef void (*rotenc_event_cb_t)(rotenc_event_t event);

/**
 * @brief Button callback function type
 * @param arg pointer to opaque user-specific data
 */
typedef void (*rotenc_button_cb_t)(void*);

/**
 * @brief Struct contains information to control the button.
 */
typedef struct 
{
    rotenc_button_cb_t callback;        ///< Function to call when button is pressed.
    gpio_num_t pin;                     ///< GPIO for push button.
    esp_timer_handle_t timer;           ///< Software timer to apply the anti-bounce.
    uint32_t debounce_us;               ///< Period in uS for anti-bounce.
    bool cb_invoked;                    ///< True, the button callback has already been invoked.
} rotenc_button_t;

/**
 * @brief Struct contains information to the events by queue.
 */
typedef struct 
{
    QueueHandle_t queue;                ///< Handle for event queue.
    uint32_t wait_ms;                   ///< Time in mS that waits for the reception of an event.
} rotenc_queue_t;

/**
 * @brief Struct instance of the driver for a rotary encoder device.
 */
typedef struct
{
    gpio_num_t pin_clk;                 ///< GPIO for clock (A) from the rotary encoder device.
    gpio_num_t pin_dta;                 ///< GPIO for data (B) from the rotary encoder device.
    rotenc_queue_t q_event;             ///< Information for events by queue.
    esp_timer_handle_t debounce_timer;  ///< Software timer to apply the anti-bounce.
    uint32_t debounce_us;               ///< Period in uS that the anti-bounce takes. 
    volatile rotenc_event_t state;      ///< Device state.
    bool flip_direction;                ///< Reverse (flip) the sense of the direction.
    rotenc_event_cb_t event_callback;   ///< Function to call when there is a new position event.
    int irq_data_level;                 ///< The value of the data pin when the irq enters.
    rotenc_button_t button;             ///< Button information.
} rotenc_handle_t;

/**
 * @brief Initialise the rotary encoder device.
 * @param[in, out] handle Pointer to allocated rotary encoder instance.
 * @param[in] pin_clk GPIO number for clock (A) (triggers the IRQ on a falling edge).
 * @param[in] pin_data GPIO number for data (B) (read only, to detect the direction of rotation).
 * @param[in] debounce_us Period in uS that the anti-bounce takes, by default 1000 uS.
 * @return ESP_OK if successful, ESP_FAIL or ESP_ERR_* if an error occurred.
 */
esp_err_t rotenc_init(rotenc_handle_t * handle, gpio_num_t pin_a, gpio_num_t pin_b, uint32_t debounce_us);

/**
 * @brief Reverse (flip) the sense of the direction.
 * @param[in] handle Pointer to allocated rotary encoder instance.
 * @return ESP_OK if successful, ESP_FAIL or ESP_ERR_* if an error occurred.
 */
esp_err_t rotenc_flip_direction(rotenc_handle_t * handle);

/**
 * @brief Uninitialize the handlers and driver resources.     
 * @param[in] handle Pointer to allocated rotary encoder instance.
 * @return ESP_OK if successful, ESP_FAIL or ESP_ERR_* if an error occurred.
 */
esp_err_t rotenc_uninit(rotenc_handle_t * handle);

/**
 * @brief Configure a queue to proccess the rotary event.
 *        Note: If the report is already done by callback, it returns a status error.
 * @param[in] handle Pointer to allocated rotary encoder instance.
 * @param[in] wait_time_ms Time in mS that waits for the reception of an event.
 * @return ESP_OK if successful, or ESP_ERR_* if an error.
 */
esp_err_t rotenc_set_event_queue(rotenc_handle_t * handle, uint32_t wait_time_ms);

/**
 * @brief Configure the callback function to proccess the rotary event.
 *        Note: If the report is already done by queue, it returns a status error.
 * @param[in] handle Pointer to allocated rotary encoder instance.
 * @param[in] rotenc_event_cb_t Function to call when there is a new position event.
 * @return ESP_OK if successful, or ESP_ERR_* if an error.
 */
esp_err_t rotenc_set_event_callback(rotenc_handle_t * handle, rotenc_event_cb_t callback);

/**
 * @brief Wait for FreeRtos queue events. 
 * @param[in] handle Pointer to allocated rotary encoder instance.
 * @param[in] event Pointer of the struct to store the event.
 * @return ESP_OK if successful, ESP_TIMEOUT when event timeout expired. or ESP_ERR_* if an error occurred.
 */
esp_err_t rotenc_wait_event(rotenc_handle_t * handle, rotenc_event_t* event);

/**
 * @brief Poll the current position of the rotary encoder.
 * @param[in] handle Pointer to allocated rotary encoder instance.
 * @param[in, out] event Pointer of the struct to store the event.
 * @return ESP_OK if successful, ESP_FAIL or ESP_ERR_* if an error occurred.
 */
esp_err_t rotenc_polling(const rotenc_handle_t * handle, rotenc_event_t * event);

/**
 * @brief Reset the current position of the rotary encoder to zero.
 * @param[in] handle Pointer to allocated rotary encoder instance.
 * @return ESP_OK if successful, ESP_FAIL or ESP_ERR_* if an error occurred.
 */
esp_err_t rotenc_reset(rotenc_handle_t * handle);

/**
 * @brief Configure the push button.
 * @param[in] handle Pointer to allocated rotary encoder instance.
 * @param[in] pin GPIO number for push button.
 * @param[in] debounce_us Period in uS for the debounce, by default 1000 uS.
 * @param[in] callback Function to call when the button is pressed.
 * @return ESP_OK if successful, or ESP_ERR_* if an error.
 */
esp_err_t rotenc_init_button(rotenc_handle_t * handle, 
                             gpio_num_t pin_b, uint32_t debounce_us, 
                             rotenc_button_cb_t callback);

#ifdef __cplusplus
}
#endif

#endif  // ROTARY_ENCODER_H
