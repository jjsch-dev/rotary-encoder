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

#ifdef __cplusplus
extern "C" {
#endif

typedef int32_t rotenc_position_t;

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
 * @brief Struct represents the current state of the device in terms of incremental position and direction of last movement
 */
typedef struct
{
    rotenc_position_t position;     ///< Numerical position since reset. 
    rotenc_direction_t direction;   ///< Direction of last movement. Set to NOT_SET on reset.
} rotenc_state_t;

/**
 * @brief Struct represents an event, used to communicate current position to a waiting task
 */
typedef struct
{
    rotenc_state_t state;           ///< The device state corresponding to this event
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
 * @brief Struct carries all the information needed by this driver to manage the rotary encoder device.
 *        The fields of this structure should not be accessed directly.
 */
typedef struct
{
    gpio_num_t pin_clk;                 ///< GPIO for clock (A) from the rotary encoder device.
    gpio_num_t pin_dta;                 ///< GPIO for data (B) from the rotary encoder device.
    QueueHandle_t queue;                ///< Handle for event queue.
    esp_timer_handle_t debounce_timer;  ///< Software timer to apply the anti-bounce.
    uint32_t debounce_us;               ///< Period in uS that the anti-bounce takes. 
    volatile rotenc_state_t state;      ///< Device state.
    uint32_t event_wait_time_ms;        ///< Time in mS that waits for the reception of an event.
    bool flip_direction;                ///< Reverse (flip) the sense of the direction.
    rotenc_event_cb_t event_callback;   ///< Function to call when there is a new position event.
    int irq_data_level;                 ///< The value of the data pin when the irq enters.
    rotenc_button_t button;             ///< Button information.
} rotenc_info_t;

/**
 * @brief Initialise the rotary encoder device with the specified GPIO pins and full step increments.
 *        This function will set up the GPIOs as needed,
 *        Note: this function assumes that gpio_install_isr_service(0) has already been called.
 * @param[in, out] info Pointer to allocated rotary encoder info structure.
 * @param[in] pin_clk GPIO number for clock (A) (triggers the IRQ on a falling edge).
 * @param[in] pin_data GPIO number for data (B) (read only, to detect the direction of rotation).
 * @param[in] debounce_us Period in uS that the anti-bounce takes, by default 1000 uS.
 * @return ESP_OK if successful, ESP_FAIL or ESP_ERR_* if an error occurred.
 */
esp_err_t rotenc_init(rotenc_info_t * info, gpio_num_t pin_a, gpio_num_t pin_b, uint32_t debounce_us);

/**
 * @brief Reverse (flip) the sense of the direction.
 *        Use this if clockwise/counterclockwise are not what you expect.
 * @param[in] info Pointer to initialised rotary encoder info structure.
 * @return ESP_OK if successful, ESP_FAIL or ESP_ERR_* if an error occurred.
 */
esp_err_t rotenc_flip_direction(rotenc_info_t * info);

/**
 * @brief Eliminate interrupt handlers and if the event was by queue, delete it.
 *        
 * @param[in] info Pointer to initialised rotary encoder info structure.
 * @return ESP_OK if successful, ESP_FAIL or ESP_ERR_* if an error occurred.
 */
esp_err_t rotenc_uninit(rotenc_info_t * info);

/**
 * @brief Configure a queue to proccess the rotary event.
 * @param[in] info Pointer with the initialised rotary encoder info structure.
 * @param[in] wait_time_ms Time in mS that waits for the reception of an event.
 * @return ESP_OK if successful, or ESP_ERR_* if an error.
 */
esp_err_t rotenc_set_event_queue(rotenc_info_t * info, uint32_t wait_time_ms);

/**
 * @brief Configure the callback function to proccess the rotary event.
 * @param[in] info Pointer with the initialised rotary encoder info structure.
 * @param[in] rotenc_event_cb_t Function to call when there is a new position event.
 * @return ESP_OK if successful, or ESP_ERR_* if an error.
 */
esp_err_t rotenc_set_event_callback(rotenc_info_t * info, rotenc_event_cb_t callback);

/**
 * @brief Wait queue events. 
 * @param[in] info Pointer with the initialised rotary encoder info structure.
 * @param[in] event Pointer of the struct to store the event.
 * @return ESP_OK if successful, ESP_TIMEOUT when event timeout expired. or ESP_ERR_* if an error occurred.
 */
esp_err_t rotenc_wait_event(rotenc_info_t * info, rotenc_event_t* event);

/**
 * @brief Get the current position of the rotary encoder.
 * @param[in] info Pointer to initialised rotary encoder info structure.
 * @param[in, out] state Pointer to an allocated rotenc_state_t struct that will
 * @return ESP_OK if successful, ESP_FAIL or ESP_ERR_* if an error occurred.
 */
esp_err_t rotenc_get_state(const rotenc_info_t * info, rotenc_state_t * state);

/**
 * @brief Reset the current position of the rotary encoder to zero.
 * @param[in] info Pointer to initialised rotary encoder info structure.
 * @return ESP_OK if successful, ESP_FAIL or ESP_ERR_* if an error occurred.
 */
esp_err_t rotenc_reset(rotenc_info_t * info);

/**
 * @brief Configure the push button.
 * @param[in] info Pointer with the initialised rotary encoder info structure.
 * @param[in] pin GPIO number for push button.
 * @param[in] debounce_us Period in uS for the debounce, by default 1000 uS.
 * @param[in] callback Function to call when the button is pressed.
 * @return ESP_OK if successful, or ESP_ERR_* if an error.
 */
esp_err_t rotenc_init_button(rotenc_info_t * info, 
                             gpio_num_t pin_b, uint32_t debounce_us, 
                             rotenc_button_cb_t callback);

#ifdef __cplusplus
}
#endif

#endif  // ROTARY_ENCODER_H
