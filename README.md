# ESP32-C3 Rotary Encoder Example

This is an example of using the [esp32-c3-rotary-encoder](https://github.com/) driver to track the relative position of an [incremental](https://tech.alpsalpine.com/prod/e/html/encoder/incremental/ec11/ec11_list.html) rotary encoder.

![alt text](images/EC11B202440M.png)
![alt text](images/ky04-20.png)

The need to develop this component arises from using an incremental encoder in an IOT project with an ESP32-C3 processor that lacks the [PCNT](https://github.com/espressif/esp-idf/tree/73db142/examples/peripherals/pcnt/rotary_encoder) module that supports the rotary encoder component by espressif.

On GitHub there are many components developed for esp32, some very good like [David Antliff](https://github.com/DavidAntliff/esp32-rotary-encoder-example). While it works, it makes extensive use of IRQs to decode the two bit gray code. 

Therefore, to reduce misfires due to contact bounces and lack of Schmitt trigger, I used the strategy of defining one of the pins as clock (A) and the other as data (B), in idle state the irq clock is enabled and data disabled. When the user changes the shaft position, the isr is triggered which deactivates the irq clock, enables the irq data and programs a timer, which when it expires defines the position and the state, and the last step is to wait for the data irq to return to the idle state.

## Circuit

1. Connect GND to the ESP32-C3 ground reference.
2. Connect + to the ESP32-C3 3.3V output.
3. Connect DT (pin B) to an ESP32-C3 GPIO.
4. Connect CLK (pin A) to an ESP32-C3 GPIO.
5. Use `idf.py menuconfig` to configure the correct ESP32-C3 GPIOs according to the previous connections.

While not necessary to improve encoder performance due to mechanical bounce contact, two 10nF capacitors were added to improve EMI performance. It is also not convenient to use a higher value, because the edges begin to deform and shift the points of change of the irqs too far.

![alt text](images/Schematic.png)

## Prototype 
This component is part of an IOT project so I started testing it on the following development board.

![alt text](images/pcb_proto_1.png)
![alt text](images/pcb_proto_2.png)
![alt text](images/pcb_proto_3.png)

## Behavior analysis
To view IRQ events, a test PIN is connected to an oscilloscope channel.
The following images show oscillograms with typical events when the encoder is operated, for all images: channels 1 (yellow) are connected to the clock pin, channel 2 (light blue) is connected to the data pin, and channel 3 (violet) is connected to the test pin.

In the configuration menu there are two options that allow you to enable debugging of irqs events and choose the GPIO for the test pin, please use `idf.py menuconfig`.

The following image shows the one-step rotation without bouncing of the mechanical contacts.
When the clock goes down, the test channel toggle indicating that the firmware is running the isr that triggers the timer and disables the clock irq, and 1 mS later we see that the control channel toggle again indicating that the routine validated the encoder status, and since the data irq had been activated previously, a last toggle indicate that the cycle is starting again (clock irq activated and data irq deactivated).

![alt text](images/TEK_one_step_ok.png)

The following image shows the fast rotation of several steps, the smallest pulse lasts approximately 3 mS.

![alt text](images/TEK_fast_steps_ok.png)

The following image shows the false trips by the mechanical contact, it can be seen that there is no retrigger of the 1mS timer because the routine manages to read the clock pin low.

![alt text](images/TEK_bounce_1ms.png)

The following image shows the false trips by mechanical contact, you can see that there is a 1mS timer re-trigger because the routine does not read the clock pin low.

![alt text](images/TEK_timer.png)

## Brief description of the API
To use the component, you must get the handle of the instance `rotenc_handle_t` with the function` rotenc_init`, which takes as parameters the pins where the clock (A) and the data (B) are connected plus the time for the anti-bounce, if successful, returns ESP_OK.

The driver can notify the event in three different ways, by Polling, by Freertos Queue, or through function callback. Queuing and polling have no requirements, but the callback function should be used very carefully so as not to perform blocking/delaying operations, as this could affect the performance of the esp_timer component.

Note: if you use the example application you can configure the pins and the notification mode with the `idf.py menuconfig`.

To set the callback, you have to use the `rotenc_set_event_callback` function, and for the message queue `rotenc_set_event_queue` and wait with the `rotenc_wait_event function`. Both return a `rotenc_event_t` structure that contains the position fields and the detected direction of rotation.

For mechanical encoders with pushbuttons, and although this functionality can be implemented very easily with espressif idf, a callback can be configured with the `rotenc_init_button` function.

Callback Example
----------------
```c
    
    static void event_callback(rotenc_event_t event)
    {
        ESP_LOGI(TAG, "Event: position %d, direction %s", event.position,
                  event.direction ? (event.direction == ROTENC_CW ? "CW" : "CCW") : "NOT_SET")  ;
    }

    void app_main()
    {
        // Initialize the handle instance of the rotary device, 
        // by default it uses 1 mS for the debounce time.
        rotenc_handle_t handle = { 0 };
        ESP_ERROR_CHECK(rotenc_init(&handle, 
                                    CONFIG_ROT_ENC_CLK_GPIO, 
                                    CONFIG_ROT_ENC_DTA_GPIO, 
                                    CONFIG_ROT_ENC_DEBOUNCE));
        ESP_ERROR_CHECK(rotenc_set_event_callback(&handle, event_callback));

        while (1) {
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }
```

Push Button and Queue Example
-----------------------------
```c
    static void button_callback(void* arg)
    {
        rotenc_handle_t * handle = (rotenc_handle_t*) arg;
        ESP_LOGI(TAG, "Reset rotary encoder");
        ESP_ERROR_CHECK(rotenc_reset(handle));
    }

    static void event_callback(rotenc_event_t event)
    {
        ESP_LOGI(TAG, "Event: position %d, direction %s", event.position,
                  event.direction ? (event.direction == ROTENC_CW ? "CW" : "CCW") : "NOT_SET")  ;
    }

    void app_main()
    {
        // Initialize the handle instance of the rotary device, 
        // by default it uses 1 mS for the debounce time.
        rotenc_handle_t handle = { 0 };
        ESP_ERROR_CHECK(rotenc_init(&handle, 
                                    CONFIG_ROT_ENC_CLK_GPIO, 
                                    CONFIG_ROT_ENC_DTA_GPIO, 
                                    CONFIG_ROT_ENC_DEBOUNCE));
        ESP_ERROR_CHECK(rotenc_set_event_queue(&handle, 1000));
        ESP_ERROR_CHECK(rotenc_init_button(&handle, 
                                       CONFIG_ROT_ENC_BUTTON_GPIO, 
                                       CONFIG_ROT_ENC_BUTTON_DEBOUNCE, 
                                       button_callback));

        while (1) {
            rotenc_event_t event = { 0 };
            if (rotenc_wait_event(&handle, &event) == ESP_OK) {
                event_callback(event);
            }
        }
    }
```

Polling Example
----------------
```c
    static void event_callback(rotenc_event_t event)
    {
        ESP_LOGI(TAG, "Event: position %d, direction %s", event.position,
                  event.direction ? (event.direction == ROTENC_CW ? "CW" : "CCW") : "NOT_SET")  ;
    }

    void app_main()
    {
        // Initialize the handle instance of the rotary device, 
        // by default it uses 1 mS for the debounce time.
        rotenc_handle_t handle = { 0 };
        ESP_ERROR_CHECK(rotenc_init(&handle, 
                                    CONFIG_ROT_ENC_CLK_GPIO, 
                                    CONFIG_ROT_ENC_DTA_GPIO, 
                                    CONFIG_ROT_ENC_DEBOUNCE));

        while (1) {
            rotenc_event_t event = { 0 };
            ESP_ERROR_CHECK(rotenc_polling(&handle, &event));
            event_callback(event);

            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
    }
```