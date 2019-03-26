/* ESP32_Functional_Test
   Written by Chris Taylor, 3/26/19

   A small program that utilizes an RTOS on the ESP32 that controls an LED's brightness with two user inputs.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

// General Includes
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_err.h"

// GPIO Includes
#include "driver/gpio.h"

// LEDC Includes
#include "driver/ledc.h"


/**
 * Brief:
 * This code intialzies two GPIO inputs to be used with buttons connected to ground.
 * The code waits for a GPIO interrupt, then fades the LED up or down based on which
 * button was pressed. 
 *
 * Pin definitions:
 * 
 * GPIO4:  input, pulled up, interrupt from falling edge. Fades LED up.
 * GPIO5:  input, pulled up, interrupt from falling edge. Fades LED down. 
 *
 * GPIO18: led PWM output. 
 */


// GPIO Defines
#define GPIO_INPUT_IO_0     	4
#define GPIO_INPUT_IO_1     	5
#define GPIO_INPUT_PIN_SEL  	((1ULL<<GPIO_INPUT_IO_0) | (1ULL<<GPIO_INPUT_IO_1))
#define ESP_INTR_FLAG_DEFAULT 	0

// LED Defines
#define LEDC_HS_TIMER			LEDC_TIMER_0
#define LEDC_HS_MODE			LEDC_HIGH_SPEED_MODE
#define LEDC_HS_CH0_GPIO 		(18)
#define LEDC_HS_CH0_CHANNEL		LEDC_CHANNEL_0

#define LEDC_TEST_DUTY			(4000)
#define LEDC_TEST_FADE_TIME		(3000)

// Event queue for GPIO interrupts
static xQueueHandle gpio_evt_queue = NULL;

// GPIO ISR Handler
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void button_press_task(void* arg)
{
	uint32_t io_num;
	uint32_t fade_value;
	
	// Task waits indefinitely for xQueueReceive
	for(;;) {
		// Wait on button press, block indefinitely 
		if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
			
			// Debounce	delay for 10ms
			vTaskDelay(10 / portTICK_RATE_MS);
			
			// Check which button was pressed
			if(io_num == GPIO_INPUT_IO_0) { fade_value = LEDC_TEST_DUTY; }
			else { fade_value = 0; }
            
			// Change brightness
			ledc_set_fade_with_time(LEDC_HS_MODE, LEDC_HS_CH0_CHANNEL, fade_value, LEDC_TEST_FADE_TIME);
			// Start fade
			ledc_fade_start(LEDC_HS_MODE, LEDC_HS_CH0_CHANNEL, LEDC_FADE_NO_WAIT);
		
			printf("GPIO[%d] pressed.", io_num);
		}
	}
}

void app_main()
{
	// ********************************************************************
	// LEDC CONFIGURATION
	// ********************************************************************
	
	// Configure timer
	ledc_timer_config_t ledc_timer = {
		.duty_resolution = LEDC_TIMER_13_BIT, 	// resolution of PWM duty
		.freq_hz = 5000,						// frequency of PWM signal
		.speed_mode = LEDC_HS_MODE,				// timer mode
		.timer_num = LEDC_HS_TIMER				// timer index
	};
	
	// Set configuration of timer0 for high speed
	ledc_timer_config(&ledc_timer);

	// Prepare channel configuration of LED controller
	ledc_channel_config_t ledc_channel = {
			.channel 	= LEDC_HS_CH0_CHANNEL,
			.duty		= 0,
			.gpio_num	= LEDC_HS_CH0_GPIO,
			.speed_mode	= LEDC_HS_MODE,
			.timer_sel	= LEDC_HS_TIMER
	};
	
	// Initialize LED controller with configuration
	ledc_channel_config(&ledc_channel);

	// Initialize fade service
	ledc_fade_func_install(0);


	// ********************************************************************
   	// GPIO CONFIGURATION
	// ********************************************************************
	
	gpio_config_t io_conf;

    //interrupt of falling edge
    io_conf.intr_type = GPIO_PIN_INTR_NEGEDGE;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    //start gpio task
    xTaskCreate(button_press_task, "button_press_task", 2048, NULL, 10, NULL);

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void*) GPIO_INPUT_IO_0);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_1, gpio_isr_handler, (void*) GPIO_INPUT_IO_1);

    
	printf("Running...");
    
	// Wait for gpio interrupt
	while(1) {
		printf("Waiting for button press...");
		vTaskDelay(1000 / portTICK_RATE_MS);
    }
}
