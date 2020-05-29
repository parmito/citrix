/*
 * RemoteReceiver.c
 *
 *  Created on: 30/10/2018
 *      Author: danilo
 */
/* Standard includes. */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_log.h"


#include "driver/gpio.h"
#include "RemoteReceiver.h"


void RemoteReceiver_Io_Configuration_AnyEdge(void);
/**
 * Brief:
 * This test code shows how to configure gpio and how to use gpio interrupt.
 *
 * GPIO status:
 * GPIO18: output
 * GPIO19: output
 * GPIO4:  input, pulled up, interrupt from rising edge and falling edge
 * GPIO5:  input, pulled up, interrupt from rising edge.
 *
 * Test:
 * Connect GPIO18 with GPIO4
 * Connect GPIO19 with GPIO5
 * Generate pulses on GPIO18/19, that triggers interrupt on GPIO4/5
 *
 */

#define GPIO_INPUT_IO_34     34
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_IO_34))
#define ESP_INTR_FLAG_DEFAULT 0

typedef enum
{
	Silence = 0,
	Preamble,
	Address
} tenStateBits;

volatile uint8_t ready;
volatile uint32_t data;
static xQueueHandle gpio_evt_queue = NULL;

//////////////////////////////////////////////
//
//
//       	gpio_isr_handler
//
//
//////////////////////////////////////////////
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
	static tenStateBits State = Silence;
	static unsigned int dataPulseLength;
	static unsigned long lastTime = 0;
	static uint8_t u8Counter = 0;
	static unsigned long u32DataBuffer = 0;
	static uint8_t boHighEdge = true;
	/*static uint32_t u32EdgeHigh,u32EdgeLow;*/

	uint32_t now = esp_timer_get_time();
	uint32_t length = now - lastTime;
	lastTime = now;

	switch(State)
	{

		case Silence:
			  //If time at "0" is between 9200 us (23 cycles of 400us) and 13800 us (23 cycles of 600 us).
			if((length > 9200) && (length < 13800))
			{
				dataPulseLength = (length/23);
				State = Preamble;
			}
		break;

		case Preamble:
			if((length > dataPulseLength*0.5) && (length < (dataPulseLength*1.5)))
			{
				State = Address;
				u32DataBuffer = 0;
				u8Counter = 0;
				boHighEdge = true;
			}
			else
			{
			    State = Silence;
			}
		break;

		case Address:
			if(boHighEdge)
			{
				boHighEdge = false;
			}
			else
			{
				boHighEdge = true;
				if(u8Counter < 28)
				{
					if((length > 0.5*dataPulseLength) && (length < (dataPulseLength*1.5)))
					{
						u32DataBuffer = (u32DataBuffer << 1) + 1;   // add "1" on data buffer
						u8Counter++;
					}
					else
					{
						if((length >= 1.5*dataPulseLength) && (length < (dataPulseLength*2.5)))
						{
							u32DataBuffer = (u32DataBuffer << 1);       // add "0" on data buffer
							u8Counter++;
						}
						else
						{
							State = Silence;
						}
					}
				}
				else
				{
					xQueueSendFromISR(gpio_evt_queue, &u32DataBuffer, NULL);
					if(u8Counter==28)
					{
						if ((u32DataBuffer & 0x0000000F) == 5)
						{
							/*xQueueSendFromISR(gpio_evt_queue, &u32DataBuffer, NULL);*/
						}
					}
					State = Silence;
				}
			}
		break;

#if 0
		case AddressEdgeHigh:
		{
			u32EdgeHigh = now;
			State = AddressEdgeLow;
		}
		break;

		case AddressEdgeLow:
		{
			if(u8Counter < 28)
			{
				u32EdgeLow = now;
				if(u32EdgeLow > u32EdgeHigh) length = u32EdgeLow-u32EdgeHigh;
				else length = (0xFFFFFFFF-u32EdgeHigh)+u32EdgeLow;

				u8Counter++;
				State = AddressEdgeHigh;

				xQueueSendFromISR(gpio_evt_queue, &u8Counter, NULL);

				if((length > 0.5*dataPulseLength) && (length < (dataPulseLength*1.5)))
				{
					u32DataBuffer = (u32DataBuffer << 1) + 1;   // add "1" on data buffer
				}
				else
				{
					if((length > 1.5*dataPulseLength) && (length < (dataPulseLength*2.5)))
					{
						u32DataBuffer = (u32DataBuffer << 1);       // add "0" on data buffer
					}
					else
					{
						State = SilenceNegEdge;
						gpio_set_intr_type(GPIO_INPUT_IO_34, GPIO_INTR_NEGEDGE);
					}
				}
			}
			else
			{
				if(u8Counter==28)
				{
					/* Check if Anti-Code is OK (last 4 bits of buffer equal "0101")*/
#if 0
					if (((u32DataBuffer & 0x00000001) == 1) &&
						((u32DataBuffer & 0x00000002) == 0) &&
						((u32DataBuffer & 0x00000004) == 1) &&
						((u32DataBuffer & 0x00000008) == 0))
					{

					}
#endif
					State = SilenceNegEdge;
					gpio_set_intr_type(GPIO_INPUT_IO_34, GPIO_INTR_NEGEDGE);
				}
			}
		}

		break;
#endif
		default:
		break;
	}
}

//////////////////////////////////////////////
//
//
//       		vTaskRemoteReceiver
//
//
//////////////////////////////////////////////
static void vTaskRemoteReceiver(void *pvParameters)
{
    uint32_t u32DataReceived;
    for(;;)
    {
    	if(xQueueReceive(gpio_evt_queue, &u32DataReceived, 0))
    	{
            printf("DataAddress=%X\r\n",u32DataReceived);
    	}
		vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

//////////////////////////////////////////////
//
//
//    RemoteReceiver_Io_Configuration_NegEdge
//
//
//////////////////////////////////////////////
void RemoteReceiver_Io_Configuration_AnyEdge(void)
{
    gpio_config_t io_conf;

    //interrupt of rising edge
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 0;
    io_conf.pull_down_en = 0;
    gpio_config(&io_conf);


    //change gpio intrrupt type for one pin
    gpio_set_intr_type(GPIO_INPUT_IO_34, GPIO_INTR_ANYEDGE);

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_34, gpio_isr_handler, (void*) GPIO_INPUT_IO_34);

    //remove isr handler for gpio number.
    gpio_isr_handler_remove(GPIO_INPUT_IO_34);
    //hook isr handler for specific gpio pin again
    gpio_isr_handler_add(GPIO_INPUT_IO_34, gpio_isr_handler, (void*) GPIO_INPUT_IO_34);


}

//////////////////////////////////////////////
//
//
//            RemoteReceiverInit
//
//
//////////////////////////////////////////////
void RemoteReceiverInit(void)
{
	//create a queue to handle gpio event from isr
	gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
	//start gpio task
	xTaskCreate(vTaskRemoteReceiver, "vTaskRemoteReceiver", 1024*4, NULL, configMAX_PRIORITIES-3, NULL);

	RemoteReceiver_Io_Configuration_AnyEdge();
}







