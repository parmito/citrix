/*
 * UartConfig.c
 *
 *  Created on: 24/09/2018
 *      Author: danilo
 */

 /* UART asynchronous example, that uses separate RX and TX tasks

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "soc/uart_struct.h"
#include "string.h"

#include "defines.h"

static void rx_task();

extern char cConfigUartRxBuffer[RX_BUF_SIZE];
extern char *ptrRxConfig;


/*extern char *ptrRxGsm;*/
/*extern char cGsmRxBuffer[RX_BUF_SIZE];*/



#define TXD_PIN (GPIO_NUM_1)
#define RXD_PIN (GPIO_NUM_3)
//////////////////////////////////////////////
//
//
//              UartConfiginit
//
//
//////////////////////////////////////////////
void UartConfiginit(void)
{
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_0, &uart_config);
    uart_set_pin(UART_NUM_0, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_0, RX_BUF_SIZE * 2, 0, 0, NULL, 0);

    xTaskCreate(rx_task, "uart_rx_task", 1024*2, NULL, configMAX_PRIORITIES, NULL);
}

int UartConfigSendData(const char* logName, const char* data)
{
    const int len = strlen(data);
    const int txBytes = uart_write_bytes(UART_NUM_0, data, len);
    return txBytes;
}


static void rx_task()
{
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE+1);
    while (1) {
        int rxBytes = uart_read_bytes(UART_NUM_0, data, RX_BUF_SIZE, 1000 / portTICK_RATE_MS);
        if (rxBytes > 0)
        {
            data[rxBytes] = 0;

            strcpy(ptrRxConfig,(char*)data);
            if((ptrRxConfig - cConfigUartRxBuffer)<= RX_BUF_SIZE)
            {
            	ptrRxConfig += rxBytes;
            }
            else
            {
            	ptrRxConfig = cConfigUartRxBuffer;
            }

        }
    }
    free(data);
}







