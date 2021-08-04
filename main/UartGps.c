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
#include "tinygps.h"

#include "UartGps.h"
static void rx_task();

static const int RX_BUF_SIZE = 512;

#define TXD_PIN (GPIO_NUM_17)
#define RXD_PIN (GPIO_NUM_16)
static const char *UART_GPS_TASK_TAG = "UART_GPS_TASK";

//////////////////////////////////////////////
//
//
//              GpsInit
//
//
//////////////////////////////////////////////
void UartGpsinit(void)
{
    const uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_2, &uart_config);
    uart_set_pin(UART_NUM_2, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_2, RX_BUF_SIZE * 2, 0, 0, NULL, 0);

    ESP_LOGI(UART_GPS_TASK_TAG,"UART GPS INIT");

    xTaskCreate(rx_task, "uart_rx_task", 1024*2, NULL, configMAX_PRIORITIES-1, NULL);
}

static void rx_task()
{
    /*static const char *UART_GPS_TASK_TAG = "UART_GPS_TASK";
    esp_log_level_set(UART_GPS_TASK_TAG, ESP_LOG_INFO);*/
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE+1);
    while (1) {
        ESP_LOGI(UART_GPS_TASK_TAG,"UART GPS TASK");
        int rxBytes = uart_read_bytes(UART_NUM_2, data, RX_BUF_SIZE, 1000 / portTICK_RATE_MS);
        if (rxBytes > 0) {
            data[rxBytes] = 0;

            ESP_LOGI(UART_GPS_TASK_TAG,"%s\r\n",data);
            unsigned int u16 = 0;
            while(rxBytes > 0)
            {
            	gps_encode((char)data[u16]);
            	u16++;
            	rxBytes--;
            }
        }
    }
    free(data);
}




