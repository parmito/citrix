/* ESP HTTP Client Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#ifndef _APP_WIFI_H_
#define _APP_WIFI_H_

#include "esp_wifi.h"

#define CONFIG_WIFI_SSID	"kkkkkk"
#define CONFIG_WIFI_PASSWORD "jjjjjj"

void app_wifi_initialise(void);
void app_wifi_wait_connected();
uint32_t app_wifi_get_status(void);
void app_wifi_clear_connected_bit(void);
void app_wifi_clear_disconnected_bit(void);


#endif
