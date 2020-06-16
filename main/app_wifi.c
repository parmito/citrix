/* ESP HTTP Client Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/event_groups.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "app_wifi.h"
#include "http_server.h"


#include "string.h"
#include "defines.h"

static const char *TAG = "WLAN";
extern tstConfiguration stConfigData;


/* FreeRTOS event group to signal when we are connected & ready to make a request */
static EventGroupHandle_t wifi_event_group;

/* The event group allows multiple bits for each event,
   but we only care about one event - are we connected
   to the AP with an IP? */
const int CONNECTED_BIT = BIT0;
const int DISCONNECTED_BIT = BIT1;

#if 0
static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch (event->event_id) {
        case SYSTEM_EVENT_STA_START:
            esp_wifi_connect();
            break;
        case SYSTEM_EVENT_STA_GOT_IP:
            xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
            xEventGroupClearBits(wifi_event_group, DISCONNECTED_BIT);
            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
            /* This is a workaround as ESP32 WiFi libs don't currently
               auto-reassociate. */
            /*esp_wifi_connect();*/
            xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
            xEventGroupSetBits(wifi_event_group, DISCONNECTED_BIT);
            break;

        default:
            break;
    }
    return ESP_OK;
}
#endif

static void ip_event_handler(void* arg, esp_event_base_t event_base,
        						int32_t event_id, void* event_data)
{
	httpd_handle_t* server = (httpd_handle_t*) arg;

    switch (event_id) {
        case IP_EVENT_AP_STAIPASSIGNED:

                if (*server == NULL) {
                    ESP_LOGI(TAG, "Starting webserver");
                    *server = start_webserver();
                }
            break;
        case IP_EVENT_STA_LOST_IP:
            /* This is a workaround as ESP32 WiFi libs don't currently
               auto-reassociate. */

            break;

        default:
            break;
    }
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                    int32_t event_id, void* event_data)
{

    switch (event_id) {
        case WIFI_EVENT_STA_START:
            esp_wifi_connect();
            break;
        case WIFI_EVENT_STA_CONNECTED:
            xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
            xEventGroupClearBits(wifi_event_group, DISCONNECTED_BIT);
            break;
        case WIFI_EVENT_STA_DISCONNECTED:
            /* This is a workaround as ESP32 WiFi libs don't currently
               auto-reassociate. */
            /*esp_wifi_connect();*/
            xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
            xEventGroupSetBits(wifi_event_group, DISCONNECTED_BIT);
            break;

        default:
            break;
    }

    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" join, AID=%d",
                 MAC2STR(event->mac), event->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" leave, AID=%d",
                 MAC2STR(event->mac), event->aid);
    }
}

void app_wifi_initialise(void)
{
	static httpd_handle_t server = NULL;

   /*wifi_config_t wifi_config;*/

   wifi_config_t wifi_config = {
          .sta = {
              .ssid = CONFIG_WIFI_SSID,
              .password = CONFIG_WIFI_PASSWORD,
          },
      };

   wifi_config_t wifi_configAP = {
		   .ap = {
              .ssid = "CITRIX",
              .password = "poliana90",
			  .max_connection = (uint8_t)(4),
			  .authmode = 3
          },
      };

    memset(wifi_config.sta.ssid,0x00,sizeof(wifi_config.sta.ssid));
    memset(wifi_config.sta.password,0x00,sizeof(wifi_config.sta.password));

    memcpy((char*)&wifi_config.sta.ssid,(const char*)&stConfigData.cWifiApName,sizeof(stConfigData.cWifiApName));
    memcpy((char*)&wifi_config.sta.password,(const char*)&stConfigData.cWifiApPassword,sizeof(stConfigData.cWifiApPassword));


    tcpip_adapter_init();

    /*const tcpip_adapter_ip_info_t ip_info = {
    	.ip = {((u32_t)0xc0a80002UL)},
		.netmask = {((u32_t)0xffffff00UL)},
		.gw = {((u32_t)0xc0a80001UL)},
    };

	(void)tcpip_adapter_set_ip_info(TCPIP_ADAPTER_IF_AP, &ip_info);*/


    wifi_event_group = xEventGroupCreate();
    /*ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));*/
    ESP_ERROR_CHECK(esp_event_loop_create_default());


    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, ESP_EVENT_ANY_ID, &ip_event_handler, &server));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, &server));
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));


    ESP_LOGI(TAG, "WiFi configuration:%s,%s", wifi_config.sta.ssid,wifi_config.sta.password);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_configAP));

    ESP_ERROR_CHECK(esp_wifi_start());

    /* Start the server for the first time */
    server = start_webserver();


}

void app_wifi_wait_connected()
{
    xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT , false, true, 10000 / portTICK_PERIOD_MS);
}

uint32_t app_wifi_get_status(void)
{
	uint32_t EventBits_t;
	EventBits_t =  xEventGroupGetBits(wifi_event_group);
	return(EventBits_t);
}

void app_wifi_clear_connected_bit(void)
{
	xEventGroupClearBits( wifi_event_group, CONNECTED_BIT );
}

void app_wifi_clear_disconnected_bit(void)
{
	xEventGroupClearBits( wifi_event_group, DISCONNECTED_BIT );
}

