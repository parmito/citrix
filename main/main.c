/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <assert.h>
#include <stdint.h>
#include <string.h>
#include <sys/param.h>

#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_spiffs.h"

#include <esp_system.h>
#include <esp_spi_flash.h>
#include <rom/spi_flash.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "app_wifi.h"
#include "http_client.h"
#include "http_server.h"

#include "defines.h"
#include "UartGps.h"
#include "UartGsm.h"
#include "UartConfig.h"
#include "Debug.h"
#include "Io.h"
#include "Gps.h"
#include "Sd.h"
#include "Gsm.h"
#include "Ble.h"


/*static const char *TAG = "example";*/
void app_main()
{

#if 0
    ESP_LOGI(TAG, "Initializing SPIFFS");

    esp_vfs_spiffs_conf_t conf = {
      .base_path = "/spiffs",
      .partition_label = NULL,
      .max_files = 5,
      .format_if_mount_failed = true
    };

    // Use settings defined above to initialize and mount SPIFFS filesystem.
    // Note: esp_vfs_spiffs_register is an all-in-one convenience function.
    esp_err_t ret = esp_vfs_spiffs_register(&conf);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition");
        } else {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return;
    }

    size_t total = 0, used = 0;
    ret = esp_spiffs_info(NULL, &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    }

    // Use POSIX and C standard library functions to work with files.
    // First create a file.
    ESP_LOGI(TAG, "Opening file");
    FILE* f = fopen("/spiffs/hello.txt", "w");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return;
    }
    fprintf(f, "Hello World!\n");
    fclose(f);
    ESP_LOGI(TAG, "File written");

    // Check if destination file exists before renaming
    struct stat st;
    if (stat("/spiffs/foo.txt", &st) == 0) {
        // Delete it if it exists
        unlink("/spiffs/foo.txt");
    }

    // Rename original file
    ESP_LOGI(TAG, "Renaming file");
    if (rename("/spiffs/hello.txt", "/spiffs/foo.txt") != 0) {
        ESP_LOGE(TAG, "Rename failed");
        return;
    }

    // Open renamed file for reading
    ESP_LOGI(TAG, "Reading file");
    FILE* f = fopen("/spiffs/foo.txt", "r");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for reading");
        return;
    }
    char line[64];
    fgets(line, sizeof(line), f);
    fclose(f);
    // strip newline
    char* pos = strchr(line, '\n');
    if (pos) {
        *pos = '\0';
    }
    ESP_LOGI(TAG, "Read from file: '%s'", line);

    // All done, unmount partition and disable SPIFFS
    esp_vfs_spiffs_unregister(NULL);
    ESP_LOGI(TAG, "SPIFFS unmounted");
#endif


    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is ESP32 chip with %d CPU cores, WiFi%s%s, ",
            chip_info.cores,
            (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    printf("silicon revision %d, ", chip_info.revision);

    printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    /* Uart Gsm initialization*/
    UartGsminit();

    /* Uart Gps initialization*/
    UartGpsinit();

    /* Uart Config initialization*/
    UartConfiginit();

	xQueueSd = xQueueCreate(sdQUEUE_LENGTH,			/* The number of items the queue can hold. */
							sizeof( sMessageType ) );	/* The size of each item the queue holds. */


	xQueueGsm = xQueueCreate(gsmQUEUE_LENGTH,			/* The number of items the queue can hold. */
								sizeof( sMessageType ) );	/* The size of each item the queue holds. */

	xQueueBle = xQueueCreate(10, sizeof(sMessageType));


	xQueueHttpCli = xQueueCreate(httcliQUEUE_LENGTH,			/* The number of items the queue can hold. */
								sizeof( sMessageType ) );	/* The size of each item the queue holds. */

	xQueueHttpSrv = xQueueCreate(httsrvQUEUE_LENGTH,			/* The number of items the queue can hold. */
								sizeof( sMessageType ) );	/* The size of each item the queue holds. */

	xQueueDebug = xQueueCreate(sdQUEUE_LENGTH,			/* The number of items the queue can hold. */
							sizeof( sMessageType ) );	/* The size of each item the queue holds. */


    /* Sd initialization*/
    SdInit();

    /* Ble initialization*/
    BleInit();

    /* Debug initialization*/
    DebugInit();

    /* Gps initialization*/
    GpsInit();

    /*Http Client*/
    Http_Init();

    /* Gsm initialization*/
    GsmInit();

    /* RemoteReceiver initialization*/
    /*RemoteReceiverInit();*/

    /* Io initialization*/
    Io_Init();

    while(1)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
