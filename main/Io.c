/*
 * Io.c
 *
 *  Created on: 02/10/2018
 *      Author: danilo
 */

/* Kernel includes. */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "freertos/semphr.h"
#include "sdkconfig.h"

#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_spiffs.h"
#include "esp_sleep.h"

#include <esp_system.h>
#include <esp_spi_flash.h>
#include <rom/spi_flash.h>
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

#include "UartGsm.h"
#include "State.h"
#include "defines.h"
#include "Sd.h"
#include "Ble.h"
#include "Gsm.h"

#include "owb.h"
#include "owb_rmt.h"
#include "ds18b20.h"


#define GPIO_DS18B20_0       (GPIO_ONE_WIRE)
#define MAX_DEVICES          (8)
#define DS18B20_RESOLUTION   (DS18B20_RESOLUTION_12_BIT)
#define SAMPLE_PERIOD (1000) // milliseconds

#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          //Multisampling

//////////////////////////////////////////////
//
//
//            FUNCTION PROTOTYPES
//
//
//////////////////////////////////////////////
void vTaskIo( void *pvParameters );
void Io_Sleeping(void);

//////////////////////////////////////////////
//
//
//            VARIABLES
//
//
//////////////////////////////////////////////
OneWireBus * owb;
owb_rmt_driver_info rmt_driver_info;
OneWireBus_ROMCode device_rom_codes[MAX_DEVICES] = {0};
int num_devices = 0;
OneWireBus_SearchState search_state = {0};
bool found = false;


extern tstIo stIo;
extern tstConfiguration stConfigData;
sMessageType stDebugMsg;
static const char *IO_TASK_TAG = "IO_TASK";

/*static esp_adc_cal_characteristics_t *adc_chars;*/
static const adc_channel_t channel = ADC_CHANNEL_3;     /*GPIO39 if ADC1*/
static const adc_atten_t atten = ADC_ATTEN_DB_11;
/*static const adc_unit_t unit = ADC_UNIT_1;*/


//////////////////////////////////////////////
//
//
//              Io_Configuration
//
//
//////////////////////////////////////////////
void Io_Configuration(void)
{
	/////////////////
	// IGNITION PIN
	/////////////////
	gpio_config_t io_conf;

	//disable interrupt
	io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
	//set as output mode
	io_conf.mode = GPIO_MODE_INPUT;
	//bit mask of the pins that you want to set,e.g.GPIO18/19
	io_conf.pin_bit_mask = GPIO_INPUT_IGNITION_PIN_SEL;
	//disable pull-down mode
	io_conf.pull_down_en = 1;
	//disable pull-up mode
	io_conf.pull_up_en = 0;
	//configure GPIO with the given settings
	gpio_config(&io_conf);


    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(channel, atten);


    // Create a 1-Wire bus, using the RMT timeslot driver
    owb = owb_rmt_initialize(&rmt_driver_info, GPIO_DS18B20_0, RMT_CHANNEL_1, RMT_CHANNEL_0);
    owb_use_crc(owb, true);  // enable CRC check for ROM code

    // Find all connected devices
    printf("Find devices:\n");
    owb_search_first(owb, &search_state, &found);
    while (found)
    {
        char rom_code_s[17];
        owb_string_from_rom_code(search_state.rom_code, rom_code_s, sizeof(rom_code_s));
        printf("  %d : %s\n", num_devices, rom_code_s);
        device_rom_codes[num_devices] = search_state.rom_code;
        ++num_devices;
        owb_search_next(owb, &search_state, &found);
    }
    printf("Found %d device%s\n", num_devices, num_devices == 1 ? "" : "s");

	// In this example, if a single device is present, then the ROM code is probably
	// not very interesting, so just print it out. If there are multiple devices,
	// then it may be useful to check that a specific device is present.

	if (num_devices == 1)
	{
		// For a single device only:
		OneWireBus_ROMCode rom_code;
		owb_status status = owb_read_rom(owb, &rom_code);
		if (status == OWB_STATUS_OK)
		{
			char rom_code_s[OWB_ROM_CODE_STRING_LENGTH];
			owb_string_from_rom_code(rom_code, rom_code_s, sizeof(rom_code_s));
			printf("Single device %s present\n", rom_code_s);
		}
		else
		{
			printf("An error occurred reading ROM code: %d", status);
		}
	}
	else
	{
		// Search for a known ROM code (LSB first):
		// For example: 0x1502162ca5b2ee28
		OneWireBus_ROMCode known_device = {
			.fields.family = { 0x28 },
			.fields.serial_number = { 0xee, 0xb2, 0xa5, 0x2c, 0x16, 0x02 },
			.fields.crc = { 0x15 },
		};
		char rom_code_s[OWB_ROM_CODE_STRING_LENGTH];
		owb_string_from_rom_code(known_device, rom_code_s, sizeof(rom_code_s));
		bool is_present = false;

		owb_status search_status = owb_verify_rom(owb, known_device, &is_present);
		if (search_status == OWB_STATUS_OK)
		{
			printf("Device %s is %s\n", rom_code_s, is_present ? "present" : "not present");
		}
		else
		{
			printf("An error occurred searching for known device: %d", search_status);
		}
	}

	//    // Read temperatures from all sensors sequentially
	//    while (1)
	//    {
	//        printf("\nTemperature readings (degrees C):\n");
	//        for (int i = 0; i < num_devices; ++i)
	//        {
	//            float temp = ds18b20_get_temp(devices[i]);
	//            printf("  %d: %.3f\n", i, temp);
	//        }
	//        vTaskDelay(1000 / portTICK_PERIOD_MS);
	//    }

}
//////////////////////////////////////////////
//
//
//              Io_Sleeping
//
//
//////////////////////////////////////////////
void Io_Sleeping(void)
{
	/*const TickType_t TicksDelay = 1000;*/
    const int ext_wakeup_pin_1 = 36;
    const int ext_wakeup_pin_0  = 39;
    const uint64_t ext_wakeup_pin_1_mask = 1ULL << ext_wakeup_pin_1;


    /*gpio_set_level(GPIO_OUTPUT_GSM_ENABLE, 0);
	gpio_set_level(GPIO_OUTPUT_GPS_ENABLE, 0);
	vTaskDelay(TicksDelay);*/


	#if DEBUG_IO
	ESP_LOGI(IO_TASK_TAG,"SLEEPING\r\n");
	#endif



    printf("Enabling EXT1 wakeup on pins GPIO%d\r\n", ext_wakeup_pin_1);
    esp_sleep_enable_ext1_wakeup(ext_wakeup_pin_1_mask , ESP_EXT1_WAKEUP_ANY_HIGH );
    esp_sleep_enable_ext0_wakeup(ext_wakeup_pin_0 , 0 );

    esp_deep_sleep_start();
}


//////////////////////////////////////////////
//
//
//              Io_Init
//
//
//////////////////////////////////////////////
void Io_Init(void)
{

    xTaskCreate(vTaskIo, "vTaskIo", 1024*2, NULL, configMAX_PRIORITIES-8, NULL);
	/* Create the queue used by the queue send and queue receive tasks.
	http://www.freertos.org/a00116.html */

    Io_Configuration();
}



//////////////////////////////////////////////
//
//
//              TaskIo_ReadIo
//
//
//////////////////////////////////////////////
/*extern unsigned long u32TimeToSleep;*/

unsigned char TaskIo_ReadIo(void)
{
	unsigned char boError = true;
	static unsigned char ucCurrIgnition = 0xFF;
	static long i32EnterSleepMode =0xFFFFFFFF;
    uint32_t adc_reading = 0;
    static char cLocalBuffer[64];
    static unsigned char ucToggleBleDiag = 0;
	static unsigned long u32CurrentTime,u32PreviousTime = 0xFFFFFFFF,u32DeltaTime;

    /* Multisampling */
    for (int i = 0; i < NO_OF_SAMPLES; i++)
    {
    	adc_reading += adc1_get_raw((adc1_channel_t)channel);
    }
    adc_reading /= NO_OF_SAMPLES;

/*
2520	11
2790	12
3007	13
3366	14.3

Regression Model	Linear
R^2	0.998037925839085
Standard Error	0.076513650923424

Slope	0.0039414266813	16.144083686603
Intercept	1.0630780205943	4354.36757235425

2520	10.9954732574692
2790	12.0596584614201
3007	12.9149480512621
3366	14.3299202298486
*/

    stIo.flAdMainBatteryVoltage = (adc_reading*16.14);
    stIo.flAdMainBatteryVoltage += 4354.36;
    stIo.flAdMainBatteryVoltage /= (((1ULL<<12)));


	#if DEBUG_IO
	ESP_LOGI(IO_TASK_TAG,"AD Voltage=%d\r\n",adc_reading);
	ESP_LOGI(IO_TASK_TAG,"MainBattery Voltage=%.2f\r\n",stIo.flAdMainBatteryVoltage);
	#endif

	memset(cLocalBuffer,0,sizeof(cLocalBuffer));
	switch(ucToggleBleDiag){

			case 0:
				sprintf(cLocalBuffer,"AD,BAT=%d,%.1f\r\n",adc_reading,stIo.flAdMainBatteryVoltage);
				ucToggleBleDiag++;
			break;

			case 1:
				sprintf(cLocalBuffer,"TEMP,IGN=%.1f,%d\r\n",stIo.flI2cTemperature,stIo.ucIgnition);
				ucToggleBleDiag++;
			break;

			case 2:
				sprintf(cLocalBuffer,"SW=%s\r\n",SOFTWARE_VERSION);
				ucToggleBleDiag++;
			break;

			case 3:
				sprintf(cLocalBuffer,"SLEEP=%ld\r\n",i32EnterSleepMode*configTICK_RATE_HZ);
				ucToggleBleDiag = 0;
			break;

			default:
			break;

	}
	/*sprintf(cLocalBuffer,"A,B,T,I=%d,%.1f,%.1f,%d\r\n",adc_reading,stIo.flAdMainBatteryVoltage,stIo.flI2cTemperature,stIo.ucIgnition);*/
	/*sprintf(cLocalBuffer,"AD,BAT=%d,%.1f\r\n",adc_reading,stIo.flAdMainBatteryVoltage);*/

	stDebugMsg.ucSrc = SRC_IO;
	stDebugMsg.ucDest = SRC_BLE;
	stDebugMsg.ucEvent = (int)NULL;
	stDebugMsg.pcMessageData = &cLocalBuffer[0];

	xQueueSend(xQueueBle,( void * )&stDebugMsg,NULL);



	//    // Read temperatures from all sensors sequentially
	//    while (1)
	//    {
	//        printf("\nTemperature readings (degrees C):\n");
	//        for (int i = 0; i < num_devices; ++i)
	//        {
	//            float temp = ds18b20_get_temp(devices[i]);
	//            printf("  %d: %.3f\n", i, temp);
	//        }
	//        vTaskDelay(1000 / portTICK_PERIOD_MS);
	//    }

	// Create DS18B20 devices on the 1-Wire bus
	DS18B20_Info * devices[MAX_DEVICES] = {0};
	for (int i = 0; i < num_devices; ++i)
	{
		DS18B20_Info * ds18b20_info = ds18b20_malloc();  // heap allocation
		devices[i] = ds18b20_info;

		if (num_devices == 1)
		{
			#if DEBUG_IO
			/*ESP_LOGI(IO_TASK_TAG,"Single device optimisations enabled\n");*/
			#endif

			ds18b20_init_solo(ds18b20_info, owb);          // only one device on bus
		}
		else
		{
			ds18b20_init(ds18b20_info, owb, device_rom_codes[i]); // associate with bus and device
		}
		ds18b20_use_crc(ds18b20_info, true);           // enable CRC check for temperature readings
		ds18b20_set_resolution(ds18b20_info, DS18B20_RESOLUTION);
	}

	// Read temperatures more efficiently by starting conversions on all devices at the same time
	int errors_count[MAX_DEVICES] = {0};
	int sample_count = 0;
	if (num_devices > 0)
	{
		ds18b20_convert_all(owb);

		// In this application all devices use the same resolution,
		// so use the first device to determine the delay
		ds18b20_wait_for_conversion(devices[0]);

		// Read the results immediately after conversion otherwise it may fail
		// (using printf before reading may take too long)
		float readings[MAX_DEVICES] = { 0 };
		DS18B20_ERROR errors[MAX_DEVICES] = { 0 };

		for (int i = 0; i < num_devices; ++i)
		{
			errors[i] = ds18b20_read_temp(devices[i], &readings[i]);
		}

		// Print results in a separate loop, after all have been read
		#if DEBUG_IO
		/*ESP_LOGI(IO_TASK_TAG,"\nTemperature readings (degrees C): sample %d\n", ++sample_count);*/
		#endif

		for (int i = 0; i < num_devices; ++i)
		{
			if (errors[i] != DS18B20_OK)
			{
				++errors_count[i];
			}
			stIo.flI2cTemperature = readings[i];
			#if DEBUG_IO
			/*ESP_LOGI(IO_TASK_TAG,"  %d: %.1f    %d errors\n", i, readings[i], errors_count[i]);*/
			ESP_LOGI(IO_TASK_TAG,"Temperature=%.1f C\n", readings[i]);


			memset(cLocalBuffer,0,sizeof(cLocalBuffer));
			sprintf(cLocalBuffer,"IO:Bat,Temp,Ign=%.1f,%.1f,%d\r\n",stIo.flAdMainBatteryVoltage,stIo.flI2cTemperature,stIo.ucIgnition);

			stDebugMsg.ucSrc = SRC_IO;
			stDebugMsg.ucDest = SRC_BLE;
			stDebugMsg.ucEvent = (int)NULL;
			stDebugMsg.pcMessageData = &cLocalBuffer[0];

			xQueueSend(xQueueBle,( void * )&stDebugMsg,NULL);


			#endif
		}
	}

	// clean up dynamically allocated data
	for (int i = 0; i < num_devices; ++i)
	{
		ds18b20_free(&devices[i]);
	}
	/*owb_uninitialize(owb);*/

	if(stIo.flAdMainBatteryVoltage > 9)
	{
		if(gpio_get_level(GPIO_INPUT_IGNITION) == 1)
		{
			stIo.ucIgnition = 1;
			i32EnterSleepMode = (long)((stConfigData.u32TimeToSleepInSec)*configTICK_RATE_HZ);

			if(ucCurrIgnition != stIo.ucIgnition)
			{
				#if DEBUG_IO
				ESP_LOGI(IO_TASK_TAG,"Ignition ON\r\n");
				#endif
				ucCurrIgnition = stIo.ucIgnition;
			}
		}
		else
		{
			stIo.ucIgnition = 0;

			if(i32EnterSleepMode ==0xFFFFFFFF)
			{
				i32EnterSleepMode = (stConfigData.u32TimeToSleepInSec*configTICK_RATE_HZ);
			}

			u32CurrentTime = (unsigned long)(xTaskGetTickCount());
			#if DEBUG_IO
			ESP_LOGI(IO_TASK_TAG,"u32CurrentTime=%ld\r\n",u32CurrentTime);
			#endif
			if(u32PreviousTime != 0xFFFFFFFF){
				u32DeltaTime = u32CurrentTime - u32PreviousTime;
				u32PreviousTime = u32CurrentTime;
			}else{
				u32PreviousTime = u32CurrentTime;
			}
			#if DEBUG_IO
			ESP_LOGI(IO_TASK_TAG,"ElapsedTime=%ld\r\n",u32DeltaTime);
			#endif



			if(ucCurrIgnition != stIo.ucIgnition)
			{
				#if DEBUG_IO
				ESP_LOGI(IO_TASK_TAG,"Ignition OFF\r\n");
				#endif
				ucCurrIgnition = stIo.ucIgnition;
			}

			if(i32EnterSleepMode > 0)
			{
				i32EnterSleepMode-= u32DeltaTime;
				#if DEBUG_IO
				ESP_LOGI(IO_TASK_TAG,"Sleep=%d\r\n",(int)i32EnterSleepMode*configTICK_RATE_HZ);
				#endif
			}
			else
			{
				Io_Sleeping();
			}
		}
	}
	return(boError);
}


void vTaskIo( void *pvParameters )
{
	TickType_t elapsed_time;
	for( ;; )
	{
		elapsed_time = xTaskGetTickCount();
		TaskIo_ReadIo();

		/*vTaskDelay(1000-(after_time)/portTICK_PERIOD_MS);*/

		vTaskDelayUntil(&elapsed_time, 1000 / portTICK_PERIOD_MS);
	}
}


