/*
 * gps.c
 *
 *  Created on: Oct 16, 2017
 *      Author: Danilo_2
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
#include "esp_system.h"
#include "esp_log.h"
#include "driver/gpio.h"

#include "defines.h"
#include "tinygps.h"

#include "Sd.h"
#include "Debug.h"
#include "Ble.h"
#include "Gps.h"

void vTaskGps( void *pvParameters );


char cGpsDataCopiedToSd[RX_BUF_SIZE];
char szFilenameToBeWritten[RX_BUF_SIZE];

sMessageType stGpsMsg;
tstIo stIo = {
	{0},/*unsigned char ucIgnition;*/
	{0},/*unsigned int u16AdMainBatteryVoltage;*/
	{0},/*float flAdMainBatteryVoltage;*/
	{0},/*unsigned int u16AdBackupBatteryVoltage;*/
	{0},/*float flAdBackupBatteryVoltage;*/
	{0},/*float flI2cTemperature;*/
	{0},/*unsigned int u16IntTemperature;*/
	{{0}},/*char cLatitude[20];*/
	{{0}},/*char cLongitude[20];*/
	{0},/*unsigned char ucNumSat;*/
	{0},/*unsigned char ulHdop;*/
	{0},/*int altitude;*/
	{0},/*int course;*/
	{0},/*int speedkmh;*/
	{0},/*time_t timeSinceEpoch;*/
	{0}/*struct tm t;*/
};

static const char *GPS_TASK_TAG = "GPS_TASK";
extern tstConfiguration stConfigData;
unsigned long u32QtyFilesStored;

static unsigned char mac[6];

#define MAX_PRECISION	(10)
static const double rounders[MAX_PRECISION + 1] =
{
	0.5,				// 0
	0.05,				// 1
	0.005,				// 2
	0.0005,				// 3
	0.00005,			// 4
	0.000005,			// 5
	0.0000005,			// 6
	0.00000005,			// 7
	0.000000005,		// 8
	0.0000000005,		// 9
	0.00000000005		// 10
};

char * ftoa(double f, char * buf, int precision)
{
	char * ptr = buf;
	char * p = ptr;
	char * p1;
	char c;
	long intPart;

	// check precision bounds
	if (precision > MAX_PRECISION)
		precision = MAX_PRECISION;

	// sign stuff
	if (f < 0)
	{
		f = -f;
		*ptr++ = '-';
	}

	if (precision < 0)  // negative precision == automatic precision guess
	{
		if (f < 1.0) precision = 6;
		else if (f < 10.0) precision = 5;
		else if (f < 100.0) precision = 4;
		else if (f < 1000.0) precision = 3;
		else if (f < 10000.0) precision = 2;
		else if (f < 100000.0) precision = 1;
		else precision = 0;
	}

	// round value according the precision
	if (precision)
		f += rounders[precision];

	// integer part...
	intPart = f;
	f -= intPart;

	if (!intPart)
		*ptr++ = '0';
	else
	{
		// save start pointer
		p = ptr;

		// convert (reverse order)
		while (intPart)
		{
			*p++ = '0' + intPart % 10;
			intPart /= 10;
		}

		// save end pos
		p1 = p;

		// reverse result
		while (p > ptr)
		{
			c = *--p;
			*p = *ptr;
			*ptr++ = c;
		}

		// restore end pos
		ptr = p1;
	}

	// decimal part
	if (precision)
	{
		// place decimal point
		*ptr++ = '.';

		// convert
		while (precision--)
		{
			f *= 10.0;
			c = f;
			*ptr++ = '0' + c;
			f -= c;
		}
	}

	// terminating zero
	*ptr = 0;

	return buf;
}

//////////////////////////////////////////////
//
//
//              Gps_On
//
//
//////////////////////////////////////////////
void Gps_On(void)
{
	gpio_set_level(GPIO_OUTPUT_GPS_ENABLE, 1);
}

//////////////////////////////////////////////
//
//
//              Gps_Off
//
//
//////////////////////////////////////////////
void Gps_Off(void)
{
	gpio_set_level(GPIO_OUTPUT_GPS_ENABLE, 0);
}

//////////////////////////////////////////////
//
//
//              Gps_Io_Configuration
//
//
//////////////////////////////////////////////
void Gps_Io_Configuration(void)
{
	gpio_config_t io_conf;
	//disable interrupt
	io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
	//set as output mode
	io_conf.mode = GPIO_MODE_OUTPUT;
	//bit mask of the pins that you want to set,e.g.GPIO18/19
	io_conf.pin_bit_mask = GPIO_OUTPUT_GPS_ENABLE_PIN_SEL ;
	//disable pull-down mode
	io_conf.pull_down_en = 0;
	//disable pull-up mode
	io_conf.pull_up_en = 0;
	//configure GPIO with the given settings
	gpio_config(&io_conf);

	Gps_On();
}
//////////////////////////////////////////////
//
//
//              GpsInit
//
//
//////////////////////////////////////////////
void GpsInit(void)
{
	ESP_LOGI(GPS_TASK_TAG, "GPS INIT");

	Gps_Io_Configuration();

    xTaskCreate(vTaskGps, "vTaskGps", 1024*2, NULL, configMAX_PRIORITIES-5, NULL);

	(void)esp_efuse_mac_get_default(&mac[0]);
	/*ESP_LOGI(GPS_TASK_TAG, "MAC=%x:%x:%x:%x:%x:%x",mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);*/
}


void vTaskGps( void *pvParameters )
{
	static int year;
	static byte month,day,hour,minute,second,hundredths;
	static unsigned long age;
	unsigned char ucFixData;
    static unsigned long u32PeriodGpsWriteSd = 0;
	/*struct tm t;*/

	char cfAdMainBatteryVoltage[20];
    /*char* cLocalBuffer = (char*) malloc(RX_BUF_SIZE+1);*/

    static unsigned char ucCurrIgnition = 0xFF;
    static bool boGpsHasFixedOnce = false;
    static char cLocalBuffer[64];
    /*esp_log_level_set(GPS_TASK_TAG, ESP_LOG_INFO);*/


	/*stGpsMsg.ucSrc = SRC_GPS;
	stGpsMsg.ucDest = SRC_DEBUG;
	stGpsMsg.ucEvent = EVENT_IO_GPS_SEARCHING;
	xQueueSend( xQueueDebug, ( void * )&stGpsMsg, 0);*/

    /* Open or create a log file and ready to append */

	for( ;; )
	{
	    /*ESP_LOGI(GPS_TASK_TAG, "Running...");*/

		ucFixData = gps_fix_data();
		if( ucFixData == 1)
		{
			ESP_LOGI(GPS_TASK_TAG, "<<<<FIXING>>>>\r\n");

			stGpsMsg.ucSrc = SRC_GPS;
			stGpsMsg.ucDest = SRC_DEBUG;
			stGpsMsg.ucEvent = EVENT_IO_GPS_FIXED;
			xQueueSend( xQueueDebug, ( void * )&stGpsMsg, 0);

			boGpsHasFixedOnce = true;
			/*stGpsMsg.ucSrc = SRC_GPS;
			stGpsMsg.ucDest = SRC_DEBUG;
			stGpsMsg.ucEvent = EVENT_IO_GPS_ACQUIRING;
			xQueueSend( xQueueDebug, ( void * )&stGpsMsg, 0);*/

			/*************************/
			/*	NUMBER OF SATELLITES */
			/*************************/
			stIo.ucNumSat = (unsigned char)(gps_num_sats());

			/*************************/
			/*	 		HDOP		 */
			/*************************/
			stIo.ulHdop = (unsigned char)(gps_num_hdop());

			gps_crack_datetime(&year,&month,&day,\
								&hour,&minute,&second,&hundredths,&age);

			/*************************/
			/*	 	  ALTITUDE	     */
			/*************************/
			stIo.altitude = (int)gps_f_altitude();
			/*************************/
			/*	 		COURSE		 */
			/*************************/
			stIo.course = (int)gps_f_course();
			/*************************/
			/*	 		SPEED		 */
			/*************************/
			stIo.speedkmh =(int)gps_f_speed_kmph();

			float flat;
			float flon;

			gps_f_get_position(&flat,&flon,&age);

			memset(stIo.cLatitude,0,sizeof(stIo.cLatitude));
			memset(stIo.cLongitude,0,sizeof(stIo.cLongitude));

			/*************************/
			/*	 	LAT & LONG	 	 */
			/*************************/
			(void)ftoa((double)flat,(char*)&stIo.cLatitude, 8);
			(void)ftoa((double)flon,(char*)&stIo.cLongitude, 8);
			/* Print as parts, note that you need 0-padding for fractional bit.
			Since d1 is 678 and d2 is 123, you get "678.0123".*/


			/******************************/
			/*Epoch Time since 1970 in HEX*/
			/******************************/
			stIo.t.tm_year = year-1900;/* because of mktime*/
			stIo.t.tm_mon = month-1;/* because of mktime which makes Jan = 0*/
			stIo.t.tm_mday = day;
			stIo.t.tm_hour = hour;
			stIo.t.tm_min = minute;
			stIo.t.tm_sec = second;
			stIo.timeSinceEpoch = mktime(&stIo.t);

			/*************************/
			/* MAIN BATTERY VOLTAGE  */
			/*************************/
			(void)ftoa((double)stIo.flAdMainBatteryVoltage, (char*)&cfAdMainBatteryVoltage, 2);

			memset(szFilenameToBeWritten,0x00,sizeof(szFilenameToBeWritten));
			sprintf(szFilenameToBeWritten,"/spiffs/DATA_%02d%02d%02d%02d.TXT",stIo.t.tm_year-100,(stIo.t.tm_mon+1),stIo.t.tm_mday,stIo.t.tm_hour);

			ESP_LOGI(GPS_TASK_TAG, "%s", szFilenameToBeWritten);

			if(stIo.flAdMainBatteryVoltage > 9)
			{
				memset(cGpsDataCopiedToSd,0,RX_BUF_SIZE);
				sprintf(cGpsDataCopiedToSd,"S=%02X%02X%02X%02X%02X%02X,%d,%d,%s,%s,%d,%d,%d,%d,%X,%s,%.1f\r\n",mac[0],mac[1],mac[2],mac[3],mac[4],mac[5],stIo.ucNumSat,(int)stIo.ulHdop,stIo.cLatitude,stIo.cLongitude,stIo.altitude,stIo.course,stIo.speedkmh,stIo.ucIgnition,(unsigned int)stIo.timeSinceEpoch,cfAdMainBatteryVoltage,stIo.flI2cTemperature);
				/*ESP_LOGI(GPS_TASK_TAG, "\r\n<<<<GPS TASK>>>>\r\n%s\r\n<<<<>>>>\r\n",cGpsDataCopiedToSd);*/

				ESP_LOGI(GPS_TASK_TAG, "BATTERY VOLTAGE > 9\r\n");
				/* Ignition was ON and now is OFF*/
				if(ucCurrIgnition != stIo.ucIgnition)
				{
					ucCurrIgnition = stIo.ucIgnition;

					memset(cLocalBuffer,0,sizeof(cLocalBuffer));
					strcpy(cLocalBuffer,(const char*)("GPS:Ign Changed\r\n"));

					ESP_LOGI(GPS_TASK_TAG, "IGNITION CHANGED\r\n");

#if SRC_BLE
					stGpsMsg.ucSrc = SRC_GPS;
					stGpsMsg.ucDest = SRC_BLE;
					stGpsMsg.ucEvent = (int)NULL;
					stGpsMsg.pcMessageData = &cLocalBuffer[0];
					xQueueSend(xQueueBle,( void * )&stGpsMsg,0);
#endif
					stGpsMsg.ucSrc = SRC_GPS;
					stGpsMsg.ucDest = SRC_SD;
					stGpsMsg.ucEvent = EVENT_SD_WRITING;
					xQueueSend( xQueueSd, ( void * )&stGpsMsg, 0);

				}
				else
				{
					/* IGN was OFF and now is ON*/
					ucCurrIgnition = stIo.ucIgnition;

					if(++u32PeriodGpsWriteSd >= stConfigData.u32PeriodLogInSec)
					{
						u32PeriodGpsWriteSd = 0;

						if(stIo.ucIgnition == 1)
						{
							ESP_LOGI(GPS_TASK_TAG, "LOGGING...\r\n");

							memset(cLocalBuffer,0,sizeof(cLocalBuffer));
							strcpy(cLocalBuffer,(const char*)("GPS:Periodic\r\n"));

#if SRC_BLE
							stGpsMsg.ucSrc = SRC_GPS;
							stGpsMsg.ucDest = SRC_BLE;
							stGpsMsg.ucEvent = (int)NULL;
							stGpsMsg.pcMessageData = &cLocalBuffer[0];
							xQueueSend(xQueueBle,( void * )&stGpsMsg,0);
#endif
							stGpsMsg.ucSrc = SRC_GPS;
							stGpsMsg.ucDest = SRC_SD;
							stGpsMsg.ucEvent = EVENT_SD_WRITING;
							xQueueSend( xQueueSd, ( void * )&stGpsMsg, 0);

						}
					}
				}
			}
			else
			{
				if(++u32PeriodGpsWriteSd >= stConfigData.u32PeriodLogInSec)
				{
					u32PeriodGpsWriteSd = 0;

					ESP_LOGI(GPS_TASK_TAG, "ATHLETE MODE\r\n");

					memset(cGpsDataCopiedToSd,0,RX_BUF_SIZE);
					sprintf(cGpsDataCopiedToSd,"S=%02X%02X%02X%02X%02X%02X,%d,%d,%s,%s,%d,%d,%d,1,%X,%s,%.1f\r\n",mac[0],mac[1],mac[2],mac[3],mac[4],mac[5],stIo.ucNumSat,(int)stIo.ulHdop,stIo.cLatitude,stIo.cLongitude,stIo.altitude,stIo.course,stIo.speedkmh,(unsigned int)stIo.timeSinceEpoch,cfAdMainBatteryVoltage,stIo.flI2cTemperature);

					memset(cLocalBuffer,0,sizeof(cLocalBuffer));
					strcpy(cLocalBuffer,(const char*)("GPS:Periodic\r\n"));
#if SRC_BLE
					stGpsMsg.ucSrc = SRC_GPS;
					stGpsMsg.ucDest = SRC_BLE;
					stGpsMsg.ucEvent = (int)NULL;
					stGpsMsg.pcMessageData = &cLocalBuffer[0];
					xQueueSend(xQueueBle,( void * )&stGpsMsg,0);
#endif
					stGpsMsg.ucSrc = SRC_GPS;
					stGpsMsg.ucDest = SRC_SD;
					stGpsMsg.ucEvent = EVENT_SD_WRITING;
					xQueueSend( xQueueSd, ( void * )&stGpsMsg, 0);

				}
			}
		}
		else
		{
			ESP_LOGI(GPS_TASK_TAG, "<<<<NOT FIXING>>>>\r\n");

			memset(cLocalBuffer,0,sizeof(cLocalBuffer));
			strcpy(cLocalBuffer,(const char*)("GPS:NOT FIXING\r\n"));
#if SRC_BLE
			stGpsMsg.ucSrc = SRC_GPS;
			stGpsMsg.ucDest = SRC_BLE;
			stGpsMsg.ucEvent = (int)NULL;
			stGpsMsg.pcMessageData = &cLocalBuffer[0];
			xQueueSend(xQueueBle,( void * )&stGpsMsg,NULL);
#endif
			stGpsMsg.ucSrc = SRC_GPS;
			stGpsMsg.ucDest = SRC_DEBUG;
			stGpsMsg.ucEvent = EVENT_IO_GPS_NOT_FIXED;
			xQueueSend( xQueueDebug, ( void * )&stGpsMsg, 0);


			if(	(boGpsHasFixedOnce != false) &&\
				(ucCurrIgnition != stIo.ucIgnition) &&\
				(stIo.ucIgnition == 0)	)
			{

				strcat(cLocalBuffer,(const char*)("GPS:LOG NO GPS\r\n"));
#if SRC_BLE
				stGpsMsg.ucSrc = SRC_GPS;
				stGpsMsg.ucDest = SRC_BLE;
				stGpsMsg.ucEvent = (int)NULL;
				stGpsMsg.pcMessageData = &cLocalBuffer[0];
				xQueueSend(xQueueBle,( void * )&stGpsMsg,NULL);
#endif
				memset(cGpsDataCopiedToSd,0,RX_BUF_SIZE);
				sprintf(cGpsDataCopiedToSd,"S=%02X%02X%02X%02X%02X%02X,%d,%d,%s,%s,%d,%d,%d,%d,%X,%s,%.1f\r\n",mac[0],mac[1],mac[2],mac[3],mac[4],mac[5],stIo.ucNumSat,(int)stIo.ulHdop,stIo.cLatitude,stIo.cLongitude,stIo.altitude,stIo.course,stIo.speedkmh,stIo.ucIgnition,(unsigned int)stIo.timeSinceEpoch,cfAdMainBatteryVoltage,stIo.flI2cTemperature);

				stGpsMsg.ucSrc = SRC_GPS;
				stGpsMsg.ucDest = SRC_SD;
				stGpsMsg.ucEvent = EVENT_SD_WRITING;
				xQueueSend( xQueueSd, ( void * )&stGpsMsg, 0);
			}

			ucCurrIgnition = stIo.ucIgnition;


			/*stGpsMsg.ucSrc = SRC_GPS;
			stGpsMsg.ucDest = SRC_DEBUG;
			stGpsMsg.ucEvent = EVENT_IO_GPS_SEARCHING;
			xQueueSend( xQueueDebug, ( void * )&stGpsMsg, 0);*/

#if 0
			int d5 = stIo.flAdMainBatteryVoltage;            // Get the integer part (678).
			float f1 = stIo.flAdMainBatteryVoltage - d5;     // Get fractional part (678.0123 - 678 = 0.0123).
			int d6 = (int)(f1 * 100); // Or this one: Turn into integer.
			if(d6 <0 ) d6*=-1;

			int d1 = stIo.flAdBackupBatteryVoltage;            // Get the integer part (678).
			f1 = stIo.flAdBackupBatteryVoltage - d1;     // Get fractional part (678.0123 - 678 = 0.0123).
			int d2 = (int)(f1 * 100); // Or this one: Turn into integer.
			if(d2 <0 ) d2*=-1;
#endif
		}
		vTaskDelay(1000/portTICK_PERIOD_MS);
	}
}
