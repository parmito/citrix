/*
 * Sd.c
 *
 *  Created on: 20/09/2018
 *      Author: danilo
 */
#include <stdio.h>
#include <assert.h>
#include <stdint.h>
#include <string.h>
#include <sys/param.h>
#include <sys/dirent.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_system.h"

#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_spiffs.h"

#include <esp_system.h>
#include <esp_spi_flash.h>
#include <rom/spi_flash.h>
#include <ff.h>

#include "defines.h"
#include "State.h"
#include "Gsm.h"
#include "Sd.h"
#include "Ble.h"
#include "http_client.h"

//////////////////////////////////////////////
//
//
//            FUNCTION PROTOTYPES
//
//
//////////////////////////////////////////////
void vTaskSd( void *pvParameters );
unsigned char TaskSd_Writing(sMessageType *psMessage);
unsigned char TaskSd_IgnoreEvent(sMessageType *psMessage);
unsigned char ucSdWriteWifiApName(char* a);
unsigned char ucSdAppSource(char* a);
unsigned char ucSdWriteWifiApPassword(char* a);
unsigned char ucSdWritePeriodLogInSec(char* a);
unsigned char ucSdWriteModemPeriodTxInSec(char* a);
unsigned char ucSdWriteModemApn(char* a);
unsigned char ucSdWriteModemApnLogin(char* a);
unsigned char ucSdWriteModemApnPassword(char* a);
unsigned char ucSdWriteTimeToSleep(char* a);
unsigned char ucSdDoNothing(char* a);

//////////////////////////////////////////////
//
//
//              VARIABLES
//
//
//////////////////////////////////////////////
unsigned long u32TimeToSleep;
unsigned long u32TimeToWakeup;

tstConfiguration stConfigData =
{
	.u32TimeToSleepInSec = (unsigned long)180,
	.u32PeriodLogInSec = (unsigned long)30,
	.cBuzzerStatus = "OFF",
	.cWifiApName= "Iphone4_EXT",
	.cWifiApPassword = "poliana90",
	.cHttpDomain = "gpslogger.esy.es",
	.cPageUrl = "http://gpslogger.esy.es/pages/upload/index.php",
	.cModemApn = "vodafone.br",
	.cModemApnLogin = " ",
	.cModemApnPassword = " ",
	.u32ModemPeriodTxInSec = (unsigned long)120,
	.cAppSource ="SRC_WIFI"
};
const char cConfigText[]= {"CONFIG/CONFIG.txt"};

char szFilenameToBeRead[RX_BUF_SIZE+1];
char  cConfigAndData[RX_BUF_SIZE+1];
extern char szFilenameToBeWritten[RX_BUF_SIZE+1];
extern char cGpsDataCopiedToSd[RX_BUF_SIZE+1];
extern unsigned char ucCurrentStateGsm;

static long int liFilePointerPositionBeforeReading = 0;
static long int liFilePointerPositionAfterReading = 0;

char cConfigRxBuffer[RX_BUF_SIZE];
char *ptrRxConfig;


/*extern tstIo stIo;*/
sMessageType stSdMsg;

static const char *SD_TASK_TAG = "SD_TASK";

typedef struct{
    unsigned char ucIndex;
    char cIdentifier[RX_BUF_SIZE_REDUCED];
    char cParam1[RX_BUF_SIZE_REDUCED];
    unsigned char (*ucFuncPtr)(char *a);
}tstReadWriteDataByIdentifier;

tstReadWriteDataByIdentifier const astReadWriteTable[] =
{
    { 1, "CONF:","WIFIAPNAME:",ucSdWriteWifiApName},
    { 2, "CONF:","WIFIAPPASSWORD:",ucSdWriteWifiApPassword},
	{ 3, "CONF:","PERIODLOGINSEC:",ucSdWritePeriodLogInSec},
	{ 4, "CONF:","MODEMPERIODTXINSEC:",ucSdWriteModemPeriodTxInSec},
	{ 5, "CONF:","APPSOURCE:",ucSdAppSource},
	{ 6, "CONF:","MODEMAPN:",ucSdWriteModemApn},
	{ 7, "CONF:","MODEMAPNLOGIN:",ucSdWriteModemApnLogin},
	{ 8, "CONF:","MODEMAPNPASSWORD:",ucSdWriteModemApnPassword},
	{ 9, "CONF:","TIMETOSLEEP:",ucSdWriteTimeToSleep},
    { 255, "END OF ARRAY:","END OF ARRAY",ucSdDoNothing}
};


//////////////////////////////////////////////
//
//
//  SearchReadWriteDataByIdentifierFunctions
//
//
//////////////////////////////////////////////
unsigned char SearchReadWriteDataByIdentifierFunctions(char *data, tstReadWriteDataByIdentifier const *pst)
{
    char *ptr;
    unsigned char ucResp = false;
    
    ESP_LOGI(SD_TASK_TAG,"\r\nSearchReadWriteDataByIdentifierFunctions\r\n");

    while(pst->ucIndex != 255)
    {
        ptr = strstr(data,pst->cIdentifier);
        if(ptr != NULL)
        {
        	/*ESP_LOGI(SD_TASK_TAG,"\r\n%s",pst->cIdentifier);*/
            ptr = strstr(data,pst->cParam1);
            if(ptr != NULL)
            {
            	/*ESP_LOGI(SD_TASK_TAG,"\r\n%s",pst->cParam1);*/

                ptr = strtok (ptr,":");
                ptr = strtok(NULL,":");
				if(ptr != NULL)
                {
					ucResp = pst->ucFuncPtr(ptr);
				}	                
                break;
            }                
        }
        pst++;
    }
    return(ucResp);
}
//////////////////////////////////////////////
//
//
//           TaskSd_ReadWriteConfig
//
//
//////////////////////////////////////////////
unsigned char TaskSd_ReadWriteConfig(sMessageType *psMessage)
{
	unsigned char boError = true;
	tstReadWriteDataByIdentifier const *pst = &astReadWriteTable[0];
	
	/*ESP_LOGI(SD_TASK_TAG,"\r\nTaskSd_ReadWriteConfig=%s\r\n",psMessage->pcMessageData);*/

	(void)SearchReadWriteDataByIdentifierFunctions(psMessage->pcMessageData, pst);

	return(boError);
}
//////////////////////////////////////////////
//
//
//  			ucSdWriteConfigFile
//
//
//////////////////////////////////////////////
unsigned char ucSdWriteConfigFile(void)
{
	unsigned char boError = true;
	tstConfiguration *pstConfigData = &stConfigData;
	FILE *f;
	static char BleDebugMsg[RX_BUF_SIZE];
	
	// Use POSIX and C standard library functions to work with files.
	// First create a file.
	ESP_LOGI(SD_TASK_TAG, "File will be written");
	f = fopen("/spiffs/CONFIG.TXT", "w");
	if (f == NULL) {
		ESP_LOGE(SD_TASK_TAG, "Failed to open file for writing");
		return(boError);
    }

    fwrite(pstConfigData,sizeof(tstConfiguration),1,f);
    fclose(f);
	
	memset(BleDebugMsg,0x00,sizeof(BleDebugMsg));
	strcpy(BleDebugMsg,"Data Written OK!");

	stSdMsg.ucSrc = SRC_SD;
	stSdMsg.ucDest = SRC_BLE;
	stSdMsg.ucEvent = (int)NULL;
	stSdMsg.pcMessageData = (char*)BleDebugMsg;
	xQueueSend(xQueueBle,( void * )&stSdMsg,NULL);
	
    memset(&cConfigRxBuffer,0x00,RX_BUF_SIZE);
    ptrRxConfig = &cConfigRxBuffer[0];

	return(boError);
}


//////////////////////////////////////////////
//
//
//  			ucSdWriteWifiApName
//
//
//////////////////////////////////////////////
unsigned char ucSdWriteWifiApName(char* a)
{
	unsigned char boError = true;

	ESP_LOGI(SD_TASK_TAG,"\r\n ucSdWriteWifiApName\r\n");

	memset(stConfigData.cWifiApName,0x00,sizeof(((tstConfiguration){0}).cWifiApName));
	strcpy(stConfigData.cWifiApName,a);

	ucSdWriteConfigFile();

	/*esp_restart();*/
    return (boError);
}

//////////////////////////////////////////////
//
//
//  			ucSdWriteModemApn
//
//
//////////////////////////////////////////////
unsigned char ucSdWriteModemApn(char* a)
{
	unsigned char boError = true;

	ESP_LOGI(SD_TASK_TAG,"\r\nucSdWriteModemApn\r\n");

	memset(stConfigData.cModemApn,0x00,sizeof(((tstConfiguration){0}).cModemApn));
	strcpy(stConfigData.cModemApn,a);

	ucSdWriteConfigFile();


	/*esp_restart();*/
    return (boError);
}

//////////////////////////////////////////////
//
//
//  			ucSdWriteModemApnLogin
//
//
//////////////////////////////////////////////
unsigned char ucSdWriteModemApnLogin(char* a)
{
	unsigned char boError = true;

	ESP_LOGI(SD_TASK_TAG,"\r\nucSdWriteModemApnLogin:%s\r\n",a);

	memset(stConfigData.cModemApnLogin,0x00,sizeof(((tstConfiguration){0}).cModemApnLogin));
	strcpy(stConfigData.cModemApnLogin,a);

	ESP_LOGI(SD_TASK_TAG,"\r\n stConfigData.cModemApnLogin:%s\r\n",stConfigData.cModemApnLogin);

	ucSdWriteConfigFile();


	/*esp_restart();*/
    return (boError);
}


//////////////////////////////////////////////
//
//
//  			ucSdWriteModemApnPassword
//
//
//////////////////////////////////////////////
unsigned char ucSdWriteModemApnPassword(char* a)
{
	unsigned char boError = true;
	
	ESP_LOGI(SD_TASK_TAG,"\r\nucSdWriteModemApnPassword\r\n");

	memset(stConfigData.cModemApnPassword,0x00,sizeof(stConfigData.cModemApnPassword));
	strcpy(stConfigData.cModemApnPassword,a);
	   
	ucSdWriteConfigFile();
	
	/*esp_restart();*/
    return (boError);
}

//////////////////////////////////////////////
//
//
//  			ucSdWriteTimeToSleep
//
//
//////////////////////////////////////////////
unsigned char ucSdWriteTimeToSleep(char* a)
{
	unsigned char boError = true;
	unsigned long u32;

	ESP_LOGI(SD_TASK_TAG,"\r\nucSdWriteTimeToSleep\r\n");

	u32  = atol(a);

	stConfigData.u32TimeToSleepInSec = u32;

	ucSdWriteConfigFile();

	return (boError);
}


//////////////////////////////////////////////
//
//
//  			ucSdAppSource
//
//
//////////////////////////////////////////////
unsigned char ucSdAppSource(char* a)
{
	unsigned char boError = true;

	ESP_LOGI(SD_TASK_TAG,"\r\nucSdAppSource\r\n");

	memset(stConfigData.cAppSource,0x00,sizeof(stConfigData.cAppSource));
	strcpy(stConfigData.cAppSource,a);

	ucSdWriteConfigFile();

	/*esp_restart();*/
    return (boError);
}

//////////////////////////////////////////////
//
//
//  			ucSdWriteWifiApPassword
//
//
//////////////////////////////////////////////
unsigned char ucSdWriteWifiApPassword(char* a)
{
	unsigned char boError = true;
	

	ESP_LOGI(SD_TASK_TAG,"\r\nucSdWriteWifiApPassword\r\n");

	memset(stConfigData.cWifiApPassword,0x00,sizeof(stConfigData.cWifiApPassword));
	strcpy(stConfigData.cWifiApPassword,a);	
	   
	ucSdWriteConfigFile();
	
	/*esp_restart();*/
    return (boError);
}

//////////////////////////////////////////////
//
//
//  			ucSdWritePeriodLogInSec
//
//
//////////////////////////////////////////////
unsigned char ucSdWritePeriodLogInSec(char* a)
{
	unsigned char boError = true;
	unsigned long u32;


	ESP_LOGI(SD_TASK_TAG,"\r\nucSdWritePeriodLogInSec\r\n");

	u32  = atol(a);

	stConfigData.u32PeriodLogInSec = u32;

	ucSdWriteConfigFile();

    return (boError);
}
//////////////////////////////////////////////
//
//
//  	ucSdWriteModemPeriodTxInSec
//
//
//////////////////////////////////////////////
unsigned char ucSdWriteModemPeriodTxInSec(char* a)
{
	unsigned char boError = true;
	unsigned long u32;

	ESP_LOGI(SD_TASK_TAG,"\r\nucSdWriteModemPeriodTxInSec\r\n");

	u32  = atol(a);

	stConfigData.u32ModemPeriodTxInSec = u32;

	ucSdWriteConfigFile();

    return (boError);
}

//////////////////////////////////////////////
//
//
//  			ucSdDoNothing
//
//
//////////////////////////////////////////////
unsigned char ucSdDoNothing(char* a)
{
	unsigned char boError = true;
	char *ptr;
	ptr = a;
    return (boError);
}

//////////////////////////////////////////////
//
//
//              Sd_Init
//
//
//////////////////////////////////////////////
void SdInit(void)
{
	/*esp_log_level_set(SD_TASK_TAG, ESP_LOG_INFO);*/
#if 0
	xQueueSd = xQueueCreate(sdQUEUE_LENGTH,			/* The number of items the queue can hold. */
							sizeof( sMessageType ) );	/* The size of each item the queue holds. */
#endif

    xTaskCreate(vTaskSd, "vTaskSd", 1024*4, NULL, configMAX_PRIORITIES-1, NULL);
	/* Create the queue used by the queue send and queue receive tasks.
	http://www.freertos.org/a00116.html */

    memset(&cConfigRxBuffer,0x00,RX_BUF_SIZE);
    ptrRxConfig = &cConfigRxBuffer[0];

	stSdMsg.ucSrc = SRC_SD;
	stSdMsg.ucDest = SRC_SD;
	stSdMsg.ucEvent = EVENT_SD_INIT;
	xQueueSend( xQueueSd, ( void * )&stSdMsg, 0);
}

//////////////////////////////////////////////
//
//
//              TaskSd_Init
//
//
//////////////////////////////////////////////
unsigned char TaskSd_Init(sMessageType *psMessage)
{
	unsigned char boError = true;

	char* cLocalBuffer = (char*) malloc(sizeof(stConfigData));
	tstConfiguration *pstConfigData = &stConfigData;

    ESP_LOGI(SD_TASK_TAG, "INIT\r\n");
    ESP_LOGI(SD_TASK_TAG, "Initializing SPIFFS");

    esp_vfs_spiffs_conf_t conf = {
      .base_path = "/spiffs",
      .partition_label = NULL,
      .max_files = 5,
      .format_if_mount_failed = true
    };

    // Use settings defined above to initialize and mount SPIFFS filesystem.
    // Note: esp_vfs_spiffs_register is an all-in-one convenience function.
    esp_err_t ret = esp_vfs_spiffs_register(&conf);

    if (ret != ESP_OK)
    {
        if (ret == ESP_FAIL)
        {
            ESP_LOGE(SD_TASK_TAG, "Failed to mount or format filesystem");
        }
        else if (ret == ESP_ERR_NOT_FOUND)
        {
            ESP_LOGE(SD_TASK_TAG, "Failed to find SPIFFS partition");
        }
        else
        {
            ESP_LOGE(SD_TASK_TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        boError = false;
        return(boError);
    }

    size_t total = 0, used = 0;
    ret = esp_spiffs_info(NULL, &total, &used);
    if (ret != ESP_OK)
    {
        ESP_LOGE(SD_TASK_TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
    }
    else
    {
        ESP_LOGI(SD_TASK_TAG, "Partition size: total: %d, used: %d", total, used);
    }

    /*********************************************
     *				DELETE FILE
     *********************************************/
#if 0
    if (remove("/spiffs/CONFIG.TXT") == 0)
    	ESP_LOGI(SD_TASK_TAG,"Deleted successfully");
     else
    	 ESP_LOGI(SD_TASK_TAG,"Unable to delete the file");
#endif

#if 1
    /*********************************************
     *		READ FILES INSIDE FOLDER
     *********************************************/
    DIR *dir;
    struct dirent *ent;
    if ((dir = opendir ("/spiffs/")) != NULL)
    {
		/* print all the files and directories within directory */
		while ((ent = readdir (dir)) != NULL)
		{
		  ESP_LOGI(SD_TASK_TAG,"%s\n", ent->d_name);
		}
		closedir (dir);
    }
    else
    {
      /* could not open directory */
    	ESP_LOGE(SD_TASK_TAG, "Error opening directory");
        boError = false;
        return(boError);
    }
#endif

    FILE *f;
#if 0
    /*********************************************
     *			WRITING CONFIG FILE
     *********************************************/
	boError = ucSdWriteConfigFile();
	if (boError != true) return(boError);	       
#endif

    /*********************************************
     *			READING CONFIG FILE
     *********************************************/
    // Open renamed file for reading
    ESP_LOGI(SD_TASK_TAG, "Reading Config file");
    f = fopen("/spiffs/CONFIG.TXT", "r");
    if (f == NULL)
    {
		ESP_LOGE(SD_TASK_TAG, "Failed to open file for reading");
		boError = false;
		free(cLocalBuffer);
		return(boError);
    }

    memset(cLocalBuffer,0x00,sizeof(tstConfiguration));
	fread (cLocalBuffer,sizeof(tstConfiguration),1,f);
	fclose(f);


#if 0
    /*********************************************
     *			UNMOUNT SPIFFS
     *********************************************/
    // All done, unmount partition and disable SPIFFS
    esp_vfs_spiffs_unregister(NULL);
    ESP_LOGI(SD_TASK_TAG, "SPIFFS unmounted");
#endif

	pstConfigData = (tstConfiguration*)(cLocalBuffer);
    /* TimeToSleep*/
	stConfigData.u32TimeToSleepInSec = pstConfigData->u32TimeToSleepInSec;	
	/* Period Log*/
	stConfigData.u32PeriodLogInSec = pstConfigData->u32PeriodLogInSec;	
    /* Buzzer Status*/
	if(strlen(pstConfigData->cBuzzerStatus))
	{
		memset(stConfigData.cBuzzerStatus,0x00,sizeof(stConfigData.cBuzzerStatus));
		strcpy(stConfigData.cBuzzerStatus,pstConfigData->cBuzzerStatus);	
	}		
	/* Wifi ApName*/
	if(strlen(pstConfigData->cWifiApName))
	{	
		memset(stConfigData.cWifiApName,0x00,sizeof(stConfigData.cWifiApName));
		strcpy(stConfigData.cWifiApName,pstConfigData->cWifiApName);
	}	
	/* Wifi Password*/
	if(strlen(pstConfigData->cWifiApPassword))
	{	
		memset(stConfigData.cWifiApPassword,0x00,sizeof(stConfigData.cWifiApPassword));
		strcpy(stConfigData.cWifiApPassword,pstConfigData->cWifiApPassword);
	}		
	/* Http Domain*/
	if(strlen(pstConfigData->cHttpDomain))
	{	
		memset(stConfigData.cHttpDomain,0x00,sizeof(stConfigData.cHttpDomain));
		strcpy(stConfigData.cHttpDomain,pstConfigData->cHttpDomain);
	}		
	/* cPageUrl*/
	if(strlen(pstConfigData->cPageUrl))
	{	
		memset(stConfigData.cPageUrl,0x00,sizeof(stConfigData.cPageUrl));
		strcpy(stConfigData.cPageUrl,pstConfigData->cPageUrl);
	}	
	/* cModemApn*/
	if(strlen(pstConfigData->cModemApn))
	{	
		memset(stConfigData.cModemApn,0x00,sizeof(stConfigData.cModemApn));
		strcpy(stConfigData.cModemApn,pstConfigData->cModemApn);
	}
	/* cModemApnLogin*/
	if(strlen(pstConfigData->cModemApnLogin))
	{
		memset(stConfigData.cModemApnLogin,0x00,sizeof(stConfigData.cModemApnLogin));
		strcpy(stConfigData.cModemApnLogin,pstConfigData->cModemApnLogin);
	}
	/* cModemPassword*/
	if(strlen(pstConfigData->cModemApnPassword))
	{	
		memset(stConfigData.cModemApnPassword,0x00,sizeof(stConfigData.cModemApnPassword));
		strcpy(stConfigData.cModemApnPassword,pstConfigData->cModemApnPassword);
	}	
    /* u32ModemPeriodTxInSec*/
	stConfigData.u32ModemPeriodTxInSec = pstConfigData->u32ModemPeriodTxInSec;	


	/* cAppSource*/
	if(strlen(pstConfigData->cAppSource))
	{
		memset(stConfigData.cAppSource,0x00,sizeof(stConfigData.cAppSource));
		strcpy(stConfigData.cAppSource,pstConfigData->cAppSource);
	}


    ESP_LOGI(SD_TASK_TAG, "\r\nu32TimeToSleepInSec=%d\r\n",(int)stConfigData.u32TimeToSleepInSec);
    ESP_LOGI(SD_TASK_TAG, "\r\nu32PeriodLogInSec=%d\r\n",(int)stConfigData.u32PeriodLogInSec);
    ESP_LOGI(SD_TASK_TAG, "\r\ncBuzzerStatus=%s\r\n",stConfigData.cBuzzerStatus);
    ESP_LOGI(SD_TASK_TAG, "\r\ncWifiAPName=%s\r\ncWifiPwd=%s\r\n",stConfigData.cWifiApName,stConfigData.cWifiApPassword);
    ESP_LOGI(SD_TASK_TAG, "\r\ncHttpDomain=%s\r\n",stConfigData.cHttpDomain);
    ESP_LOGI(SD_TASK_TAG, "\r\ncPageUrl=%s\r\n",stConfigData.cPageUrl);
    ESP_LOGI(SD_TASK_TAG, "\r\ncApn=%s\r\ncApnLogin=%s\r\ncApnPassword=%s\r\n",stConfigData.cModemApn,stConfigData.cModemApnLogin,stConfigData.cModemApnPassword);
    ESP_LOGI(SD_TASK_TAG, "\r\nu32ModemPeriodTxInSec=%d\r\n",(int)stConfigData.u32ModemPeriodTxInSec);

    free(cLocalBuffer);

	/*stSdMsg.ucSrc = SRC_SD;
	stSdMsg.ucDest = SRC_SD;
	stSdMsg.ucEvent = EVENT_SD_OPENING;
	xQueueSend( xQueueSd, ( void * )&stSdMsg, 0);*/


    return(boError);
}

//////////////////////////////////////////////
//
//
//              TaskSd_Opening
//
//
//////////////////////////////////////////////
unsigned char TaskSd_Opening(sMessageType *psMessage)
{
	unsigned char boError = true;
	FILE *f;
	char* cLocalBuffer = (char*) malloc(RX_BUF_SIZE+1);
	#if DEBUG_SDCARD
    ESP_LOGI(SD_TASK_TAG, "<<<<OPENING>>>>\r\n");
	#endif

    const char BleDebugMsg[] = "SD:OPENING FILE";

    stSdMsg.ucSrc = SRC_SD;
    stSdMsg.ucDest = SRC_BLE;
    stSdMsg.ucEvent = (int)NULL;
    stSdMsg.pcMessageData = (char*)BleDebugMsg;
	xQueueSend(xQueueBle,( void * )&stSdMsg,NULL);


    /*********************************************
     *		READ FILES INSIDE FOLDER
     *********************************************/
    DIR *dir;
    struct dirent *ent;

    if ((dir = opendir ("/spiffs/")) != NULL)
    {
		/* print all the files and directories within directory */
		(void)rewinddir(dir);
		for(;;)
		{
			if((ent = readdir (dir)) != NULL)
			{
				ESP_LOGI(SD_TASK_TAG,"%s,FileNumber=%d\n", ent->d_name,ent->d_ino);

				if(strstr(ent->d_name,"DATA_") != NULL)
				{
					if(strstr(ent->d_name,".TXT") != NULL)
					{
						memset(cLocalBuffer,0,RX_BUF_SIZE+1);

						strcpy(cLocalBuffer,"/spiffs/");
						strcat(cLocalBuffer,ent->d_name);
						f = fopen((const char*)cLocalBuffer, "r");
						if(f == NULL )
						{
							ESP_LOGE(SD_TASK_TAG, "Failed to open file for reading");
							boError = false;
						}
						else
						{
							fclose(f);
							strcpy(szFilenameToBeRead,cLocalBuffer);

							liFilePointerPositionBeforeReading = 0;
							liFilePointerPositionAfterReading = 0;

							stSdMsg.ucSrc = SRC_SD;
							stSdMsg.ucDest = SRC_SD;
							stSdMsg.ucEvent = EVENT_SD_READING;
							xQueueSend( xQueueSd, ( void * )&stSdMsg, 0);
						}
						break;
					}
					else/*File does not end with .TXT*/
					{
						#if DEBUG_SDCARD
						ESP_LOGE(SD_TASK_TAG, "Trying to remove file");
						#endif

						memset(cLocalBuffer,0,RX_BUF_SIZE+1);

						strcpy(cLocalBuffer,"/spiffs/");
						strcat(cLocalBuffer,ent->d_name);

						int ret = remove(cLocalBuffer);

						if(ret == 0)
						{
							#if DEBUG_SDCARD
							ESP_LOGE(SD_TASK_TAG, "File deleted successfully");
							#endif
						 }
					}
				}
				else
				{
					if(strstr(ent->d_name,"CONFIG") == NULL)
					{
						memset(cLocalBuffer,0,RX_BUF_SIZE+1);

						strcpy(cLocalBuffer,"/spiffs/");
						strcat(cLocalBuffer,ent->d_name);

						#if DEBUG_SDCARD
						ESP_LOGE(SD_TASK_TAG, "Trying to remove file:%s",cLocalBuffer);
						#endif

						int ret = remove(cLocalBuffer);

						if(ret == 0)
						{
							#if DEBUG_SDCARD
							ESP_LOGE(SD_TASK_TAG, "File deleted successfully");
							#endif
						 }
					}
				}
			}
			else
			{
				ESP_LOGE(SD_TASK_TAG,"No more DATA files\n");
		        boError = false;
#if SRC_GSM
				stSdMsg.ucSrc = SRC_SD;
				stSdMsg.ucDest = SRC_GSM;
				stSdMsg.ucEvent = EVENT_GSM_ENDING;
				xQueueSend( xQueueGsm, ( void * )&stSdMsg, 0);
#endif

#if SRC_HTTPCLI
				stSdMsg.ucSrc = SRC_SD;
				stSdMsg.ucDest = SRC_HTTPCLI;
				stSdMsg.ucEvent = EVENT_HTTPCLI_ENDING;
				xQueueSend( xQueueHttpCli, ( void * )&stSdMsg, 0);
#endif

				break;
			}
		}
		closedir (dir);
    }
    else
    {
      /* could not open directory */
    	ESP_LOGE(SD_TASK_TAG, "Error opening directory");
        boError = false;
    }
    free(cLocalBuffer);
	return(boError);
}


//////////////////////////////////////////////
//
//
//              TaskSd_Writing
//
//
//////////////////////////////////////////////
unsigned char TaskSd_Writing(sMessageType *psMessage)
{
	unsigned char boError = true;
	FILE *f;

#if DEBUG_SDCARD
    ESP_LOGI(SD_TASK_TAG, "<<<<WRITING>>>>\r\n%s\r\n<<<<>>>>\r\n",szFilenameToBeWritten);
#endif

    const char BleDebugMsg[] = "SD:WRITING FILE";

    stSdMsg.ucSrc = SRC_SD;
    stSdMsg.ucDest = SRC_BLE;
    stSdMsg.ucEvent = (int)NULL;
    stSdMsg.pcMessageData = (char*)BleDebugMsg;
	xQueueSend(xQueueBle,( void * )&stSdMsg,NULL);


#if 0
    f = fopen((const char*)szFilenameToBeWritten, "w+" );
#if DEBUG_SDCARD
    ESP_LOGI(SD_TASK_TAG, "DATA GPS:%s",cGpsDataCopiedToSd);
#endif
	fprintf(f,"%s",(const char*)"DANILO");
	fclose(f);

    f = fopen((const char*)szFilenameToBeWritten, "r" );
	char* cLocalBuffer = (char*) malloc(RX_BUF_SIZE+1);
	memset(cLocalBuffer,0,RX_BUF_SIZE+1);
	fread (cLocalBuffer,1,RX_BUF_SIZE,f);
	/*fgets (cLocalBuffer,RX_BUF_SIZE,f);*/

#if DEBUG_SDCARD
    ESP_LOGI(SD_TASK_TAG, "DATA:%s",cLocalBuffer);
#endif
	free(cLocalBuffer);
	(void)fclose(f);
#endif


	f = fopen(szFilenameToBeWritten, "a" );
	if(f != NULL)
	{
#if DEBUG_SDCARD
    ESP_LOGI(SD_TASK_TAG, "Reading for update...");
#endif
    	int i;
		if((i = strlen(cGpsDataCopiedToSd))>0)
		{
		    ESP_LOGI(SD_TASK_TAG, "Qty:%d,%s",i,cGpsDataCopiedToSd);

			int i = fprintf(f,"%s",cGpsDataCopiedToSd);
			if(i > 0)
			{
				#if DEBUG_SDCARD
				ESP_LOGI(SD_TASK_TAG, "Writing OK!");
				#endif
			}
			else
			{
				#if DEBUG_SDCARD
				ESP_LOGI(SD_TASK_TAG, "Writing NOK!");
				#endif
			}
		}
		fclose(f);
	}
	else
	{
		f = fopen(szFilenameToBeWritten, "w");
		if(f != NULL)
		{
			if(strlen(cGpsDataCopiedToSd)>0)
			{
				int i = fprintf(f,"%s",cGpsDataCopiedToSd);
				if(i > 0)
				{
					#if DEBUG_SDCARD
					ESP_LOGI(SD_TASK_TAG, "Writing new file");
					#endif
				}
				else
				{
					#if DEBUG_SDCARD
					ESP_LOGE(SD_TASK_TAG, "Error writing new file");
					#endif
				}
			}
			fclose(f);
		}
	}
	return(boError);
}

//////////////////////////////////////////////
//
//
//              TaskSd_Reading
//
//
//////////////////////////////////////////////
unsigned char TaskSd_Reading(sMessageType *psMessage)
{
	unsigned char boError = true;
	char* cLocalBuffer = (char*) malloc(RX_BUF_SIZE+1);
	FILE *f;

#if DEBUG_SDCARD
	ESP_LOGI(SD_TASK_TAG, "<<<<READING>>>>\r\n%s\r\n<<<<>>>>\r\n",szFilenameToBeRead);
#endif

    const char BleDebugMsg[] = "SD:READING FILE";

    stSdMsg.ucSrc = SRC_SD;
    stSdMsg.ucDest = SRC_BLE;
    stSdMsg.ucEvent = (int)NULL;
    stSdMsg.pcMessageData = (char*)BleDebugMsg;
	xQueueSend(xQueueBle,( void * )&stSdMsg,NULL);

	f = fopen(szFilenameToBeRead, "r");
	if(f != NULL)
	{
		#if DEBUG_SDCARD
			ESP_LOGI(SD_TASK_TAG, "Opening for Reading OK!");
		#endif

		for(;;)
		{
			fseek(f,liFilePointerPositionBeforeReading,SEEK_SET);

			#if DEBUG_SDCARD
			ESP_LOGI(SD_TASK_TAG, "PositionBeforeReading=%d\r\n",(int)liFilePointerPositionBeforeReading);
			#endif

			/*Reading file until finds \n or RX_BUF_SIZE, whichever happens first*/
			memset(cLocalBuffer,0,RX_BUF_SIZE+1);
			int i;
			if((fgets ( cLocalBuffer, RX_BUF_SIZE, f )) != NULL )
			{

				liFilePointerPositionAfterReading =  ftell (f);

				#if DEBUG_SDCARD
				ESP_LOGI(SD_TASK_TAG, "PositionAfterReading=%d\r\n",(int)liFilePointerPositionAfterReading);
				#endif

				i = strlen(cLocalBuffer);

				ESP_LOGI(SD_TASK_TAG, "Content of file with %d Bytes: %s",i, cLocalBuffer);
				/* Verify if file is OK*/
				if((cLocalBuffer[0] == 'S') && (cLocalBuffer[1] == '=') && (cLocalBuffer[i-2] == '\r') && (cLocalBuffer[i-1] == '\n'))
				{
					memset(cConfigAndData,0,RX_BUF_SIZE+1);
					strncpy(cConfigAndData,cLocalBuffer,i-2);



#if SRC_GSM
					//if(ucCurrentStateGsm == TASKGSM_COMMUNICATING)
					//{
						stSdMsg.ucSrc = SRC_SD;
						stSdMsg.ucDest = SRC_GSM;
						stSdMsg.ucEvent = EVENT_GSM_SEND_HTTPURL/*EVENT_GSM_SEND_HTTPPREPAREDATA*//*EVENT_GSM_SEND_HTTPPREPAREDATA*//*EVENT_GSM_SEND_START*//*EVENT_GSM_SEND_CIPSEND*//*EVENT_GSM_SEND_DATA*/;
						stSdMsg.pcMessageData = "";
						xQueueSend( xQueueGsm, ( void * )&stSdMsg, 0);
					//}
#endif
#if SRC_HTTPCLI
					//else
					//{
						stSdMsg.ucSrc = SRC_SD;
						stSdMsg.ucDest = SRC_HTTPCLI;
						stSdMsg.ucEvent = EVENT_HTTPCLI_POST;
						stSdMsg.pcMessageData = "";
						xQueueSend( xQueueHttpCli, ( void * )&stSdMsg, 0);
					//}
#endif
					fclose(f);
					break;
				}
				else
				{
					liFilePointerPositionBeforeReading = liFilePointerPositionAfterReading;
				}
			}
			else
			{
				#if DEBUG_SDCARD
					ESP_LOGE(SD_TASK_TAG, "No possible to retrieve data");
				#endif


				if(feof(f))
				{
					ESP_LOGI(SD_TASK_TAG, "End of File");
				}
				else
				{
					ESP_LOGI(SD_TASK_TAG, "File has not been deleted before, delete now!");
				}

				int ret = remove(szFilenameToBeRead);

				if(ret == 0)
				{
					#if DEBUG_SDCARD
					ESP_LOGE(SD_TASK_TAG, "File deleted successfully");
					#endif
				 }
				else
				{
					#if DEBUG_SDCARD
					ESP_LOGE(SD_TASK_TAG, "Error: unable to delete the file");
					#endif
				}
#if SRC_GSM
				stSdMsg.ucSrc = SRC_SD;
				stSdMsg.ucDest = SRC_GSM;
				stSdMsg.ucEvent = EVENT_GSM_ENDING/*EVENT_GSM_LIST_SMS_MSG*/;
				xQueueSend( xQueueGsm, ( void * )&stSdMsg, 0);
#endif

#if SRC_HTTPCLI
				stSdMsg.ucSrc = SRC_SD;
				stSdMsg.ucDest = SRC_HTTPCLI;
				stSdMsg.ucEvent = EVENT_HTTPCLI_ENDING/*EVENT_GSM_LIST_SMS_MSG*/;
				xQueueSend( xQueueHttpCli, ( void * )&stSdMsg, 0);
#endif

				fclose(f);
				break;
			}
		}
	}
	else
	{
		#if DEBUG_SDCARD
			ESP_LOGE(SD_TASK_TAG, "No more files...");
		#endif
#if SRC_GSM
		stSdMsg.ucSrc = SRC_SD;
		stSdMsg.ucDest = SRC_GSM;
		stSdMsg.ucEvent = EVENT_GSM_ENDING/*EVENT_GSM_LIST_SMS_MSG*/;
		xQueueSend( xQueueGsm, ( void * )&stSdMsg, 0);
#endif

#if SRC_HTTPCLI
		stSdMsg.ucSrc = SRC_SD;
		stSdMsg.ucDest = SRC_HTTPCLI;
		stSdMsg.ucEvent = EVENT_HTTPCLI_ENDING;
		xQueueSend( xQueueHttpCli, ( void * )&stSdMsg, 0);
#endif

		stSdMsg.ucSrc = SRC_SD;
		stSdMsg.ucDest = SRC_SD;
		stSdMsg.ucEvent = EVENT_SD_OPENING;
		xQueueSend( xQueueSd, ( void * )&stSdMsg, 0);



	}
	free(cLocalBuffer);
	return(boError);
}

//////////////////////////////////////////////
//
//
//              TaskSd_Marking
//
//
//////////////////////////////////////////////
unsigned char TaskSd_Marking(sMessageType *psMessage)
{
	unsigned char boError = true;
	FILE *f;

#if DEBUG_SDCARD
	ESP_LOGI(SD_TASK_TAG, "<<<<MARKING>>>>\r\n");
#endif

    const char BleDebugMsg[] = "SD:MARKING FILE";

    stSdMsg.ucSrc = SRC_SD;
    stSdMsg.ucDest = SRC_BLE;
    stSdMsg.ucEvent = (int)NULL;
    stSdMsg.pcMessageData = (char*)BleDebugMsg;
	xQueueSend(xQueueBle,( void * )&stSdMsg,NULL);

	if((f = fopen(szFilenameToBeRead, "r+")) != NULL)
	{
		fseek(f,liFilePointerPositionBeforeReading,SEEK_SET);
		fputs("*",f);
		liFilePointerPositionBeforeReading = liFilePointerPositionAfterReading;
		fclose(f);

	}

	stSdMsg.ucSrc = SRC_SD;
	stSdMsg.ucDest = SRC_SD;
	stSdMsg.ucEvent = EVENT_SD_READING;
	xQueueSend( xQueueSd, ( void * )&stSdMsg, 0);

	return(boError);
}



//////////////////////////////////////////////
//
//
//              TaskSd_IgnoreEvent
//
//
//////////////////////////////////////////////
unsigned char TaskSd_IgnoreEvent(sMessageType *psMessage)
{
	unsigned char boError = false;
	return(boError);
}

static sStateMachineType const gasTaskSd_Initializing[] =
{
        /* Event                            Action routine          Success state               Failure state*/
        //  State specific transitions
        {EVENT_SD_INIT,                     TaskSd_Init,            	TASKSD_INITIALIZING,        TASKSD_INITIALIZING },
		{EVENT_SD_OPENING, 					TaskSd_Opening,			 	TASKSD_INITIALIZING,		TASKSD_INITIALIZING	},
		{EVENT_SD_WRITING, 					TaskSd_Writing,	    	 	TASKSD_INITIALIZING,		TASKSD_INITIALIZING	},
		{EVENT_SD_READING, 					TaskSd_Reading,				TASKSD_INITIALIZING,		TASKSD_INITIALIZING	},
	   	{EVENT_SD_MARKING, 					TaskSd_Marking,				TASKSD_INITIALIZING,		TASKSD_INITIALIZING },
	   	{EVENT_SD_READWRITE_CONFIG,			TaskSd_ReadWriteConfig,		TASKSD_INITIALIZING,		TASKSD_INITIALIZING },		
	    {EVENT_SD_NULL,                     TaskSd_IgnoreEvent,			TASKSD_INITIALIZING,		TASKSD_INITIALIZING }
};

static sStateMachineType const * const gpasTaskSd_StateMachine[] =
{
    gasTaskSd_Initializing
};

static unsigned char ucCurrentStateSd = TASKSD_INITIALIZING;

void vTaskSd( void *pvParameters )
{
	TickType_t elapsed_time;

    uint8_t * spp_cmd_buff = NULL;
    spp_cmd_buff = (uint8_t *)malloc((128) * sizeof(uint8_t));

	for( ;; )
	{
	    /*ESP_LOGI(SD_TASK_TAG, "Running...");*/
		elapsed_time = xTaskGetTickCount();

		if( xQueueReceive( xQueueSd, &( stSdMsg ),0 ) )
		{
            (void)eEventHandler ((unsigned char)SRC_SD,gpasTaskSd_StateMachine[ucCurrentStateSd], &ucCurrentStateSd, &stSdMsg);
		}

		if(strlen(cConfigRxBuffer) > 0)
		{
			ESP_LOGI(SD_TASK_TAG, "%s\r\n",cConfigRxBuffer);

			if(strstr((const char*)cConfigRxBuffer,"$CONF:") != NULL)
        	{
                ESP_LOGI(SD_TASK_TAG, "$CONF:\r\n");
				/* Receive data over BT and pass it over to SD*/
				stSdMsg.ucSrc = SRC_SD;
				stSdMsg.ucDest = SRC_SD;
				stSdMsg.ucEvent = EVENT_SD_READWRITE_CONFIG;
				stSdMsg.pcMessageData = (char*)cConfigRxBuffer;

				xQueueSend( xQueueSd, ( void * )&stSdMsg, NULL);
        	}
		}

		vTaskDelayUntil(&elapsed_time, 1000 / portTICK_PERIOD_MS);
	}




}
