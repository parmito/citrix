/*
 * Gsm.c
 *
 *  Created on: 24/09/2018
 *      Author: danilo
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

/* Kernel includes. */
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
#include "driver/gpio.h"

#include "UartGsm.h"
#include "State.h"
#include "defines.h"
#include "Sd.h"
#include "Debug.h"
#include "Ble.h"
#include "Gsm.h"

#define M2M_VODAFONE	0
#define VODAFONE		1
#define CLARO			2
#define VIVO			3
#define TMDATA			4

#define APN TMDATA


//////////////////////////////////////////////
//
//
//            FUNCTION PROTOTYPES
//
//
//////////////////////////////////////////////
unsigned char TaskGsm_ParseSmsResp(void);
unsigned char TaskSd_IgnoreEvent(sMessageType *psMessage);
unsigned char TaskGsm_ParseSmsGetCsq(sMessageType *psMessage);
unsigned char TaskGsm_ParseSmsGetCcid(sMessageType *psMessage);
unsigned char TaskGsm_ParseSmsGetGps(sMessageType *psMessage);
unsigned char TaskGsm_DeleteSmsMsg(sMessageType *psMessage);

unsigned char TaskGsm_SendStart(sMessageType *psMessage);
unsigned char TaskGsm_SendCipsend(sMessageType *psMessage);
unsigned char TaskGsm_SendData(sMessageType *psMessage);

void vTaskGsm( void *pvParameters );



//////////////////////////////////////////////
//
//
//            	VARIABLES
//
//
//////////////////////////////////////////////
char cGsmRxBuffer[RX_BUF_SIZE];
char cGsmSmsBuffer[RX_BUF_SIZE+10];
char *ptrRxGsm;
extern char cConfigAndData[RX_BUF_SIZE];
extern tstConfiguration stConfigData;

char cExpectedResp1[RX_BUF_SIZE_REDUCED];
char cExpectedResp2[RX_BUF_SIZE_REDUCED];

unsigned char ucModemAttempt = MODEM_ATTEMPT;
unsigned char ucCurrentStateGsm = TASKGSM_IDLING;


extern char cApn[RX_BUF_SIZE_REDUCED];
extern char cApnLogin[RX_BUF_SIZE_REDUCED];
extern char cApnPassword[RX_BUF_SIZE_REDUCED];
extern char cHttpDns[RX_BUF_SIZE_REDUCED];
extern char cWebPage[RX_BUF_SIZE_REDUCED];

const char cModem_OK[]      = {"OK"};
const char cModem_ERROR[] = {"ERROR"};
const char cModem_ACK[]     = {"ReGcOr"};
const char cCREG1[] ={"+CREG: 0,1"};
const char cCREG2[] ={"+CREG: 0,2"};
const char cCSQ1[]  ={"+CSQ:"};
const char cCSQ2[]  ={"+CSQ: 0,0"};
const char cCPIN1[] ={"+CPIN: READY"};
const char cCIICR1[] ={"+PDP: DEACT"};
const char cCGML[]  ={"+CMGL: "};
const char cHttpAtStartCIP[]    = {"AT+CIPSTART=\"TCP\",\""};
const char cHttpAtEndCIP[]      = {"\",\"80\""};
const char cHttpHeader1[]       = {"POST "};
const char cHttpHeader2[]       = {" HTTP/1.1\r\nHost: "};
const char cHttpHeader3[]       = {":8080\r\nAccept: */*\r\nContent-Length: "};
const char cHttpHeader4[]       = {"Connection: Keep-Alive\r\nContent-Type: application/x-www-form-urlencoded\r\n\r\n"};
const char cHttpContent[]       = {"application/x-www-form-urlencoded"};

const char cTEL1[]  ={"TEL1=\""};
const char cTEL2[]  ={"TEL2=\""};
const char cTEL3[]  ={"TEL3=\""};
const char cRESET[]  ={"RESET"};
const char cALARM[]  ={"ALARM"};

const char cSET[]  ={"SET "};
const char cGET[]  ={"GET "};
const char cGETCSQ[]  ={"GET CSQ"};
const char cESN[]  ={"ESN"};
const char cGETCCID[]  ={"GET CCID"};

const char cSEND[] ={">"};
const char cCIPSEND[]  			= {"AT+CIPSEND="};

static const char *GSM_TASK_TAG = "GSM_TASK";
static SMS_TYPE stSmsRecv;
/*static PHONE_TYPE stPhone[3];*/

sMessageType stGsmMsg;
static unsigned char ucGsmTry = 0;
extern tstIo stIo;
extern tstConfiguration stConfigData;

static sSmsCommandType const gasTaskGsm_SmsCommandTable[] =
{
		{	"GET CSQ",		TaskGsm_ParseSmsGetCsq				},
		{	"GET CCID",		TaskGsm_ParseSmsGetCcid				},
		{	"GET GPS",		TaskGsm_ParseSmsGetGps				},
		/*{	"SET CFG PL",	TaskGsm_ParseSmsSetConfigPeriodLog	},*/
		{	(char*)NULL,	TaskGsm_DeleteSmsMsg				}
};

typedef struct
{
	/* Command : GET or SET  */
	unsigned char ucReponseCode;
	/* action routine invoked for the event */
	const char *cWifiResponse;
}tstGsmTableResponse;


//////////////////////////////////////////////
//
//
//              Gsm_On
//
//
//////////////////////////////////////////////
void Gsm_On(void)
{
	gpio_set_level(GPIO_OUTPUT_GSM_ENABLE, 1);
}

//////////////////////////////////////////////
//
//
//              Gsm_Off
//
//
//////////////////////////////////////////////
void Gsm_Off(void)
{
	gpio_set_level(GPIO_OUTPUT_GSM_ENABLE, 0);
}

//////////////////////////////////////////////
//
//
//              Gsm_Io_Configuration
//
//
//////////////////////////////////////////////
void Gsm_Io_Configuration(void)
{
	gpio_config_t io_conf;
	//disable interrupt
	io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
	//set as output mode
	io_conf.mode = GPIO_MODE_OUTPUT;
	//bit mask of the pins that you want to set,e.g.GPIO18/19
	io_conf.pin_bit_mask = GPIO_OUTPUT_GSM_ENABLE_PIN_SEL ;
	//disable pull-down mode
	io_conf.pull_down_en = 0;
	//disable pull-up mode
	io_conf.pull_up_en = 0;
	//configure GPIO with the given settings
	gpio_config(&io_conf);

}


//////////////////////////////////////////////
//
//
//              Gsm_Init
//
//
//////////////////////////////////////////////
void GsmInit(void)
{
	/*esp_log_level_set(GSM_TASK_TAG, ESP_LOG_INFO);*/
    ESP_LOGI(GSM_TASK_TAG, "GSM INIT");

	Gsm_Io_Configuration();

#if 0
	xQueueGsm = xQueueCreate(gsmQUEUE_LENGTH,			/* The number of items the queue can hold. */
								sizeof( sMessageType ) );	/* The size of each item the queue holds. */
#endif

    xTaskCreate(vTaskGsm, "vTaskGsm", 1024*4, NULL, configMAX_PRIORITIES-7, NULL);
	/* Create the queue used by the queue send and queue receive tasks.
	http://www.freertos.org/a00116.html */


	stGsmMsg.ucSrc = SRC_SD;
	stGsmMsg.ucDest = SRC_GSM;
	stGsmMsg.ucEvent = EVENT_GSM_INIT;
	xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
}

//////////////////////////////////////////////
//
//
//              TaskGsm_SendAtCmd1
//
//
//////////////////////////////////////////////
void TaskGsm_SendAtCmd1(const char *pcDebug,const char *pcAtCmd)
{

    ESP_LOGI(GSM_TASK_TAG, "\r\n<<<<SEND GSM 1>>>>\r\n%s\r\n%s\r\n<<<<>>>>\r\n",pcDebug,pcAtCmd);

    memset(&cGsmRxBuffer,0x00,RX_BUF_SIZE);
    ptrRxGsm = &cGsmRxBuffer[0];

    UartGsmSendData(GSM_TASK_TAG,pcAtCmd);
#if SRC_BLE
	stGsmMsg.ucSrc = SRC_GSM;
	stGsmMsg.ucDest = SRC_BLE;
	stGsmMsg.ucEvent = (int)NULL;
	stGsmMsg.pcMessageData = (char*)pcDebug;
	xQueueSend(xQueueBle,( void * )&stGsmMsg,NULL);
#endif
}


//////////////////////////////////////////////
//
//
//              TaskGsm_SendAtCmd
//
//
//////////////////////////////////////////////
void TaskGsm_SendAtCmd(const char *pcDebug,const char *pcAtCmd,\
const char *pcSuccessResp, const char* pcFailedResp)
{
    ESP_LOGI(GSM_TASK_TAG, "\r\n<<<<SEND GSM>>>>\r\n%s\r\n%s<<<<>>>>\r\n",pcDebug,pcAtCmd);

    memset(&cExpectedResp1,0x00,RX_BUF_SIZE_REDUCED);
    strcpy(&cExpectedResp1[0],pcSuccessResp);

    memset(&cExpectedResp2,0x00,RX_BUF_SIZE_REDUCED);
    strcpy(&cExpectedResp2[0],pcFailedResp);

    memset(&cGsmRxBuffer,0x00,RX_BUF_SIZE);
    ptrRxGsm = &cGsmRxBuffer[0];

    UartGsmSendData(GSM_TASK_TAG,pcAtCmd);
}

//////////////////////////////////////////////
//
//
//              TaskGsm_ParseResp1
//
//
//////////////////////////////////////////////
unsigned char TaskGsm_ParseResp1(tstGsmTableResponse const *pstGsmTableResponse)
{
    unsigned char ucResp = 0;
    const char *ptr=NULL;

    tstGsmTableResponse const *pst = pstGsmTableResponse;

    while(pst->ucReponseCode != 255){

    	ptr = strstr((const char*)&cGsmRxBuffer[0],pst->cWifiResponse);
        if(ptr != NULL)
        {
        	ucResp = pst->ucReponseCode;
        	ESP_LOGI(GSM_TASK_TAG, "\r\n<<<<PARSE1 RESP:%d GSM>>>>\r\n%s\r\n<<<<>>>>\r\n",ucResp,cGsmRxBuffer/*pst->cWifiResponse*/);
        	break;
        }
        pst++;
    }

    return(ucResp);
}

//////////////////////////////////////////////
//
//
//              TaskGsm_ParseResp
//
//
//////////////////////////////////////////////
unsigned char TaskGsm_ParseResp(void)
{
    unsigned char ucResp = 0;
    const char *ptr=NULL;

    ptr = strstr((const char*)&cGsmRxBuffer[0],(const char*)&cExpectedResp1[0]);
    if(ptr != NULL)
    {
        ucResp = 1;
        ESP_LOGI(GSM_TASK_TAG, "\r\n<<<<RESP:1 GSM>>>>\r\n%s\r\n<<<<>>>>\r\n",cGsmRxBuffer);
    }
    else
    {
        ptr = strstr((const char*)&cGsmRxBuffer[0],(const char*)&cExpectedResp2[0]);
        if(ptr != NULL)
        {
            ucResp = 2;
            ESP_LOGI(GSM_TASK_TAG, "\r\n<<<<RESP:2 GSM>>>>\r\n%s\r\n<<<<>>>>\r\n",cGsmRxBuffer);
        }
        else
        {
        	if(strlen(cGsmRxBuffer) > 0)
        	{
        		ESP_LOGI(GSM_TASK_TAG, "\r\n<<<<RESP:0 GSM>>>>\r\n%s\r\n<<<<>>>>\r\n",cGsmRxBuffer);
        	}
        }
    }

    return(ucResp);
}

//////////////////////////////////////////////
//
//
//        TaskGsm_WaitResponse1
//
//
//////////////////////////////////////////////
unsigned char TaskGsm_WaitResponse1(const tstGsmTableResponse *pstGsmTableResponse, unsigned char (*ucFunc)(const tstGsmTableResponse *),unsigned char ucPeriod, unsigned char ucTimeout)
{
    unsigned char ucResp = 0;

    while((ucResp == 0) && (ucTimeout > 0))
    {
        vTaskDelay((ucPeriod*1000)/portTICK_PERIOD_MS);
        ucResp = (*ucFunc)(pstGsmTableResponse);
        if(ucTimeout > 0 ) ucTimeout--;
    }

    return(ucResp);
}


//////////////////////////////////////////////
//
//
//        TaskGsm_WaitResponse
//
//
//////////////////////////////////////////////
unsigned char TaskGsm_WaitResponse(unsigned char (*ucFunc)(void),unsigned char ucPeriod, unsigned char ucTimeout)
{
    unsigned char ucResp = 0;

    while((ucTimeout > 0))
    {
        vTaskDelay((ucPeriod*1000)/portTICK_PERIOD_MS);
        ucResp = ucFunc();
        if(ucResp > 0)
        {
        	break;
        }
        else
        {
            if(ucTimeout > 0 )
            {
            	ucTimeout--;
            }
        }
    }

    return(ucResp);
}


//////////////////////////////////////////////
//
//
//              TaskGsm_Init
//
//
//////////////////////////////////////////////
unsigned char TaskGsm_Init(sMessageType *psMessage)
{
    unsigned char boError = true;

	ESP_LOGI(GSM_TASK_TAG, "GSM:>INIT\r\n");

	stGsmMsg.ucSrc = SRC_GSM;
	stGsmMsg.ucDest = SRC_DEBUG;
	stGsmMsg.ucEvent = EVENT_IO_GSM_INIT;
	xQueueSend( xQueueDebug, ( void * )&stGsmMsg, 0);

	Gsm_Off();
	vTaskDelay(100/portTICK_PERIOD_MS);
	Gsm_On();

	stGsmMsg.ucSrc = SRC_GSM;
	stGsmMsg.ucDest = SRC_GSM;
	stGsmMsg.ucEvent = EVENT_GSM_SETBAUD;
	xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);


    return(boError);
}

//////////////////////////////////////////////
//
//
//              TaskGsm_SetBaudRate
//
//
//////////////////////////////////////////////
unsigned char TaskGsm_SetBaudRate(sMessageType *psMessage)
{
	unsigned char boError = true;
    unsigned char ucResp = 0xFF;

	/*TaskGsm_SendAtCmd("GSM:BAUD\r\n","AT\r\n",cModem_OK,cModem_ERROR);
	ucResp = TaskGsm_WaitResponse(TaskGsm_ParseResp,1,3);*/

    tstGsmTableResponse const gasTableGsmResponse[] =
    {
    		{	1,		"OK"				},
    		{	2,		"ERROR"				},
    		{  255,    "END OF ARRAY"		}
    };

	TaskGsm_SendAtCmd1("GSM:>BAUD\r\n","AT\r\n");
	ucResp = TaskGsm_WaitResponse1(&gasTableGsmResponse[0],TaskGsm_ParseResp1,1,3);

	switch(ucResp)
	{
		case 0:
		case 2:
			stGsmMsg.ucSrc = SRC_GSM;
			stGsmMsg.ucDest = SRC_GSM;
			stGsmMsg.ucEvent = EVENT_GSM_SETBAUD;
			xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
			boError = false;
		break;

		case 1:
			stGsmMsg.ucSrc = SRC_GSM;
			stGsmMsg.ucDest = SRC_GSM;
			stGsmMsg.ucEvent = EVENT_GSM_AT;
			xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);

			boError = true;
		break;

		default:
		break;
	}

	return(boError);

}

//////////////////////////////////////////////
//
//
//              TaskGsm_SendAt
//
//
//////////////////////////////////////////////
unsigned char TaskGsm_SendAt(sMessageType *psMessage)
{
    unsigned char boError = true;
    unsigned char ucResp = 0xFF;

    /*TaskGsm_SendAtCmd("GSM:AT\r\n","AT\r\n",cModem_OK,cModem_ERROR);
    ucResp = TaskGsm_WaitResponse(TaskGsm_ParseResp,1, 5);*/

    tstGsmTableResponse const gasTableGsmResponse[] =
    {
    		{	1,		"OK"				},
    		{	2,		"ERROR"				},
    		{  255,    "END OF ARRAY"		}
    };

	TaskGsm_SendAtCmd1("GSM:>AT\r\n","AT\r\n");
	ucResp = TaskGsm_WaitResponse1(&gasTableGsmResponse[0],TaskGsm_ParseResp1,1,5);

    switch(ucResp)
    {
        case 0:
        case 2:
			stGsmMsg.ucSrc = SRC_GSM;
			stGsmMsg.ucDest = SRC_GSM;
			stGsmMsg.ucEvent = EVENT_GSM_AT;
			xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
            boError = false;
        break;

        case 1:
			stGsmMsg.ucSrc = SRC_GSM;
			stGsmMsg.ucDest = SRC_GSM;
			stGsmMsg.ucEvent = EVENT_GSM_GET_CCID;
			xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);

            boError = true;
        break;

        default:
        break;
    }
    return(boError);
}

//////////////////////////////////////////////
//
//
//              TaskGsm_GetCcid
//
//
//////////////////////////////////////////////
unsigned char TaskGsm_GetCcid(sMessageType *psMessage)
{
    unsigned char boError = true;
    static unsigned char ucResp = 0xFF;

    /*TaskGsm_SendAtCmd("GSM:CCID\r\n","AT+CCID\r\n",cModem_OK,cModem_ERROR);
    ucResp = TaskGsm_WaitResponse(TaskGsm_ParseResp,1, 5);*/

    tstGsmTableResponse const gasTableGsmResponse[] =
    {
    		{	1,		"OK"				},
    		{	2,		"ERROR"				},
    		{  255,    "END OF ARRAY"		}
    };

	TaskGsm_SendAtCmd1("GSM:>CCID\r\n","AT+CCID\r\n");
	ucResp = TaskGsm_WaitResponse1(&gasTableGsmResponse[0],TaskGsm_ParseResp1,1,5);

    switch(ucResp)
    {
		case 0:
		case 2:
			stGsmMsg.ucSrc = SRC_GSM;
			stGsmMsg.ucDest = SRC_GSM;
			stGsmMsg.ucEvent = EVENT_GSM_GET_CCID;
			xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
			boError = false;
		break;


        case 1:
			stGsmMsg.ucSrc = SRC_GSM;
			stGsmMsg.ucDest = SRC_GSM;
			stGsmMsg.ucEvent = EVENT_GSM_SEND_CSQ;
			xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
            boError = false;
            break;
    }
    return(boError);
}


//////////////////////////////////////////////
//
//
//              TaskGsm_SendCpin
//
//
//////////////////////////////////////////////
unsigned char TaskGsm_SendCpin(sMessageType *psMessage)
{
    unsigned char boError = true;
#if 0
    TaskGsm_SendAtCmd("GSM:CPIN\r\n","AT+CPIN?\r\n",cCPIN1,cModem_ERROR);

    unsigned char ucResp;
    ucResp = TaskGsm_WaitResponse(TaskGsm_ParseResp,1, 5);

    switch(ucResp)
    {
        case 0:
        case 2:
			stGsmMsg.ucSrc = SRC_GSM;
			stGsmMsg.ucDest = SRC_GSM;
			stGsmMsg.ucEvent = EVENT_GSM_SEND_CPIN;
			xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
            boError = false;
        break;

        case 1:
			stGsmMsg.ucSrc = SRC_GSM;
			stGsmMsg.ucDest = SRC_GSM;
			stGsmMsg.ucEvent = EVENT_GSM_SEND_CSQ;
			xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
            boError = true;
        break;

        default:
        break;
    }
#endif
    return(boError);
}

//////////////////////////////////////////////
//
//
//              TaskGsm_SendCsq
//
//
//////////////////////////////////////////////
unsigned char TaskGsm_SendCsq(sMessageType *psMessage)
{
    unsigned char boError = true;
    static unsigned char ucResp = 0xFF;

    tstGsmTableResponse const gasTableGsmResponse[] =
    {
    		{	1,		cCSQ1				},
			{	2, 		cCSQ2				},
    		{	3,		"ERROR"				},
    		{  255,    "END OF ARRAY"		}
    };


	TaskGsm_SendAtCmd1("GSM:CSQ\r\n","AT+CSQ\r\n");
	ucResp = TaskGsm_WaitResponse1(&gasTableGsmResponse[0],TaskGsm_ParseResp1,1,5);

    switch(ucResp)
    {
        case 2:
        case 3:
			stGsmMsg.ucSrc = SRC_GSM;
			stGsmMsg.ucDest = SRC_GSM;
			stGsmMsg.ucEvent = EVENT_GSM_SEND_CSQ;
			xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
            boError = false;
            break;

        case 1:
			stGsmMsg.ucSrc = SRC_GSM;
			stGsmMsg.ucDest = SRC_GSM;
			stGsmMsg.ucEvent = /*EVENT_GSM_SEND_CIPMUX*/EVENT_GSM_SELECT_SMS_FORMAT/*EVENT_GSM_SEND_CGATT*//*EVENT_GSM_WEB_CONNECT*/;
			xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);

			ucGsmTry = 0;
            boError = true;
            break;

        default:
        	break;
    }
    return(boError);
}

//////////////////////////////////////////////
//
//
//              TaskGsm_SendCgatt
//
//
//////////////////////////////////////////////
unsigned char TaskGsm_SendCgatt(sMessageType *psMessage)
{
    unsigned char boError = true;
    static unsigned char ucResp = 0xFF;

    tstGsmTableResponse const gasTableGsmResponse[] =
    {
    		{	1,		"+CGATT: 1"			},
    		{	2,		"ERROR"				},
    		{  255,    "END OF ARRAY"		}
    };


	TaskGsm_SendAtCmd1("GSM:CGATT?\r\n","AT+CGATT?\r\n");
	ucResp = TaskGsm_WaitResponse1(&gasTableGsmResponse[0],TaskGsm_ParseResp1,1,5);

	switch(ucResp)
    {
    	case 0:
        case 2:
        	if(++ucGsmTry <=10)
        	{
    			stGsmMsg.ucSrc = SRC_GSM;
    			stGsmMsg.ucDest = SRC_GSM;
    			stGsmMsg.ucEvent = EVENT_GSM_SEND_CGATT;
    			xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
        	}
        	else
        	{
        		stGsmMsg.ucSrc = SRC_GSM;
        		stGsmMsg.ucDest = SRC_GSM;
        		stGsmMsg.ucEvent = EVENT_GSM_ERROR;
        		xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
        		boError = false;
        	}
            boError = false;
            break;

        case 1:
			stGsmMsg.ucSrc = SRC_GSM;
			stGsmMsg.ucDest = SRC_GSM;
			stGsmMsg.ucEvent = /*EVENT_GSM_WEB_CONNECT*//*EVENT_GSM_SEND_CGDCONT*/EVENT_GSM_SEND_CIPMUX/*EVENT_GSM_SEND_CIPSHUT*//*EVENT_GSM_SEND_BEARERSET1*/;
			xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
			ucGsmTry = 0;
            boError = true;
        break;
    }

    return(boError);
}



//////////////////////////////////////////////
//
//
//              TaskGsm_SendCipStatus
//
//
//////////////////////////////////////////////
unsigned char TaskGsm_SendCipStatus(sMessageType *psMessage)
{
    unsigned char boError = true;
    static unsigned char ucResp = 0xFF;

    tstGsmTableResponse const gasTableGsmResponse[] =
    {
    		{	1,		"IP INITIAL"		},
    		{	2,		"ERROR"				},
    		{  255,    "END OF ARRAY"		}
    };


	TaskGsm_SendAtCmd1("GSM:CIPSTATUS\r\n","AT+CIPSTATUS\r\n");
	ucResp = TaskGsm_WaitResponse1(&gasTableGsmResponse[0],TaskGsm_ParseResp1,1,5);

    switch(ucResp)
    {
        case 0:
        case 2:
			stGsmMsg.ucSrc = SRC_GSM;
			stGsmMsg.ucDest = SRC_GSM;
			stGsmMsg.ucEvent = EVENT_GSM_SEND_CIPSTATUS;
			xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
            boError = false;
        break;

        case 1:
			stGsmMsg.ucSrc = SRC_GSM;
			stGsmMsg.ucDest = SRC_GSM;
			stGsmMsg.ucEvent = EVENT_GSM_SEND_CIPMUX/*EVENT_GSM_SEND_CGDCONT*/;
			xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
            boError = true;
        break;

        default:
        break;
    }

    return(boError);
}

//////////////////////////////////////////////
//
//
//              TaskGsm_SendCgdcont
//
//
//////////////////////////////////////////////
unsigned char TaskGsm_SendCgdcont(sMessageType *psMessage)
{
    unsigned char boError = true;
    static unsigned char ucResp = 0xFF;
	char* cLocalBuffer = (char*) malloc(RX_BUF_SIZE+1);

    memset(cLocalBuffer,0,RX_BUF_SIZE+1);


#if 0
#if APN == VODAFONE
    sprintf(cLocalBuffer,"AT+CGDCONT=1,\"IP\",\"%s\"\r\n",cApn);
#elif APN == CLARO
    sprintf(cLocalBuffer,"AT+CGDCONT=1,\"IP\",\"claro.com.br\"\r\n");
#elif APN == VIVO
    sprintf(cLocalBuffer,"AT+CGDCONT=1,\"IP\",\"zap.vivo.com.br\"\r\n");
#elif APN == TMDATA
    sprintf(cLocalBuffer,"AT+CGDCONT=1,\"IP\",\"tmdata.claro.com.br\"\r\n");
#else
    sprintf(cLocalBuffer,"AT+CGDCONT=1,\"IP\",\"m2m.vodafone.br\"\r\n");
#endif
#endif
    sprintf(cLocalBuffer,"AT+CGDCONT=1,\"IP\",\"%s\"\r\n",stConfigData.cModemApn);

    static tstGsmTableResponse const gasTableGsmResponse[] =
    {
    		{	1,		"OK"		},
    		{	2,		"ERROR"				},
    		{  255,    "END OF ARRAY"		}
    };


	TaskGsm_SendAtCmd1("GSM:CGDCONT\r\n",cLocalBuffer);
	ucResp = TaskGsm_WaitResponse1(&gasTableGsmResponse[0],TaskGsm_ParseResp1,1,5);

    switch(ucResp)
    {
    	case 0:
        case 2:
			stGsmMsg.ucSrc = SRC_GSM;
			stGsmMsg.ucDest = SRC_GSM;
			stGsmMsg.ucEvent = EVENT_GSM_SEND_CGDCONT;
			xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
            boError = false;
        break;

        case 1:
			stGsmMsg.ucSrc = SRC_GSM;
			stGsmMsg.ucDest = SRC_GSM;
			stGsmMsg.ucEvent = EVENT_GSM_SEND_CIPMUX;
			xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
			ucGsmTry = 0;
            boError = true;
        break;
    }
    free(cLocalBuffer);
    return(boError);
}

//////////////////////////////////////////////
//
//
//              TaskGsm_SendCipMux
//
//
//////////////////////////////////////////////
unsigned char TaskGsm_SendCipMux(sMessageType *psMessage)
{
    unsigned char boError = true;
    static unsigned char ucResp = 0xFF;

    tstGsmTableResponse const gasTableGsmResponse[] =
    {
    		{	1,		"OK"				},
    		{	2,		"ERROR"				},
    		{  255,    "END OF ARRAY"		}
    };


	TaskGsm_SendAtCmd1("GSM:CIPMUX\r\n","AT+CIPMUX=0\r\n");
	ucResp = TaskGsm_WaitResponse1(&gasTableGsmResponse[0],TaskGsm_ParseResp1,1,5);

    switch(ucResp)
    {
		case 0:
		case 2:
		if(++ucGsmTry <=3)
		{
			stGsmMsg.ucSrc = SRC_GSM;
			stGsmMsg.ucDest = SRC_GSM;
			stGsmMsg.ucEvent = EVENT_GSM_SEND_CIPMUX;
			xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
			boError = false;
		}
		else
		{
			stGsmMsg.ucSrc = SRC_GSM;
			stGsmMsg.ucDest = SRC_GSM;
			stGsmMsg.ucEvent = EVENT_GSM_ERROR;
			xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
			boError = false;
		}
		break;

        case 1:
			stGsmMsg.ucSrc = SRC_GSM;
			stGsmMsg.ucDest = SRC_GSM;
			stGsmMsg.ucEvent = /*EVENT_GSM_SEND_CSTT*/EVENT_GSM_SEND_CIPMODE;
			xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
			ucGsmTry = 0;
            boError = true;
        break;


        default:
        break;
    }

    return(boError);
}


//////////////////////////////////////////////
//
//
//              TaskGsm_SendCipMode
//
//
//////////////////////////////////////////////
unsigned char TaskGsm_SendCipMode(sMessageType *psMessage)
{
    unsigned char boError = true;
    static unsigned char ucResp = 0xFF;

    tstGsmTableResponse const gasTableGsmResponse[] =
    {
    		{	1,		"OK"				},
    		{	2,		"ERROR"				},
    		{  255,    "END OF ARRAY"		}
    };


	TaskGsm_SendAtCmd1("GSM:CIPMODE\r\n","AT+CIPMODE=0\r\n");
	ucResp = TaskGsm_WaitResponse1(&gasTableGsmResponse[0],TaskGsm_ParseResp1,1,5);

    switch(ucResp)
    {
		case 0:
		case 2:
		if(++ucGsmTry <=3)
		{
			stGsmMsg.ucSrc = SRC_GSM;
			stGsmMsg.ucDest = SRC_GSM;
			stGsmMsg.ucEvent = EVENT_GSM_SEND_CIPMODE;
			xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
			boError = false;
		}
		else
		{
			stGsmMsg.ucSrc = SRC_GSM;
			stGsmMsg.ucDest = SRC_GSM;
			stGsmMsg.ucEvent = EVENT_GSM_ERROR;
			xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
			boError = false;
		}
		break;

        case 1:
			stGsmMsg.ucSrc = SRC_GSM;
			stGsmMsg.ucDest = SRC_GSM;
			stGsmMsg.ucEvent = EVENT_GSM_SEND_CSTT;
			xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
			ucGsmTry = 0;
            boError = true;
        break;

        default:
        break;
    }

    return(boError);
}


//////////////////////////////////////////////
//
//
//              TaskGsm_SendCstt
//
//
//////////////////////////////////////////////
unsigned char TaskGsm_SendCstt(sMessageType *psMessage)
{
    unsigned char boError = true;
    static unsigned char ucResp = 0xFF;
	char* cLocalBuffer = (char*) malloc(RX_BUF_SIZE+1);

    memset(cLocalBuffer,0,RX_BUF_SIZE+1);
#if 0
#if APN == VODAFONE
	sprintf(cLocalBuffer,"AT+CSTT=\"%s\",\"%s\",\"%s\"\r\n",cApn,cApnLogin,cApnPassword);
#elif APN == CLARO
	sprintf(cLocalBuffer,"AT+CSTT=\"claro.com.br\",\"claro\",\"claro\"\r\n");
#elif APN == VIVO
	sprintf(cLocalBuffer,"AT+CSTT=\"zap.vivo.com.br\",\"vivo\",\"vivo\"\r\n");
#elif APN == TMDATA
    sprintf(cLocalBuffer,"AT+CSTT=\"tmdata.claro.com.br\",\"tmdata\",\"tmdata\"\r\n");
#else
	sprintf(cLocalBuffer,"AT+CSTT=\"m2m.vodafone.br\",\"\",\"\"\r\n");
#endif
#endif
	sprintf(cLocalBuffer,"AT+CSTT=\"%s\",\"%s\",\"%s\"\r\n",stConfigData.cModemApn,stConfigData.cModemApnLogin,stConfigData.cModemApnPassword);

    tstGsmTableResponse const gasTableGsmResponse[] =
    {
    		{	1,		"OK"				},
    		{	2,		"ERROR"				},
    		{  255,    "END OF ARRAY"		}
    };


	TaskGsm_SendAtCmd1("GSM:CSTT\r\n",cLocalBuffer);
	ucResp = TaskGsm_WaitResponse1(&gasTableGsmResponse[0],TaskGsm_ParseResp1,1,5);

    switch(ucResp)
    {
        case 0:
        case 2:
        	if(++ucGsmTry <=3)
        	{
				stGsmMsg.ucSrc = SRC_GSM;
				stGsmMsg.ucDest = SRC_GSM;
				stGsmMsg.ucEvent = EVENT_GSM_SEND_CSTT;
				xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
				boError = false;
        	}
        	else
        	{
				stGsmMsg.ucSrc = SRC_GSM;
				stGsmMsg.ucDest = SRC_GSM;
				stGsmMsg.ucEvent = EVENT_GSM_ERROR;
				xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
				boError = false;
        	}
        break;

        case 1:
			stGsmMsg.ucSrc = SRC_GSM;
			stGsmMsg.ucDest = SRC_GSM;
			stGsmMsg.ucEvent = EVENT_GSM_SEND_CIICR;
			xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
			ucGsmTry = 0;
            boError = true;
        break;

        default:
        break;
    }

    free(cLocalBuffer);
    return(boError);
}


//////////////////////////////////////////////
//
//
//           TaskGsm_SelectSmsFormat
//
//
//////////////////////////////////////////////
unsigned char TaskGsm_SelectSmsFormat(sMessageType *psMessage)
{
    unsigned char boError = true;
    static unsigned char ucResp = 0xFF;

    tstGsmTableResponse const gasTableGsmResponse[] =
    {
    		{	1,		"OK"				},
    		{	2,		"ERROR"				},
    		{  255,    "END OF ARRAY"		}
    };

	TaskGsm_SendAtCmd1("GSM:SMS FORMAT\r\n","AT+CMGF=1\r\n");
	ucResp = TaskGsm_WaitResponse1(&gasTableGsmResponse[0],TaskGsm_ParseResp1,1,5);

    switch(ucResp)
    {
        case 1:
			stGsmMsg.ucSrc = SRC_GSM;
			stGsmMsg.ucDest = SRC_GSM;
			stGsmMsg.ucEvent = /*EVENT_GSM_SEND_CGATT*/EVENT_GSM_LIST_SMS_MSG/*EVENT_GSM_PREPARE_SMS_RESP*/;
			xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
			boError = true;
        break;

		default:
			stGsmMsg.ucSrc = SRC_GSM;
			stGsmMsg.ucDest = SRC_GSM;
			stGsmMsg.ucEvent = EVENT_GSM_SELECT_SMS_FORMAT;
			xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
			boError = false;
		break;
    }
    return(boError);
}

//////////////////////////////////////////////
//
//
//           TaskGsm_ListSmsMsg
//
//
//////////////////////////////////////////////
unsigned char TaskGsm_ListSmsMsg(sMessageType *psMessage)
{
    unsigned char boError = true;
    static unsigned char ucResp = 0xFF;

    tstGsmTableResponse const gasTableGsmResponse[] =
    {
    		{	1,		cCGML				},
    		{	2,		cModem_OK			},
    		{  255,    "END OF ARRAY"		}
    };

	TaskGsm_SendAtCmd1("GSM:CMGL\r\n","AT+CMGL=\"REC UNREAD\",1\r\n");
	ucResp = TaskGsm_WaitResponse1(&gasTableGsmResponse[0],TaskGsm_ParseResp1,1,5);

    switch(ucResp)
    {
        case 1:
            stGsmMsg.ucSrc = SRC_GSM;
            stGsmMsg.ucDest = SRC_GSM;
            stGsmMsg.ucEvent = /*EVENT_GSM_SEND_CGATT*/EVENT_GSM_DECODE_SMS_MSG;
			xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
            boError = true;
        break;

        default:
   			stGsmMsg.ucEvent = EVENT_GSM_SEND_CGATT;
			stGsmMsg.ucSrc = SRC_GSM;
			stGsmMsg.ucDest = SRC_GSM;
			xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
			boError = false;
        break;
    }
    return(boError);
}

//////////////////////////////////////////////
//
//
//              TaskGsm_ParseSmsResp
//
//
//////////////////////////////////////////////
unsigned char TaskGsm_ParseSmsResp(void)
{
    static unsigned char ucResp = 0;
    const char *ptr=NULL;

    SMS_TYPE *ptrSms = &stSmsRecv;

    memset(&stSmsRecv,0x00,sizeof(SMS_TYPE));

    ptr = strstr((const char*)&cGsmRxBuffer[0],(const char*)&cExpectedResp1[0]);

    if(ptr != NULL)
    {
        ucResp = 1;

        ESP_LOGI(GSM_TASK_TAG, "\r\n<<<<RESP:1 GSM>>>>\r\n%s\r\n<<<<>>>>\r\n",cGsmRxBuffer);

        ptr+=(sizeof(cCGML)-1);
        ptr = strtok((char*)ptr,",");
        strcpy(ptrSms->cIndexMsg,ptr);

        ESP_LOGI(GSM_TASK_TAG, "\r\n<<<<INDEX>>>>\r\n%s\r\n<<<<>>>>\r\n",ptrSms->cIndexMsg);

        ptr=strtok(NULL,",");
        ptr=strtok(NULL,",");
        strcpy(ptrSms->cPhoneNumber,ptr);
        ESP_LOGI(GSM_TASK_TAG, "\r\n<<<<PHONE>>>>\r\n%s\r\n<<<<>>>>\r\n",ptrSms->cPhoneNumber);

        ptr=strtok(NULL,",");
        ptr=strtok(NULL,",");
        ptr=strtok(NULL,"\r\n");
        ptr=strtok(NULL,"\r\n");
        strcpy(ptrSms->cMessageText,ptr);
        ESP_LOGI(GSM_TASK_TAG, "\r\n<<<<TEXT>>>>\r\n%s\r\n<<<<>>>>\r\n",ptrSms->cMessageText);
    }
    else
    {
        ptr = strstr((const char*)&cGsmRxBuffer[0],(const char*)&cExpectedResp2[0]);
        if(ptr != NULL)
        {
            ucResp = 2;
        }
    }
    return(ucResp);
}

//////////////////////////////////////////////
//
//
//         TaskGsm_DecodeSmsMsg
//
//
//////////////////////////////////////////////
unsigned char TaskGsm_DecodeSmsMsg(sMessageType *psMessage)
{
    unsigned char boError = true;
    SMS_TYPE *pst = NULL;

    ESP_LOGI(GSM_TASK_TAG, "GSM:DECODE SMS\r\n");

    pst = &stSmsRecv;
	sSmsCommandType const*pstSmsCommand = &gasTaskGsm_SmsCommandTable[0];

	for (;; pstSmsCommand++)
	{
		if((pstSmsCommand->pcCommand != NULL) &&  (strstr((const char*)pst->cMessageText,pstSmsCommand->pcCommand) == NULL))
		{
		   continue;
		}
		return (*pstSmsCommand->FunctionPointer)(psMessage);
	}
	return(boError);
}

//////////////////////////////////////////////
//
//
//              TaskGsm_ParseSmsGetCsq
//
//
//////////////////////////////////////////////
unsigned char TaskGsm_ParseSmsGetCsq(sMessageType *psMessage)
{
	unsigned char boError = true;

	stGsmMsg.ucSrc = SRC_GSM;
	stGsmMsg.ucDest = SRC_GSM;
	stGsmMsg.ucEvent = EVENT_GSM_SEND_SMS_CSQ;
	stGsmMsg.pcMessageData = NULL;
	xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);

	return(boError);
}

//////////////////////////////////////////////
//
//
//              TaskGsm_ParseSmsGetCcid
//
//
//////////////////////////////////////////////
unsigned char TaskGsm_ParseSmsGetCcid(sMessageType *psMessage)
{
	unsigned char boError = true;

	stGsmMsg.ucSrc = SRC_GSM;
	stGsmMsg.ucDest = SRC_GSM;
	stGsmMsg.ucEvent = EVENT_GSM_SEND_SMS_CCID;
	stGsmMsg.pcMessageData = NULL;
	xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);

	return(boError);
}


//////////////////////////////////////////////
//
//
//              TaskGsm_ParseSmsGetGps
//
//
//////////////////////////////////////////////
unsigned char TaskGsm_ParseSmsGetGps(sMessageType *psMessage)
{
	unsigned char boError = true;

	stGsmMsg.ucSrc = SRC_GSM;
	stGsmMsg.ucDest = SRC_GSM;
	stGsmMsg.ucEvent = EVENT_GSM_SEND_SMS_GPS;
	stGsmMsg.pcMessageData = NULL;
	xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);

	return(boError);
}

//////////////////////////////////////////////
//
//
//              TaskGsm_ParseSmsSetConfigPeriodLog
//
//
//////////////////////////////////////////////
/*unsigned char TaskGsm_ParseSmsSetConfigPeriodLog(sMessageType *psMessage)
{
	unsigned char boError = true;
    SMS_TYPE *pst = NULL;

    Debug("GSM:SET CFG PL\r\n");

    pst = &stSmsRecv;




	memset(&cConfigAndData[0],0,sizeof(cConfigAndData));
	sFLASH_ReadBuffer((uint8_t*)&cConfigAndData[0], CONFIG_FILENAME_ADDR, sizeof(CONFIG_FILENAME_ADDR));




	stGsmMsg.ucSrc = SRC_GSM;
	stGsmMsg.ucDest = SRC_GSM;
	stGsmMsg.ucEvent = EVENT_GSM_SEND_SMS_GPS;
	stGsmMsg.pcMessageData = NULL;
	xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);

	return(boError);
}*/
//////////////////////////////////////////////
//
//
//              TaskGsm_SendSmsCsq
//
//
//////////////////////////////////////////////
unsigned char TaskGsm_SendSmsCsq(sMessageType *psMessage)
{
    unsigned char boError = true;
    static unsigned char ucResp = 0xFF;

    tstGsmTableResponse const gasTableGsmResponse[] =
    {
    		{	1,		cCSQ2				},
    		{	2,		cCSQ1				},
    		{  255,    "END OF ARRAY"		}
    };

	TaskGsm_SendAtCmd1("GSM:CSQ\r\n","AT+CSQ\r\n");
	ucResp = TaskGsm_WaitResponse1(&gasTableGsmResponse[0],TaskGsm_ParseResp1,1,5);

    switch(ucResp)
    {
    	case 0:
        case 1:
			stGsmMsg.ucSrc = SRC_GSM;
			stGsmMsg.ucDest = SRC_GSM;
			stGsmMsg.ucEvent = EVENT_GSM_SEND_CSQ;
			xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
            boError = false;
            break;

        case 2:
			stGsmMsg.ucSrc = SRC_GSM;
			stGsmMsg.ucDest = SRC_GSM;
			stGsmMsg.ucEvent = EVENT_GSM_PREPARE_SMS_RESP;

			memset(&cGsmSmsBuffer[0],0,RX_BUF_SIZE+1);
			strcpy(&cGsmSmsBuffer[0],&cGsmRxBuffer[0]);

			xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
			ucGsmTry = 0;
            boError = true;
            break;

        default:
        	break;
    }
    return(boError);
}

//////////////////////////////////////////////
//
//
//              TaskGsm_SendSmsCcid
//
//
//////////////////////////////////////////////
unsigned char TaskGsm_SendSmsCcid(sMessageType *psMessage)
{
    unsigned char boError = true;
    static unsigned char ucResp = 0xFF;

    tstGsmTableResponse const gasTableGsmResponse[] =
    {
    		{	1,		cModem_ERROR		},
    		{	2,		cModem_OK			},
    		{  255,    "END OF ARRAY"		}
    };

	TaskGsm_SendAtCmd1("GSM:CCID\r\n","AT+CCID\r\n");
	ucResp = TaskGsm_WaitResponse1(&gasTableGsmResponse[0],TaskGsm_ParseResp1,1,10);

    switch(ucResp)
    {
    	case 0:
        case 1:
			stGsmMsg.ucSrc = SRC_GSM;
			stGsmMsg.ucDest = SRC_GSM;
			stGsmMsg.ucEvent = EVENT_GSM_SEND_CCID;
			xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
            boError = false;
            break;

        case 2:
			stGsmMsg.ucSrc = SRC_GSM;
			stGsmMsg.ucDest = SRC_GSM;
			stGsmMsg.ucEvent = EVENT_GSM_PREPARE_SMS_RESP;

			memset(&cGsmSmsBuffer[0],0,RX_BUF_SIZE+1);
			strcpy(&cGsmSmsBuffer[0],&cGsmRxBuffer[0]);

			xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
			ucGsmTry = 0;
            boError = true;
            break;

        default:
        	break;
    }
    return(boError);
}

//////////////////////////////////////////////
//
//
//              TaskGsm_SendSmsGps
//
//
//////////////////////////////////////////////
unsigned char TaskGsm_SendSmsGps(sMessageType *psMessage)
{
    unsigned char boError = true;

    ESP_LOGI(GSM_TASK_TAG, "GSM:GPS SMS\r\n");

	stGsmMsg.ucSrc = SRC_GSM;
	stGsmMsg.ucDest = SRC_GSM;
	stGsmMsg.ucEvent = EVENT_GSM_PREPARE_SMS_RESP;

	memset(&cGsmSmsBuffer[0],0,RX_BUF_SIZE+1);
	sprintf(&cGsmSmsBuffer[0],"https://www.google.com/maps/?q=%s,%s",stIo.cLatitude,stIo.cLongitude);
	xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);


    return(boError);
}

//////////////////////////////////////////////
//
//
//           TaskGsm_PrepareSmsResponse
//
//
//////////////////////////////////////////////
unsigned char TaskGsm_PrepareSmsResponse(sMessageType *psMessage)
{
    unsigned char boError = true;
    static unsigned char ucResp = 0xFF;

	char* cLocalBuffer = (char*) malloc(RX_BUF_SIZE+1);
    memset(cLocalBuffer,0,RX_BUF_SIZE+1);

    sprintf((char*)cLocalBuffer,"AT+CMGS=%s\r\n",&stSmsRecv.cPhoneNumber[0]);

    tstGsmTableResponse const gasTableGsmResponse[] =
    {
    		{	1,		">"					},
    		{	2,		cModem_ERROR		},
    		{  255,    "END OF ARRAY"		}
    };

	TaskGsm_SendAtCmd1("GSM:CMGS\r\n",cLocalBuffer);
	ucResp = TaskGsm_WaitResponse1(&gasTableGsmResponse[0],TaskGsm_ParseResp1,1,10);

    switch(ucResp)
    {
        case 1:
            stGsmMsg.ucSrc = SRC_GSM;
            stGsmMsg.ucDest = SRC_GSM;
            stGsmMsg.ucEvent = EVENT_GSM_SEND_SMS_RESP;
			xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
            boError = true;
        break;

        default:
			stGsmMsg.ucSrc = SRC_GSM;
			stGsmMsg.ucDest = SRC_GSM;
			stGsmMsg.ucEvent = EVENT_GSM_SEND_CGATT;
			xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
			boError = false;
        break;
    }
    free(cLocalBuffer);

    return(boError);
}

//////////////////////////////////////////////
//
//
//           TaskGsm_SendSmsResponse
//
//
//////////////////////////////////////////////
#define CTRL_Z 26
unsigned char TaskGsm_SendSmsResponse(sMessageType *psMessage)
{
    unsigned char boError = true;
    unsigned int u16Len;

    u16Len = strlen(cGsmSmsBuffer);
    cGsmSmsBuffer[u16Len]=CTRL_Z;

    TaskGsm_SendAtCmd("GSM:SEND SMS\r\n",cGsmSmsBuffer,"+CMGS:",cModem_ERROR);

    static unsigned char ucResp = 0xFF;
    ucResp = TaskGsm_WaitResponse(TaskGsm_ParseResp,1,20);

    switch(ucResp)
    {
        case 1:
            stGsmMsg.ucSrc = SRC_GSM;
            stGsmMsg.ucDest = SRC_GSM;
            stGsmMsg.ucEvent = EVENT_GSM_DELETE_SMS_MSG;
			xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
            boError = true;
        break;

        default:
			stGsmMsg.ucSrc = SRC_GSM;
			stGsmMsg.ucDest = SRC_GSM;
			stGsmMsg.ucEvent = EVENT_GSM_SEND_CGATT;
			xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
			boError = false;

        break;
    }
    return(boError);

}

//////////////////////////////////////////////
//
//
//           TaskGsm_DeleteSmsMsg
//
//
//////////////////////////////////////////////
unsigned char TaskGsm_DeleteSmsMsg(sMessageType *psMessage)
{
    unsigned char boError = true;
    SMS_TYPE *ptrSms = &stSmsRecv;

	char* cLocalBuffer = (char*) malloc(RX_BUF_SIZE+1);

    memset(cLocalBuffer,0,RX_BUF_SIZE+1);
    strcpy(cLocalBuffer,"AT+CMGD=");
    strcat(cLocalBuffer,ptrSms->cIndexMsg);
    strcat(cLocalBuffer,",0\r\n");

    TaskGsm_SendAtCmd("GSM:CMGD\r\n",cLocalBuffer,cModem_OK,cModem_ERROR);

    static unsigned char ucResp = 0xFF;
    ucResp = TaskGsm_WaitResponse(TaskGsm_ParseResp,1, 20);

    switch(ucResp)
    {
        case 1:
        	stGsmMsg.ucSrc = SRC_GSM;
			stGsmMsg.ucDest = SRC_GSM;
			stGsmMsg.ucEvent = EVENT_GSM_SEND_CGATT;
			xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
            boError = true;

        break;

        default:
        	stGsmMsg.ucSrc = SRC_GSM;
			stGsmMsg.ucDest = SRC_GSM;
			stGsmMsg.ucEvent = EVENT_GSM_DELETE_SMS_MSG;
			xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
            boError = false;
        break;
    }

    free(cLocalBuffer);
    return(boError);
}

#if 0
//////////////////////////////////////////////
//
//
//              TaskModem_SendCsq
//
//
//////////////////////////////////////////////
unsigned char TaskModem_SendCsq(sMessageType *psMessage)
{
    unsigned char boError = true;

    TaskGsm_SendAtCmd("CSQ\r\n","AT+CSQ\r\n",cCSQ2,cCSQ1);

    unsigned char ucResp;
    ucResp = TaskGsm_WaitResponse(TaskGsm_ParseResp,1, 10);

    memset(&cBufferRxModemTemporario[0],0x00,sizeof(cBufferRxModemTemporario));
    strcpy(&cBufferRxModemTemporario[0],(const char*)&cBufferRxModem[0]);

    switch(ucResp)
    {
        case 2:
            if(strlen(cBufferRxModemTemporario)>0)
            {
                if(psMessage->ucSrc == SRC_GSM)
                {
                    stGsmMsg.ucSrc = SRC_GSM;
                    stGsmMsg.ucDest = SRC_GSM;
                    stGsmMsg.ucEvent = EVENT_GSM_PREPARE_SMS_RESP;
                    stGsmMsg.pcMessageData = &cBufferRxModemTemporario[0];
                    queueModem.put(&stGsmMsg);
                }
                else
                {
                    if(psMessage->ucSrc == SRC_BT)
                    {
                        stGsmMsg.ucSrc = SRC_GSM;
                        stGsmMsg.ucDest = SRC_BT;
                        stGsmMsg.ucEvent = EVENT_BT_SEND_RESP;
                        stGsmMsg.pcMessageData = &cBufferRxModemTemporario[0];
                        queueBt.put(&stGsmMsg);
                    }
                }
            }

            boError = true;
        break;

        default:
        if(strlen(cBufferRxModemTemporario)>0)
        {
            if(psMessage->ucSrc == SRC_GSM)
            {
                stGsmMsg.ucSrc = SRC_GSM;
                stGsmMsg.ucDest = SRC_GSM;
                stGsmMsg.ucEvent = EVENT_GSM_PREPARE_SMS_RESP;
                stGsmMsg.pcMessageData = &cBufferRxModemTemporario[0];
                queueModem.put(&stGsmMsg);
            }
            else
            {
                if(psMessage->ucSrc == SRC_BT)
                {
                    stGsmMsg.ucSrc = SRC_GSM;
                    stGsmMsg.ucDest = SRC_BT;
                    stGsmMsg.ucEvent = EVENT_BT_SEND_RESP;
                    stGsmMsg.pcMessageData = &cBufferRxModemTemporario[0];
                    queueBt.put(&stGsmMsg);
                }
            }
        }
        break;
    }
    return(boError);
}
#endif

#if 0
//////////////////////////////////////////////
//
//
//              TaskModem_SendCcid
//
//
//////////////////////////////////////////////
unsigned char TaskModem_SendCcid(sMessageType *psMessage)
{
    unsigned char boError = true;

    TaskGsm_SendAtCmd("CCID\r\n","AT+CCID\r\n",cModem_OK,cModem_ERROR);

    unsigned char ucResp;
    ucResp = TaskGsm_WaitResponse(TaskGsm_ParseResp,1, 10);

    memset(&cBufferRxModemTemporario[0],0x00,sizeof(cBufferRxModemTemporario));
    strcpy(&cBufferRxModemTemporario[0],(const char*)&cBufferRxModem[0]);

    switch(ucResp)
    {
        case 1:
            if(strlen(cBufferRxModemTemporario)>0)
            {
                if(psMessage->ucSrc == SRC_GSM)
                {
                    stGsmMsg.ucSrc = SRC_GSM;
                    stGsmMsg.ucDest = SRC_GSM;
                    stGsmMsg.ucEvent = EVENT_GSM_PREPARE_SMS_RESP;
                    stGsmMsg.pcMessageData = &cBufferRxModemTemporario[0];
                    queueModem.put(&stGsmMsg);
                }
                else
                {
                    if(psMessage->ucSrc == SRC_BT)
                    {
                        stGsmMsg.ucSrc = SRC_GSM;
                        stGsmMsg.ucDest = SRC_BT;
                        stGsmMsg.ucEvent = EVENT_BT_SEND_RESP;
                        stGsmMsg.pcMessageData = &cBufferRxModemTemporario[0];
                        queueBt.put(&stGsmMsg);
                    }
                }
            }

            boError = true;
        break;

        default:
            stGsmMsg.ucSrc = SRC_GSM;
            stGsmMsg.ucDest = SRC_GSM;
            stGsmMsg.ucEvent = EVENT_GSM_SEND_CCID;
            stGsmMsg.pcMessageData = "";
            queueModem.put(&stGsmMsg);
        break;
    }
    return(boError);
}
#endif

#if 0
//////////////////////////////////////////////
//
//
//              TaskModem_ReadCsq
//
//
//////////////////////////////////////////////
unsigned char TaskModem_ReadCsq(sMessageType *psMessage)
{
    unsigned char boError = true;

    TaskGsm_SendAtCmd("GSM:CSQ\r\n","AT+CSQ\r\n",cCSQ2,cCSQ1);

    unsigned char ucResp;
    ucResp = TaskGsm_WaitResponse(TaskGsm_ParseResp,1, 10);

    switch(ucResp)
    {
        case 2:
            stGsmMsg.ucSrc = SRC_GSM;
            stGsmMsg.ucDest = SRC_GSM;
            stGsmMsg.ucEvent = EVENT_GSM_PREPARE_SMS_RESP;
            stGsmMsg.pcMessageData = (psMessage->pcMessageData);
            queueModem.put(&stGsmMsg);
            boError = true;
        break;

        default:
            stGsmMsg.ucSrc = SRC_GSM;
            stGsmMsg.ucDest = SRC_GSM;
            stGsmMsg.ucEvent = EVENT_GSM_READ_CSQ;
            stGsmMsg.pcMessageData = "";

            queueModem.put(&stGsmMsg);
            boError = false;
        break;
    }
    return(boError);
}
#endif




//////////////////////////////////////////////
//
//
//              TaskGsm_WebConnect
//
//
//////////////////////////////////////////////
unsigned char TaskGsm_WebConnect(sMessageType *psMessage)
{
	unsigned char boError = true;

    ESP_LOGI(GSM_TASK_TAG, "GSM:WEB CONNECTED\r\n");

	stGsmMsg.ucSrc = SRC_GSM;
	stGsmMsg.ucDest = SRC_SD;
	stGsmMsg.ucEvent = EVENT_SD_OPENING;
	xQueueSend( xQueueSd, ( void * )&stGsmMsg, 0);

	return(boError);
}

//////////////////////////////////////////////
//
//
//              TaskGsm_OpeningFileOk
//
//
//////////////////////////////////////////////
unsigned char TaskGsm_OpeningFileOk(sMessageType *psMessage)
{
	unsigned char boError = true;

    ESP_LOGI(GSM_TASK_TAG, "GSM:OPENING FILE OK\r\n");

	/*stGsmMsg.ucSrc = SRC_GSM;
	stGsmMsg.ucDest = SRC_GSM;
	stGsmMsg.ucEvent = EVENT_GSM_SEND_START;
	xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);*/


	stGsmMsg.ucSrc = SRC_GSM;
	stGsmMsg.ucDest = SRC_SD;
	stGsmMsg.ucEvent = EVENT_SD_READING;
	xQueueSend( xQueueSd, ( void * )&stGsmMsg, 0);

    ucGsmTry = 0;

    return(boError);
}


//////////////////////////////////////////////
//
//
//              TaskGsm_SendBearerSet1
//
//
//////////////////////////////////////////////

unsigned char TaskGsm_SendBearerSet1(sMessageType *psMessage)
{
    unsigned char boError = true;
    static unsigned char ucResp = 0xFF;

    tstGsmTableResponse const gasTableGsmResponse[] =
    {
    		{	1,		cModem_OK			},
    		{	2,		cModem_ERROR		},
    		{  255,    "END OF ARRAY"		}
    };

	TaskGsm_SendAtCmd1("GSM:BEARER SET1\r\n","AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"\r\n");
	ucResp = TaskGsm_WaitResponse1(&gasTableGsmResponse[0],TaskGsm_ParseResp1,1,10);

	switch(ucResp)
    {
        case 0:
        case 2:
        	if(++ucGsmTry<=3)
        	{
				stGsmMsg.ucSrc = SRC_GSM;
				stGsmMsg.ucDest = SRC_GSM;
				stGsmMsg.ucEvent = EVENT_GSM_SEND_BEARERSET1;
				xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
        	}
        	else
        	{
				stGsmMsg.ucSrc = SRC_GSM;
				stGsmMsg.ucDest = SRC_GSM;
				stGsmMsg.ucEvent = EVENT_GSM_ERROR;
				xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
        	}
            boError = false;
        break;

        case 1:
			stGsmMsg.ucSrc = SRC_GSM;
			stGsmMsg.ucDest = SRC_GSM;
			stGsmMsg.ucEvent = EVENT_GSM_SEND_BEARERSET2;
			xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
			ucGsmTry = 0;
            boError = true;
        break;

        default:
        break;
    }
    return(boError);
}

//////////////////////////////////////////////
//
//
//              TaskGsm_SendBearerSet2
//
//
//////////////////////////////////////////////
unsigned char TaskGsm_SendBearerSet2(sMessageType *psMessage)
{
    unsigned char boError = true;
    static unsigned char ucResp = 0xFF;
	char* cLocalBuffer = (char*) malloc(RX_BUF_SIZE+1);

    memset(cLocalBuffer,0,RX_BUF_SIZE+1);
#if APN == VODAFONE
    sprintf(cLocalBuffer,"AT+SAPBR=3,1,\"APN\",\"%s\"\r\n",cApn);
#elif APN == CLARO
    sprintf(cLocalBuffer,"AT+SAPBR=3,1,\"APN\",\"claro.com.br\"\r\n");
#elif APN == VIVO
    sprintf(cLocalBuffer,"AT+SAPBR=3,1,\"APN\",\"zap.vivo.com.br\"\r\n");
#elif APN == TMDATA
    sprintf(cLocalBuffer,"AT+SAPBR=3,1,\"APN\",\"tmdata.claro.com.br\"\r\n");
#else
    sprintf(cLocalBuffer,"AT+SAPBR=3,1,\"APN\",\"m2m.vodafone.br\"\r\n");
#endif

    tstGsmTableResponse const gasTableGsmResponse[] =
    {
    		{	1,		cModem_OK			},
    		{	2,		cModem_ERROR		},
    		{  255,    "END OF ARRAY"		}
    };

	TaskGsm_SendAtCmd1("GSM:BEARER SET2\r\n",cLocalBuffer);
	ucResp = TaskGsm_WaitResponse1(&gasTableGsmResponse[0],TaskGsm_ParseResp1,1,10);

	switch(ucResp)
    {
        case 0:
        case 2:
        	if(++ucGsmTry<=3)
        	{
				stGsmMsg.ucSrc = SRC_GSM;
				stGsmMsg.ucDest = SRC_GSM;
				stGsmMsg.ucEvent = EVENT_GSM_SEND_BEARERSET2;
				xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
        	}
        	else
        	{
				stGsmMsg.ucSrc = SRC_GSM;
				stGsmMsg.ucDest = SRC_GSM;
				stGsmMsg.ucEvent = EVENT_GSM_ERROR;
				xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
        	}
            boError = false;
        break;

        case 1:
			stGsmMsg.ucSrc = SRC_GSM;
			stGsmMsg.ucDest = SRC_GSM;
			stGsmMsg.ucEvent = EVENT_GSM_SEND_BEARERSET21;
			xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
			ucGsmTry = 0;
            boError = true;
        break;

        default:
        break;
    }
    return(boError);
}

//////////////////////////////////////////////
//
//
//              TaskGsm_SendBearerSet21
//
//
//////////////////////////////////////////////
unsigned char TaskGsm_SendBearerSet21(sMessageType *psMessage)
{
    unsigned char boError = true;
    static unsigned char ucResp = 0xFF;
	char* cLocalBuffer = (char*) malloc(RX_BUF_SIZE+1);

    memset(cLocalBuffer,0,RX_BUF_SIZE+1);

#if APN ==VODAFONE
    sprintf(cLocalBuffer,"AT+SAPBR=3,1,\"USER\",\"%s\"\r\n",cApnLogin);
#elif APN == CLARO
    sprintf(cLocalBuffer,"AT+SAPBR=3,1,\"USER\",\"claro\"\r\n");
#elif APN == VIVO
    sprintf(cLocalBuffer,"AT+SAPBR=3,1,\"USER\",\"vivo\"\r\n");
#elif APN == TMDATA
    sprintf(cLocalBuffer,"AT+SAPBR=3,1,\"USER\",\"tmdata\"\r\n");
#else
    sprintf(cLocalBuffer,"AT+SAPBR=3,1,\"USER\",\"\"\r\n");
#endif


    tstGsmTableResponse const gasTableGsmResponse[] =
    {
    		{	1,		cModem_OK			},
    		{	2,		cModem_ERROR		},
    		{  255,    "END OF ARRAY"		}
    };

	TaskGsm_SendAtCmd1("GSM:BEARER SET21\r\n",cLocalBuffer);
	ucResp = TaskGsm_WaitResponse1(&gasTableGsmResponse[0],TaskGsm_ParseResp1,1,10);

    switch(ucResp)
    {
        case 0:
        case 2:
        	if(++ucGsmTry<=3)
        	{
				stGsmMsg.ucSrc = SRC_GSM;
				stGsmMsg.ucDest = SRC_GSM;
				stGsmMsg.ucEvent = EVENT_GSM_SEND_BEARERSET21;
				xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
        	}
        	else
        	{
				stGsmMsg.ucSrc = SRC_GSM;
				stGsmMsg.ucDest = SRC_GSM;
				stGsmMsg.ucEvent = EVENT_GSM_ERROR;
				xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
        	}
            boError = false;
        break;

        case 1:
			stGsmMsg.ucSrc = SRC_GSM;
			stGsmMsg.ucDest = SRC_GSM;
			stGsmMsg.ucEvent = EVENT_GSM_SEND_BEARERSET22;
			xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
			ucGsmTry = 0;
            boError = true;
        break;

        default:
        break;
    }
    return(boError);
}
//////////////////////////////////////////////
//
//
//              TaskGsm_SendBearerSet22
//
//
//////////////////////////////////////////////
unsigned char TaskGsm_SendBearerSet22(sMessageType *psMessage)
{
    unsigned char boError = true;
    static unsigned char ucResp = 0xFF;
	char* cLocalBuffer = (char*) malloc(RX_BUF_SIZE+1);

    memset(cLocalBuffer,0,RX_BUF_SIZE+1);

#if APN == VODAFONE
    sprintf(cLocalBuffer,"AT+SAPBR=3,1,\"PWD\",\"%s\"\r\n",cApnPassword);
#elif APN == CLARO
    sprintf(cLocalBuffer,"AT+SAPBR=3,1,\"PWD\",\"claro\"\r\n");
#elif APN == VIVO
    sprintf(cLocalBuffer,"AT+SAPBR=3,1,\"PWD\",\"vivo\"\r\n");
#elif APN == TMDATA
    sprintf(cLocalBuffer,"AT+SAPBR=3,1,\"PWD\",\"tmdata\"\r\n");
#else
    sprintf(cLocalBuffer,"AT+SAPBR=3,1,\"PWD\",\"\"\r\n");
#endif

    tstGsmTableResponse const gasTableGsmResponse[] =
    {
    		{	1,		cModem_OK			},
    		{	2,		cModem_ERROR		},
    		{  255,    "END OF ARRAY"		}
    };

	TaskGsm_SendAtCmd1("GSM:BEARER SET22\r\n",cLocalBuffer);
	ucResp = TaskGsm_WaitResponse1(&gasTableGsmResponse[0],TaskGsm_ParseResp1,1,10);

    switch(ucResp)
    {
        case 0:
        case 2:
        	if(++ucGsmTry<=3)
        	{
				stGsmMsg.ucSrc = SRC_GSM;
				stGsmMsg.ucDest = SRC_GSM;
				stGsmMsg.ucEvent = EVENT_GSM_SEND_BEARERSET22;
				xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
        	}
        	else
        	{
				stGsmMsg.ucSrc = SRC_GSM;
				stGsmMsg.ucDest = SRC_GSM;
				stGsmMsg.ucEvent = EVENT_GSM_ERROR;
				xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
        	}
            boError = false;
        break;

        case 1:
			stGsmMsg.ucSrc = SRC_GSM;
			stGsmMsg.ucDest = SRC_GSM;
			stGsmMsg.ucEvent = EVENT_GSM_SEND_BEARERSET3/*EVENT_GSM_SEND_CSTT*//*EVENT_GSM_SEND_CIICR*/;
			xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
			ucGsmTry = 0;
            boError = true;
        break;

        default:
        break;
    }
    return(boError);
}



//////////////////////////////////////////////
//
//
//              TaskGsm_SendCiicr
//
//
//////////////////////////////////////////////
unsigned char TaskGsm_SendCiicr(sMessageType *psMessage)
{
    unsigned char boError = true;
    static unsigned char ucResp = 0xFF;

    tstGsmTableResponse const gasTableGsmResponse[] =
    {
    		{	1,		cModem_OK			},
    		{	2,		cCIICR1				},
    		{  255,    "END OF ARRAY"		}
    };

	TaskGsm_SendAtCmd1("GSM:CIICR\r\n","AT+CIICR\r\n");
	ucResp = TaskGsm_WaitResponse1(&gasTableGsmResponse[0],TaskGsm_ParseResp1,1,30);

    switch(ucResp)
    {
        case 0:
        	if(++ucGsmTry<2)
        	{
				stGsmMsg.ucSrc = SRC_GSM;
				stGsmMsg.ucDest = SRC_GSM;
				stGsmMsg.ucEvent = EVENT_GSM_SEND_CIICR;
				xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
        	}
        	else
        	{
				stGsmMsg.ucSrc = SRC_GSM;
				stGsmMsg.ucDest = SRC_GSM;
				stGsmMsg.ucEvent = /*EVENT_GSM_SEND_CSTT*/EVENT_GSM_ERROR;
				xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
        	}
            boError = false;
            break;

        case 2:
			stGsmMsg.ucSrc = SRC_GSM;
			stGsmMsg.ucDest = SRC_GSM;
			stGsmMsg.ucEvent = /*EVENT_GSM_SEND_CSTT*/EVENT_GSM_ERROR;
			xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);

            boError = false;
            break;

        case 1:
			stGsmMsg.ucSrc = SRC_GSM;
			stGsmMsg.ucDest = SRC_GSM;
			stGsmMsg.ucEvent = EVENT_GSM_SEND_CIFSR;
			xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
			ucGsmTry = 0;
            boError = true;
        break;

        default:
        break;
    }

    return(boError);
}

//////////////////////////////////////////////
//
//
//              TaskGsm_SendCifsr
//
//
//////////////////////////////////////////////
unsigned char TaskGsm_SendCifsr(sMessageType *psMessage)
{
    unsigned char boError = true;
    static unsigned char ucResp = 0xFF;

    tstGsmTableResponse const gasTableGsmResponse[] =
    {
    		{	1,		"."					},
    		{	2,		cModem_ERROR		},
    		{  255,    "END OF ARRAY"		}
    };

	TaskGsm_SendAtCmd1("GSM:CIFSR\r\n","AT+CIFSR\r\n");
	ucResp = TaskGsm_WaitResponse1(&gasTableGsmResponse[0],TaskGsm_ParseResp1,1,10);

    switch(ucResp)
    {
        case 0:
        case 2:
        	if(++ucGsmTry <=3)
        	{
				stGsmMsg.ucSrc = SRC_GSM;
				stGsmMsg.ucDest = SRC_GSM;
				stGsmMsg.ucEvent = EVENT_GSM_SEND_CIFSR;
				xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
				boError = false;
        	}
        	else
        	{
				stGsmMsg.ucSrc = SRC_GSM;
				stGsmMsg.ucDest = SRC_GSM;
				stGsmMsg.ucEvent = EVENT_GSM_ERROR;
				xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
        	}
        break;

        case 1:
			stGsmMsg.ucSrc = SRC_GSM;
			stGsmMsg.ucDest = SRC_GSM;
			stGsmMsg.ucEvent = /*EVENT_GSM_SEND_START*//*EVENT_GSM_WEB_CONNECT*//*EVENT_GSM_SEND_BEARERSET3*/EVENT_GSM_SEND_BEARERSET1;
			xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
			ucGsmTry = 0;
            boError = true;
        break;

        default:
        break;
    }

    return(boError);
}

//////////////////////////////////////////////
//
//
//              TaskGsm_SendCipshut
//
//
//////////////////////////////////////////////
unsigned char TaskGsm_SendCipshut(sMessageType *psMessage)
{
    unsigned char boError = true;
    static unsigned char ucResp = 0xFF;

    tstGsmTableResponse const gasTableGsmResponse[] =
    {
    		{	1,		cModem_OK			},
    		{	2,		cModem_ERROR		},
    		{  255,    "END OF ARRAY"		}
    };

	TaskGsm_SendAtCmd1("GSM:CIPSHUT\r\n","AT+CIPSHUT\r\n");
	ucResp = TaskGsm_WaitResponse1(&gasTableGsmResponse[0],TaskGsm_ParseResp1,1,10);

    switch(ucResp)
    {
        case 0:
        case 2:
        	if(++ucGsmTry <=3)
        	{
				stGsmMsg.ucSrc = SRC_GSM;
				stGsmMsg.ucDest = SRC_GSM;
				stGsmMsg.ucEvent = EVENT_GSM_SEND_CIPSHUT;
				xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
				boError = false;
        	}
        	else
        	{
				stGsmMsg.ucSrc = SRC_GSM;
				stGsmMsg.ucDest = SRC_GSM;
				stGsmMsg.ucEvent = EVENT_GSM_ERROR;
				xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
				boError = false;
        	}

        break;

        case 1:
			stGsmMsg.ucSrc = SRC_GSM;
			stGsmMsg.ucDest = SRC_GSM;
			stGsmMsg.ucEvent = EVENT_GSM_SEND_CIPSTATUS;
			xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
            boError = true;
            ucGsmTry = 0;
        break;

        default:
        break;
    }

    return(boError);
}

//////////////////////////////////////////////
//
//
//              TaskGsm_GetBearer
//
//
//////////////////////////////////////////////
unsigned char TaskGsm_GetBearer(sMessageType *psMessage)
{
    unsigned char boError = true;
#if 0
    TaskGsm_SendAtCmd("GSM:BEARER GET\r\n","AT+SAPBR=2,1\r\n","1,3,\"0.0.0.0\"",cModem_ERROR);

    unsigned char ucResp;
    ucResp = TaskGsm_WaitResponse(TaskGsm_ParseResp,1, 10);

    switch(ucResp)
    {
        case 1:
			stGsmMsg.ucSrc = SRC_GSM;
			stGsmMsg.ucDest = SRC_GSM;
			stGsmMsg.ucEvent = EVENT_GSM_SEND_BEARERSET3;
			xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
			boError = false;
        break;

        case 0:
        case 2:
			stGsmMsg.ucSrc = SRC_GSM;
			stGsmMsg.ucDest = SRC_GSM;
			stGsmMsg.ucEvent = EVENT_GSM_ERROR;
			xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
			ucGsmTry = 0;
            boError = true;
        break;

        default:
			stGsmMsg.ucSrc = SRC_GSM;
			stGsmMsg.ucDest = SRC_GSM;
			stGsmMsg.ucEvent = EVENT_GSM_SEND_HTTPINIT;
			xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
			ucGsmTry = 0;
            boError = true;
        break;

    }
#endif
    return(boError);
}


//////////////////////////////////////////////
//
//
//              TaskGsm_SendBearerSet3
//
//
//////////////////////////////////////////////
unsigned char TaskGsm_SendBearerSet3(sMessageType *psMessage)
{
    unsigned char boError = true;
    static unsigned char ucResp = 0xFF;

    tstGsmTableResponse const gasTableGsmResponse[] =
    {
    		{	1,		cModem_OK			},
    		{	2,		cModem_ERROR		},
    		{  255,    "END OF ARRAY"		}
    };

	TaskGsm_SendAtCmd1("GSM:BEARER SET3\r\n","AT+SAPBR=1,1\r\n");
	ucResp = TaskGsm_WaitResponse1(&gasTableGsmResponse[0],TaskGsm_ParseResp1,1,10);

	switch(ucResp)
    {
        case 0:
        case 2:
        	if(++ucGsmTry <=5)
        	{
				stGsmMsg.ucSrc = SRC_GSM;
				stGsmMsg.ucDest = SRC_GSM;
				stGsmMsg.ucEvent = EVENT_GSM_SEND_BEARERSET3;
				xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
        	}
        	else
        	{
				stGsmMsg.ucSrc = SRC_GSM;
				stGsmMsg.ucDest = SRC_GSM;
				stGsmMsg.ucEvent = EVENT_GSM_ERROR;
				xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
        	}
			boError = false;
        break;

        case 1:
			stGsmMsg.ucSrc = SRC_GSM;
			stGsmMsg.ucDest = SRC_GSM;
			stGsmMsg.ucEvent = EVENT_GSM_SEND_HTTPINIT;
			xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
			ucGsmTry = 0;
            boError = true;
        break;

        default:
        break;
    }


    return(boError);
}

//////////////////////////////////////////////
//
//
//              TaskGsm_SendBearerSet33
//
//
//////////////////////////////////////////////
unsigned char TaskGsm_SendBearerSet33(sMessageType *psMessage)
{
    unsigned char boError = true;
#if 0
    TaskGsm_SendAtCmd("GSM:BEARER SET33\r\n","AT+SAPBR=0,1\r\n",cModem_OK,cModem_ERROR);

    unsigned char ucResp;
    ucResp = TaskGsm_WaitResponse(TaskGsm_ParseResp,1, 10);

    switch(ucResp)
    {
        case 0:
        case 2:
        	if(++ucGsmTry<=3)
        	{
				stGsmMsg.ucSrc = SRC_GSM;
				stGsmMsg.ucDest = SRC_GSM;
				stGsmMsg.ucEvent = EVENT_GSM_SEND_BEARERSET33;
				xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
        	}
        	else
        	{
				stGsmMsg.ucSrc = SRC_GSM;
				stGsmMsg.ucDest = SRC_GSM;
				stGsmMsg.ucEvent = EVENT_GSM_SEND_BEARERSET1;
				xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
        	}
            boError = false;
        break;

        case 1:
			stGsmMsg.ucSrc = SRC_GSM;
			stGsmMsg.ucDest = SRC_GSM;
			stGsmMsg.ucEvent = EVENT_GSM_SEND_HTTPINIT;
			xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
			ucGsmTry = 0;
            boError = true;
        break;

        default:
        break;
    }

#endif
    return(boError);
}

//////////////////////////////////////////////
//
//
//              TaskGsm_SendHttpInit
//
//
//////////////////////////////////////////////
unsigned char TaskGsm_SendHttpInit(sMessageType *psMessage)
{
    unsigned char boError = true;
    static unsigned char ucResp = 0xFF;

    tstGsmTableResponse const gasTableGsmResponse[] =
    {
    		{	1,		cModem_OK			},
    		{	2,		cModem_ERROR		},
    		{  255,    "END OF ARRAY"		}
    };

	TaskGsm_SendAtCmd1("GSM:HTTP INIT\r\n","AT+HTTPINIT\r\n");
	ucResp = TaskGsm_WaitResponse1(&gasTableGsmResponse[0],TaskGsm_ParseResp1,1,10);

    switch(ucResp)
    {
        case 0:
        case 2:
        	if(++ucGsmTry <= 3)
        	{
				stGsmMsg.ucSrc = SRC_GSM;
				stGsmMsg.ucDest = SRC_GSM;
				stGsmMsg.ucEvent = EVENT_GSM_SEND_HTTPINIT;
				xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
        	}
        	else
        	{
				stGsmMsg.ucSrc = SRC_GSM;
				stGsmMsg.ucDest = SRC_GSM;
				stGsmMsg.ucEvent = EVENT_GSM_ERROR;
				xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
        	}
            boError = false;
        break;

        case 1:
			stGsmMsg.ucSrc = SRC_GSM;
			stGsmMsg.ucDest = SRC_GSM;
			stGsmMsg.ucEvent = EVENT_GSM_SEND_HTTPCID;
			xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
        	ucGsmTry = 0;
            boError = true;
        break;

        default:
        break;
    }
    return(boError);
}

//////////////////////////////////////////////
//
//
//              TaskGsm_SendHttpCid
//
//
//////////////////////////////////////////////
unsigned char TaskGsm_SendHttpCid(sMessageType *psMessage)
{
    unsigned char boError = true;
    static unsigned char ucResp = 0xFF;

    tstGsmTableResponse const gasTableGsmResponse[] =
    {
    		{	1,		cModem_OK			},
    		{	2,		cModem_ERROR		},
    		{  255,    "END OF ARRAY"		}
    };

	TaskGsm_SendAtCmd1("GSM:HTTP CID\r\n","AT+HTTPPARA=\"CID\",1\r\n");
	ucResp = TaskGsm_WaitResponse1(&gasTableGsmResponse[0],TaskGsm_ParseResp1,1,10);

    switch(ucResp)
    {
        case 0:
        case 2:
        	if(++ucGsmTry <= 3)
        	{
				stGsmMsg.ucSrc = SRC_GSM;
				stGsmMsg.ucDest = SRC_GSM;
				stGsmMsg.ucEvent = EVENT_GSM_SEND_HTTPCID;
				xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
        	}
        	else
        	{
				stGsmMsg.ucSrc = SRC_GSM;
				stGsmMsg.ucDest = SRC_GSM;
				stGsmMsg.ucEvent = EVENT_GSM_ERROR;
				xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
        	}
            boError = false;
        break;

        case 1:
			stGsmMsg.ucSrc = SRC_GSM;
			stGsmMsg.ucDest = SRC_GSM;
			stGsmMsg.ucEvent = /*EVENT_GSM_SEND_HTTPURL*/EVENT_GSM_WEB_CONNECT;
			xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
			ucGsmTry  = 0;
            boError = true;
        break;

        default:
        break;
    }
    return(boError);
}

//////////////////////////////////////////////
//
//
//              TaskGsm_SendHttpUrl
//
//
//////////////////////////////////////////////
unsigned char TaskGsm_SendHttpUrl(sMessageType *psMessage)
{
    unsigned char boError = true;
    static unsigned char ucResp = 0xFF;
	char* cLocalBuffer = (char*) malloc(RX_BUF_SIZE+128);

    memset(cLocalBuffer,0,RX_BUF_SIZE+1);
    /*sprintf(cLocalBuffer,"AT+HTTPPARA=\"URL\",\"%s\"\r\n",cWebPage);*/
    sprintf(cLocalBuffer,"AT+HTTPPARA=\"URL\",\"http://gpslogger.esy.es/pages/upload/index.php?%s\"\r\n",cConfigAndData);

    tstGsmTableResponse const gasTableGsmResponse[] =
    {
    		{	1,		cModem_OK			},
    		{	2,		cModem_ERROR		},
    		{  255,    "END OF ARRAY"		}
    };

	TaskGsm_SendAtCmd1("GSM:HTTP URL\r\n",cLocalBuffer);
	ucResp = TaskGsm_WaitResponse1(&gasTableGsmResponse[0],TaskGsm_ParseResp1,1,10);

    switch(ucResp)
    {
        case 0:
        	if(++ucGsmTry <= 3)
        	{
				stGsmMsg.ucSrc = SRC_GSM;
				stGsmMsg.ucDest = SRC_GSM;
				stGsmMsg.ucEvent = EVENT_GSM_SEND_HTTPURL;
				xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
        	}
        	else
        	{
				stGsmMsg.ucSrc = SRC_GSM;
				stGsmMsg.ucDest = SRC_GSM;
				stGsmMsg.ucEvent = EVENT_GSM_ERROR;
				xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
        	}
            boError = false;
        break;

        case 2:
			stGsmMsg.ucSrc = SRC_GSM;
			stGsmMsg.ucDest = SRC_GSM;
			stGsmMsg.ucEvent = EVENT_GSM_ERROR;
			xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);

            boError = false;
        break;

        case 1:
			stGsmMsg.ucSrc = SRC_GSM;
			stGsmMsg.ucDest = SRC_GSM;
			stGsmMsg.ucEvent = EVENT_GSM_SEND_HTTPACTION;
			xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
			ucGsmTry = 0;
            boError = true;
        break;

        default:
        break;
    }
    free(cLocalBuffer);
    return(boError);
}

//////////////////////////////////////////////
//
//
//              TaskGsm_SendHttpContent
//
//
//////////////////////////////////////////////
unsigned char TaskGsm_SendHttpContent(sMessageType *psMessage)
{
    unsigned char boError = true;
    static unsigned char ucResp =0xFF;
    char* cLocalBuffer = (char*) malloc(RX_BUF_SIZE+1);

    memset(cLocalBuffer,0,RX_BUF_SIZE+1);
    sprintf(cLocalBuffer,"AT+HTTPPARA=\"CONTENT\",\"%s\"\r\n",cHttpContent);

    tstGsmTableResponse const gasTableGsmResponse[] =
    {
    		{	1,		cModem_OK			},
    		{	2,		cModem_ERROR		},
    		{  255,    "END OF ARRAY"		}
    };

	TaskGsm_SendAtCmd1("GSM:HTTP URL\r\n",cLocalBuffer);
	ucResp = TaskGsm_WaitResponse1(&gasTableGsmResponse[0],TaskGsm_ParseResp1,1,10);

    switch(ucResp)
    {
        case 0:
        case 2:
        	if(++ucGsmTry <= 3)
        	{
				stGsmMsg.ucSrc = SRC_GSM;
				stGsmMsg.ucDest = SRC_GSM;
				stGsmMsg.ucEvent = EVENT_GSM_SEND_HTTPCONTENT;
				xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
        	}
			else
			{
				stGsmMsg.ucSrc = SRC_GSM;
				stGsmMsg.ucDest = SRC_GSM;
				stGsmMsg.ucEvent = EVENT_GSM_ERROR;
				xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
			}
            boError = false;
        break;

        case 1:
			/*stGsmMsg.ucSrc = SRC_GSM;
			stGsmMsg.ucDest = SRC_GSM;
			stGsmMsg.ucEvent = EVENT_GSM_SEND_HTTPPREPAREDATA;
			xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
            boError = true;*/

			stGsmMsg.ucSrc = SRC_GSM;
			stGsmMsg.ucDest = SRC_GSM;
			stGsmMsg.ucEvent = EVENT_GSM_WEB_CONNECT;
			xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
			ucGsmTry = 0;
            boError = true;

        break;

        default:
        break;
    }
    free(cLocalBuffer);
    return(boError);
}

//////////////////////////////////////////////
//
//
//              TaskGsm_SendHttpPrepareData
//
//
//////////////////////////////////////////////
unsigned char TaskGsm_SendHttpPrepareData(sMessageType *psMessage)
{
    unsigned char boError = true;
    static unsigned char ucResp = 0xFF;
	unsigned int u16PayloadLen;
    char* cLocalBuffer = (char*) malloc(RX_BUF_SIZE+1);

    memset(cLocalBuffer,0,RX_BUF_SIZE+1);
	u16PayloadLen = strlen(cConfigAndData);

    sprintf(cLocalBuffer,"AT+HTTPDATA=%d,10000\r\n",u16PayloadLen);

    tstGsmTableResponse const gasTableGsmResponse[] =
    {
    		{	1,		"DOWNLOAD"			},
    		{	2,		cModem_ERROR		},
    		{  255,    "END OF ARRAY"		}
    };

	TaskGsm_SendAtCmd1("GSM:HTTP URL\r\n",cLocalBuffer);
	ucResp = TaskGsm_WaitResponse1(&gasTableGsmResponse[0],TaskGsm_ParseResp1,1,20);

    switch(ucResp)
    {
        case 0:
        case 2:
        {
        	if(++ucGsmTry <=3)
        	{
				stGsmMsg.ucSrc = SRC_GSM;
				stGsmMsg.ucDest = SRC_GSM;
				stGsmMsg.ucEvent = EVENT_GSM_SEND_HTTPPREPAREDATA;
				xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
        	}
        	else
        	{
				stGsmMsg.ucSrc = SRC_GSM;
				stGsmMsg.ucDest = SRC_GSM;
				stGsmMsg.ucEvent = /*EVENT_GSM_SEND_HTTPTERM*/EVENT_GSM_ERROR;
				xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
        	}
            boError = false;
        }
        break;

        case 1:
			stGsmMsg.ucSrc = SRC_GSM;
			stGsmMsg.ucDest = SRC_GSM;
			stGsmMsg.ucEvent = EVENT_GSM_SEND_HTTPDATA;
			xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
			ucGsmTry = 0;
            boError = true;
        break;

        default:
        break;
    }

    free(cLocalBuffer);
    return(boError);
}

//////////////////////////////////////////////
//
//
//              TaskGsm_SendHttpData
//
//
//////////////////////////////////////////////
unsigned char TaskGsm_SendHttpData(sMessageType *psMessage)
{
    unsigned char boError = true;
    static unsigned char ucResp = 0xFF;
    char* cLocalBuffer = (char*) malloc(RX_BUF_SIZE+1);


	stGsmMsg.ucSrc = SRC_GSM;
	stGsmMsg.ucDest = SRC_DEBUG;
	stGsmMsg.ucEvent = EVENT_IO_GSM_COMMUNICATING;
	xQueueSend( xQueueDebug, ( void * )&stGsmMsg, 0);

    memset(cLocalBuffer,0,RX_BUF_SIZE+1);
    strcpy(cLocalBuffer,cConfigAndData);

    tstGsmTableResponse const gasTableGsmResponse[] =
    {
    		{	1,		cModem_OK			},
    		{	2,		cModem_ERROR		},
    		{  255,    "END OF ARRAY"		}
    };

	TaskGsm_SendAtCmd1("GSM:DATA\r\n",cLocalBuffer);
	ucResp = TaskGsm_WaitResponse1(&gasTableGsmResponse[0],TaskGsm_ParseResp1,1,20);

    switch(ucResp)
    {
        case 0:
        case 2:
        	if(++ucGsmTry <= 3)
        	{
				stGsmMsg.ucSrc = SRC_GSM;
				stGsmMsg.ucDest = SRC_GSM;
				stGsmMsg.ucEvent = EVENT_GSM_SEND_HTTPPREPAREDATA;
				xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
        	}
        	else
        	{
    			stGsmMsg.ucSrc = SRC_GSM;
    			stGsmMsg.ucDest = SRC_GSM;
    			stGsmMsg.ucEvent = EVENT_GSM_ERROR;
    			xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
        	}
            boError = false;
        break;

        case 1:
        	stGsmMsg.ucSrc = SRC_GSM;
        	stGsmMsg.ucDest = SRC_GSM;
        	stGsmMsg.ucEvent = EVENT_GSM_SEND_HTTPACTION/*EVENT_GSM_SEND_HTTPSSL*/;
        	xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);

        	boError = true;
        break;

        default:
        break;
    }

    free(cLocalBuffer);
	return(boError);
}

//////////////////////////////////////////////
//
//
//              TaskGsm_SendHttpSsl
//
//
//////////////////////////////////////////////
unsigned char TaskGsm_SendHttpSsl(sMessageType *psMessage)
{
    unsigned char boError = true;
#if 0
    TaskGsm_SendAtCmd("GSM:HTTP SSL\r\n","AT+HTTPSSL=0\r\n",cModem_OK,cModem_ERROR);

    unsigned char ucResp;
    ucResp = TaskGsm_WaitResponse(TaskGsm_ParseResp,1, 10);

    switch(ucResp)
    {
        case 0:
        case 2:
			stGsmMsg.ucSrc = SRC_GSM;
			stGsmMsg.ucDest = SRC_GSM;
			stGsmMsg.ucEvent = EVENT_GSM_SEND_HTTPSSL;
			xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
            boError = false;
        break;

        case 1:
			stGsmMsg.ucSrc = SRC_GSM;
			stGsmMsg.ucDest = SRC_GSM;
			stGsmMsg.ucEvent = EVENT_GSM_SEND_HTTPACTION;
			xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
			ucGsmTry = 0;
            boError = true;
        break;

        default:
        break;
    }
#endif
    return(boError);
}

//////////////////////////////////////////////
//
//
//              TaskGsm_SendHttpAction
//
//
//////////////////////////////////////////////
unsigned char TaskGsm_SendHttpAction(sMessageType *psMessage)
{
    unsigned char boError = true;
    static unsigned char ucResp = 0xFF;
    static unsigned char ucGsmTryAction = 0;

    /*gucQtyItemsTx = 6*gucQtyItemsTx;
    memset(cGsmWorkBuffer,0,sizeof(cGsmWorkBuffer));
    sprintf(cGsmWorkBuffer,"1,200,%d",gucQtyItemsTx);*/

    tstGsmTableResponse const gasTableGsmResponse[] =
    {
    		{	1,		"1,200"				},
			{	2,		"1,603"				},
    		{	3,		cModem_ERROR		},
    		{  255,    "END OF ARRAY"		}
    };

	TaskGsm_SendAtCmd1("GSM:HTTP ACTION\r\n","AT+HTTPACTION=1\r\n");
	ucResp = TaskGsm_WaitResponse1(&gasTableGsmResponse[0],TaskGsm_ParseResp1,1,20);

    switch(ucResp)
    {
        case 0:
			stGsmMsg.ucSrc = SRC_GSM;
			stGsmMsg.ucDest = SRC_GSM;
			stGsmMsg.ucEvent = EVENT_GSM_ERROR;
			xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
			boError = false;
			break;

        case 2:
        case 3:
        	if(++ucGsmTryAction<2)
        	{
				stGsmMsg.ucSrc = SRC_GSM;
				stGsmMsg.ucDest = SRC_GSM;
				stGsmMsg.ucEvent = EVENT_GSM_SEND_HTTPURL;
				xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
        	}
        	else
        	{
				stGsmMsg.ucSrc = SRC_GSM;
				stGsmMsg.ucDest = SRC_GSM;
				stGsmMsg.ucEvent = EVENT_GSM_ERROR;
				xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
				ucGsmTryAction = 0;
        	}
			boError = false;
        break;

        case 1:
        	/*stGsmMsg.ucSrc = SRC_GSM;
        	stGsmMsg.ucDest = SRC_SD;
        	stGsmMsg.ucEvent = EVENT_SD_MARKING;
        	xQueueSend( xQueueSd, ( void * )&stGsmMsg, 0);

        	stGsmMsg.ucSrc = SRC_GSM;
        	stGsmMsg.ucDest = SRC_DEBUG;
        	stGsmMsg.ucEvent = EVENT_IO_GSM_UPLOAD_DONE;
        	xQueueSend( xQueueDebug, ( void * )&stGsmMsg, 0);*/

			stGsmMsg.ucSrc = SRC_GSM;
			stGsmMsg.ucDest = SRC_GSM;
			stGsmMsg.ucEvent = EVENT_GSM_SEND_HTTPREAD;
			xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);

        	ucGsmTryAction = 0;
        	boError = true;
        break;

    }

    return(boError);
}

//////////////////////////////////////////////
//
//
//              TaskGsm_SendHttpRead
//
//
//////////////////////////////////////////////
unsigned char TaskGsm_SendHttpRead(sMessageType *psMessage)
{
    unsigned char boError = true;
    static unsigned char ucResp = 0xFF;
#if 1

    tstGsmTableResponse const gasTableGsmResponse[] =
    {
    		{	1,		cModem_ACK			},
    		{	2,		cModem_ERROR		},
    		{  255,    "END OF ARRAY"		}
    };

	TaskGsm_SendAtCmd1("GSM:HTTP READ\r\n","AT+HTTPREAD\r\n");
	ucResp = TaskGsm_WaitResponse1(&gasTableGsmResponse[0],TaskGsm_ParseResp1,1,10);

    switch(ucResp)
    {
        case 0:
        case 2:
        	if(++ucGsmTry <=3)
        	{
				stGsmMsg.ucSrc = SRC_GSM;
				stGsmMsg.ucDest = SRC_GSM;
				stGsmMsg.ucEvent = EVENT_GSM_SEND_HTTPREAD;
				xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
        	}
        	else
        	{
				stGsmMsg.ucSrc = SRC_GSM;
				stGsmMsg.ucDest = SRC_GSM;
				stGsmMsg.ucEvent = EVENT_GSM_ERROR;
				xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
        	}
        	boError = false;
        break;

        case 1:
        	stGsmMsg.ucSrc = SRC_GSM;
        	stGsmMsg.ucDest = SRC_SD;
        	stGsmMsg.ucEvent = EVENT_SD_MARKING;
        	xQueueSend( xQueueSd, ( void * )&stGsmMsg, 0);

        	stGsmMsg.ucSrc = SRC_GSM;
        	stGsmMsg.ucDest = SRC_DEBUG;
        	stGsmMsg.ucEvent = EVENT_IO_GSM_UPLOAD_DONE;
        	xQueueSend( xQueueDebug, ( void * )&stGsmMsg, 0);
        	boError = true;
        break;

        default:
        break;
    }
#endif
    return(boError);
}

//////////////////////////////////////////////
//
//
//              TaskGsm_SendHttpTerm
//
//
//////////////////////////////////////////////
unsigned char TaskGsm_SendHttpTerm(sMessageType *psMessage)
{
    unsigned char boError = true;
#if 0
    TaskGsm_SendAtCmd("GSM:HTTP TERM\r\n","AT+HTTPTERM\r\n",cModem_OK,cModem_ERROR);

    unsigned char ucResp;
    ucResp = TaskGsm_WaitResponse(TaskGsm_ParseResp,1, 10);

    switch(ucResp)
    {
        case 0:
        case 2:
        	if(++ucGsmTry <=2)
        	{
				stGsmMsg.ucSrc = SRC_GSM;
				stGsmMsg.ucDest = SRC_GSM;
				stGsmMsg.ucEvent = EVENT_GSM_SEND_HTTPTERM;
				xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
				boError = false;
        	}
        	else
        	{
				stGsmMsg.ucSrc = SRC_GSM;
				stGsmMsg.ucDest = SRC_GSM;
				stGsmMsg.ucEvent = EVENT_GSM_ERROR;
				xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
        	}
        break;

        case 1:
        	stGsmMsg.ucSrc = SRC_GSM;
        	stGsmMsg.ucDest = SRC_GSM;
        	stGsmMsg.ucEvent = EVENT_GSM_SEND_BEARERSET4;
        	xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
        break;

        default:
        break;
    }
#endif
    return(boError);
}

//////////////////////////////////////////////
//
//
//              TaskGsm_SendBearerSet4
//
//
//////////////////////////////////////////////
unsigned char TaskGsm_SendBearerSet4(sMessageType *psMessage)
{
    unsigned char boError = true;
#if 0
    TaskGsm_SendAtCmd("GSM:BEARER SET4\r\n","AT+SAPBR=0,1\r\n",cModem_OK,cModem_ERROR);

    unsigned char ucResp;
    ucResp = TaskGsm_WaitResponse(TaskGsm_ParseResp,1, 10);

    switch(ucResp)
    {
        case 0:
        case 2:
        	if(++ucGsmTry <=2)
        	{
				stGsmMsg.ucSrc = SRC_GSM;
				stGsmMsg.ucDest = SRC_GSM;
				stGsmMsg.ucEvent = EVENT_GSM_SEND_BEARERSET4;
				xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
				boError = false;
        	}
        	else
        	{
				stGsmMsg.ucSrc = SRC_GSM;
				stGsmMsg.ucDest = SRC_GSM;
				stGsmMsg.ucEvent = EVENT_GSM_ERROR;
				xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
        	}
        break;

        case 1:
        	stGsmMsg.ucSrc = SRC_GSM;
        	stGsmMsg.ucDest = SRC_GSM;
        	stGsmMsg.ucEvent = EVENT_GSM_SEND_BEARERSET1;
        	xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
        break;

        default:
        break;
    }
#endif
    return(boError);
}

//////////////////////////////////////////////
//
//
//              TaskGsm_SendStart
//
//
//////////////////////////////////////////////
unsigned char TaskGsm_SendStart(sMessageType *psMessage)
{
    unsigned char boError = false;

	char* cLocalBuffer = (char*) malloc(RX_BUF_SIZE+1);

    memset(cLocalBuffer,0,RX_BUF_SIZE+1);
    sprintf(cLocalBuffer,"%s%s%s\r\n",cHttpAtStartCIP,stConfigData.cHttpDomain,cHttpAtEndCIP);
    TaskGsm_SendAtCmd("GSM:>START\r\n",cLocalBuffer,"CONNECT",cModem_ERROR);

    unsigned char ucResp;
    ucResp = TaskGsm_WaitResponse(TaskGsm_ParseResp,1,10);

    switch(ucResp)
    {
        case 0:
        case 2:
        	if(++ucGsmTry<=3)
			{
        		stGsmMsg.ucSrc = SRC_GSM;
        		stGsmMsg.ucDest = SRC_GSM;
        		stGsmMsg.ucEvent = EVENT_GSM_SEND_START;
    			xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
			}
        	else
        	{
        		stGsmMsg.ucSrc = SRC_GSM;
        		stGsmMsg.ucDest = SRC_GSM;
        		stGsmMsg.ucEvent = EVENT_GSM_ERROR;
    			xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
        	}
        break;

        case 1:
    		stGsmMsg.ucSrc = SRC_GSM;
    		stGsmMsg.ucDest = SRC_GSM;
    		stGsmMsg.ucEvent = /*EVENT_GSM_SEND_BEARERSET1*/EVENT_GSM_WEB_CONNECT/*EVENT_GSM_SEND_CIPTKA*/;
			xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
			ucGsmTry = 0;
            boError = true;
        break;
    }
    free(cLocalBuffer);
    return(boError);
}
//////////////////////////////////////////////
//
//
//       TaskGsm_SendCiptka
//
//
//////////////////////////////////////////////
unsigned char TaskGsm_SendCiptka(sMessageType *psMessage)
{

	unsigned char boError = true;
    unsigned char ucResp = 0;

	char* cLocalBuffer = (char*) malloc(RX_BUF_SIZE+1);
    memset(cLocalBuffer,0,RX_BUF_SIZE+1);

    sprintf(cLocalBuffer,"AT+CIPTKA=?\r\n");

    TaskGsm_SendAtCmd("GSM:>CIPTKA\r\n",cLocalBuffer,cModem_OK,cModem_ERROR);

    ucResp = TaskGsm_WaitResponse(TaskGsm_ParseResp,1,20);


    switch(ucResp)
    {
        case 0:
        case 2:
        	if(++ucGsmTry<=3)
			{
        		stGsmMsg.ucSrc = SRC_GSM;
        		stGsmMsg.ucDest = SRC_GSM;
        		stGsmMsg.ucEvent = EVENT_GSM_SEND_CIPTKA;
			}
        	else
        	{
        		stGsmMsg.ucSrc = SRC_GSM;
        		stGsmMsg.ucDest = SRC_GSM;
        		stGsmMsg.ucEvent = EVENT_GSM_ERROR;
        	}
			xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
            boError = false;
        break;

        case 1:
    		stGsmMsg.ucSrc = SRC_GSM;
    		stGsmMsg.ucDest = SRC_GSM;
    		stGsmMsg.ucEvent = EVENT_GSM_WEB_CONNECT;
			xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
			ucGsmTry = 0;
            boError = true;
        break;

        default:
        break;
    }
    free(cLocalBuffer);
    return(boError);
}

//////////////////////////////////////////////
//
//
//       TaskGsm_SendCipsend
//
//
//////////////////////////////////////////////
unsigned char TaskGsm_SendCipsend(sMessageType *psMessage)
{
	/*const char cLINKINVALID[]  		= {"link is not"};*/
	unsigned char boError = true;
	/*unsigned int u16PayloadLen;*/
    unsigned char ucResp = 0;

	char* cLocalBuffer = (char*) malloc(RX_BUF_SIZE+256);
    memset(cLocalBuffer,0,RX_BUF_SIZE+1);

    //sprintf(cLocalBuffer,"%s","GET http://gpslogger.esy.es/pages/upload/timestamp.php HTTP/1.1\r\nHost: gpslogger.esy.es\r\nContent-Type: application/x-www-form-urlencoded\r\n\r\n");
    //sprintf(cLocalBuffer,"POST %s %s%d%s%s\r\n",cWebPage," HTTP/1.1\r\nHost: gpslogger.esy.es\r\nAccept: */*\r\nContent-Length: ",strlen((const char*)cConfigAndData),"\r\nConnection: Keep-Alive\r\nContent-Type: application/x-www-form-urlencoded\r\n\r\n",cConfigAndData);
    //sprintf(cLocalBuffer,"POST %s%s%d%s%s\r\n",cWebPage," HTTP/1.1\r\nHost: gpslogger.esy.es\r\nContent-Length: ",strlen((const char*)cConfigAndData),"\r\nContent-Type: application/x-www-form-urlencoded\r\n\r\n",cConfigAndData);
    //sprintf(cLocalBuffer,"POST %s %s%d%s%s\r\n",cWebPage," HTTP/1.0\r\nHost: gpslogger.esy.es\r\nContent-Length: ",strlen((const char*)cConfigAndData),"\r\nConnection: Keep-Alive\r\nContent-Type: application/x-www-form-urlencoded\r\n\r\n",cConfigAndData);
    sprintf(cLocalBuffer,"POST %s%s%d%s%s\r\n",cWebPage," HTTP/1.1\r\nHost: gpslogger.esy.es\r\nContent-Length: ",strlen((const char*)cConfigAndData),"\r\nContent-Type: application/x-www-form-urlencoded\r\n\r\n",cConfigAndData);
    /*u16PayloadLen = strlen(cLocalBuffer);*/
	memset(cLocalBuffer,0x00,RX_BUF_SIZE+1);
	//sprintf(cLocalBuffer,"AT+CIPSEND=%d\r\n",u16PayloadLen);
	sprintf(cLocalBuffer,"AT+CIPSEND\r\n");

    TaskGsm_SendAtCmd("GSM:>CIPSEND\r\n",cLocalBuffer,">",cModem_ERROR);

    ucResp = TaskGsm_WaitResponse(TaskGsm_ParseResp,1,20);

    switch(ucResp)
    {
        case 0:
        case 2:
        	if(++ucGsmTry<=3)
			{
        		stGsmMsg.ucSrc = SRC_GSM;
        		stGsmMsg.ucDest = SRC_GSM;
        		stGsmMsg.ucEvent = EVENT_GSM_SEND_CIPSEND;
			}
        	else
        	{
        		stGsmMsg.ucSrc = SRC_GSM;
        		stGsmMsg.ucDest = SRC_GSM;
        		stGsmMsg.ucEvent = EVENT_GSM_ERROR;
        	}
			xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
            boError = false;
        break;

        case 1:
    		stGsmMsg.ucSrc = SRC_GSM;
    		stGsmMsg.ucDest = SRC_GSM;
    		stGsmMsg.ucEvent = EVENT_GSM_SEND_DATA;
			xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
			ucGsmTry = 0;
            boError = true;
        break;

        default:
        break;
    }
    free(cLocalBuffer);
    return(boError);
}

//////////////////////////////////////////////
//
//
//              TaskGsm_SendData
//
//
//////////////////////////////////////////////
unsigned char TaskGsm_SendData(sMessageType *psMessage)
{
	unsigned char boError = true;

	unsigned int u16PayloadLen;
	unsigned char ucResp = 0;

	char* cLocalBuffer = (char*) malloc(2*RX_BUF_SIZE+1);
    memset(cLocalBuffer,0,RX_BUF_SIZE+1);

    //sprintf(cLocalBuffer,"POST %s %s%d%s%s\r\n",cWebPage," HTTP/1.1\r\nHost: gpslogger.esy.es\r\nAccept: */*\r\nContent-Length: ",strlen((const char*)cConfigAndData),"\r\nConnection: Keep-Alive\r\nContent-Type: application/x-www-form-urlencoded\r\n\r\n",cConfigAndData);
    sprintf(cLocalBuffer,"POST %s%s%d%s%s\r\n",cWebPage," HTTP/1.1\r\nHost: gpslogger.esy.es\r\nContent-Length: ",strlen((const char*)cConfigAndData),"\r\nContent-Type: application/x-www-form-urlencoded\r\n\r\n",cConfigAndData);
    //sprintf(cLocalBuffer,"POST %s %s%d%s%s\r\n",cWebPage," HTTP/1.0\r\nHost: gpslogger.esy.es\r\nContent-Length: ",strlen((const char*)cConfigAndData),"\r\nConnection: Keep-Alive\r\nContent-Type: application/x-www-form-urlencoded\r\n\r\n",cConfigAndData);
	u16PayloadLen = strlen(cLocalBuffer);

	cLocalBuffer[u16PayloadLen]=0x1A;
	TaskGsm_SendAtCmd("GSM:>DATA\r\n",cLocalBuffer,"ReGcOr","CLOSED");

    ucResp = TaskGsm_WaitResponse(TaskGsm_ParseResp,5,20);

    switch(ucResp)
    {
        case 0:
        case 2:
        	if(++ucGsmTry<=3)
			{
        		stGsmMsg.ucSrc = SRC_GSM;
        		stGsmMsg.ucDest = SRC_GSM;
        		stGsmMsg.ucEvent = EVENT_GSM_SEND_DATA;
			}
        	else
        	{
        		stGsmMsg.ucSrc = SRC_GSM;
        		stGsmMsg.ucDest = SRC_GSM;
        		stGsmMsg.ucEvent = EVENT_GSM_ERROR;
        	}
			xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);
            boError = false;
        break;

        case 1:
        	stGsmMsg.ucSrc = SRC_GSM;
        	stGsmMsg.ucDest = SRC_SD;
        	stGsmMsg.ucEvent = EVENT_SD_MARKING;
        	xQueueSend( xQueueSd, ( void * )&stGsmMsg, 0);

        	stGsmMsg.ucSrc = SRC_GSM;
        	stGsmMsg.ucDest = SRC_DEBUG;
        	stGsmMsg.ucEvent = EVENT_IO_GSM_UPLOAD_DONE;
        	xQueueSend( xQueueDebug, ( void * )&stGsmMsg, 0);

			ucGsmTry = 0;
            boError = true;
        break;

        default:
        break;
    }
    free(cLocalBuffer);
    return(boError);
}

//////////////////////////////////////////////
//
//
//              TaskGsm_DataOk
//
//
//////////////////////////////////////////////
unsigned char TaskGsm_DataOk(sMessageType *psMessage)
{
	unsigned char boError = true;

    ESP_LOGI(GSM_TASK_TAG, "GSM:DATA OK\r\n");

	return(boError);
}

//////////////////////////////////////////////
//
//
//              TaskGsm_Ending
//
//
//////////////////////////////////////////////
unsigned char TaskGsm_Ending(sMessageType *psMessage)
{
	unsigned char boError = true;

    ESP_LOGI(GSM_TASK_TAG, "GSM:ENDING\r\n");

	vTaskDelay(2000/portTICK_PERIOD_MS);

	stGsmMsg.ucSrc = SRC_GSM;
	stGsmMsg.ucDest = SRC_SD;
	stGsmMsg.ucEvent = EVENT_SD_OPENING;
	xQueueSend( xQueueSd, ( void * )&stGsmMsg, 0);

	return(boError);
}


//////////////////////////////////////////////
//
//
//              TaskGsm_Error
//
//
//////////////////////////////////////////////
unsigned char TaskGsm_Error(sMessageType *psMessage)
{
	unsigned char boError = true;

    ESP_LOGI(GSM_TASK_TAG, "GSM:ERROR\r\n");

	stGsmMsg.ucSrc = SRC_GSM;
	stGsmMsg.ucDest = SRC_GSM;
	stGsmMsg.ucEvent = EVENT_GSM_INIT;
	xQueueSend( xQueueGsm, ( void * )&stGsmMsg, 0);


	return(boError);
}
//////////////////////////////////////////////
//
//
//              TaskGsm_IgnoreEvent
//
//
//////////////////////////////////////////////
unsigned char TaskGsm_IgnoreEvent(sMessageType *psMessage)
{
    unsigned char boError = false;
    return(boError);
}

//////////////////////////////////////////////
//
//
//             Modem State Machine
//
//
//////////////////////////////////////////////
static sStateMachineType const gasTaskGsm_Idling[] =
{
    /* Event        Action routine      Next state */
    //  State specific transitions
	{EVENT_GSM_INIT,          TaskGsm_Init,                 TASKGSM_IDLING,           		TASKGSM_IDLING        				},
	{EVENT_GSM_SETBAUD,       TaskGsm_SetBaudRate,      	TASKGSM_IDLING,					TASKGSM_IDLING  					},
    {EVENT_GSM_AT,            TaskGsm_SendAt,           	TASKGSM_INITIALIZING,   		TASKGSM_IDLING  					},
	{EVENT_GSM_ERROR,	      TaskGsm_Error,				TASKGSM_IDLING,  			  	TASKGSM_IDLING						},
    {EVENT_GSM_NULL,          TaskGsm_IgnoreEvent,          TASKGSM_IDLING,					TASKGSM_IDLING						}
};


static sStateMachineType const gasTaskGsm_Initializing[] =
{
    /* Event        Action routine      Next state */
    //  State specific transitions

    /*{EVENT_GSM_SEND_CPIN,     TaskGsm_SendCpin,         TASKGSM_INITIALIZING,         TASKGSM_INITIALIZING  },*/
    /*{EVENT_GSM_SEND_CSQ,      TaskGsm_SendCsq,          TASKGSM_INITIALIZING,         TASKGSM_INITIALIZING  },*/


    {EVENT_GSM_GET_CCID,      			TaskGsm_GetCcid,          			TASKGSM_INITIALIZING,       TASKGSM_INITIALIZING  	},
    {EVENT_GSM_SEND_CSQ,      			TaskGsm_SendCsq,          			TASKGSM_INITIALIZING,       TASKGSM_INITIALIZING  	},
    {EVENT_GSM_SELECT_SMS_FORMAT,   	TaskGsm_SelectSmsFormat,			TASKGSM_INITIALIZING,       TASKGSM_INITIALIZING  	},

    {EVENT_GSM_LIST_SMS_MSG,      		TaskGsm_ListSmsMsg,					TASKGSM_INITIALIZING,       TASKGSM_INITIALIZING  	},
    {EVENT_GSM_DECODE_SMS_MSG,      	TaskGsm_DecodeSmsMsg,				TASKGSM_INITIALIZING,       TASKGSM_INITIALIZING  	},
    {EVENT_GSM_SEND_SMS_CSQ,      		TaskGsm_SendSmsCsq,					TASKGSM_INITIALIZING,       TASKGSM_INITIALIZING  	},
    {EVENT_GSM_SEND_SMS_CCID,      		TaskGsm_SendSmsCcid,				TASKGSM_INITIALIZING,       TASKGSM_INITIALIZING  	},
    {EVENT_GSM_SEND_SMS_GPS,      		TaskGsm_SendSmsGps,					TASKGSM_INITIALIZING,       TASKGSM_INITIALIZING  	},

    {EVENT_GSM_PREPARE_SMS_RESP,      	TaskGsm_PrepareSmsResponse,			TASKGSM_INITIALIZING,       TASKGSM_INITIALIZING  	},
    {EVENT_GSM_SEND_SMS_RESP,      		TaskGsm_SendSmsResponse,			TASKGSM_INITIALIZING,       TASKGSM_INITIALIZING  	},
    {EVENT_GSM_DELETE_SMS_MSG,      	TaskGsm_DeleteSmsMsg,				TASKGSM_INITIALIZING,       TASKGSM_INITIALIZING  	},

    {EVENT_GSM_SEND_CGATT,    			TaskGsm_SendCgatt,      			TASKGSM_INITIALIZING,       TASKGSM_INITIALIZING  	},
    {EVENT_GSM_SEND_CGDCONT,  			TaskGsm_SendCgdcont,      			TASKGSM_INITIALIZING,       TASKGSM_INITIALIZING  	},
    {EVENT_GSM_SEND_CIPSHUT,  			TaskGsm_SendCipshut,     			TASKGSM_INITIALIZING,       TASKGSM_INITIALIZING  	},

    {EVENT_GSM_SEND_CIPSTATUS,			TaskGsm_SendCipStatus,      		TASKGSM_INITIALIZING,       TASKGSM_INITIALIZING 	},
    {EVENT_GSM_SEND_CIPMUX,   			TaskGsm_SendCipMux,       			TASKGSM_INITIALIZING,       TASKGSM_INITIALIZING  	},
    {EVENT_GSM_SEND_CIPMODE,   			TaskGsm_SendCipMode,       			TASKGSM_INITIALIZING,       TASKGSM_INITIALIZING  	},
    {EVENT_GSM_SEND_CSTT,     			TaskGsm_SendCstt,					TASKGSM_INITIALIZING,    	TASKGSM_INITIALIZING  	},

    {EVENT_GSM_SEND_CIICR,    			TaskGsm_SendCiicr,        			TASKGSM_INITIALIZING,      	TASKGSM_INITIALIZING  },
	{EVENT_GSM_SEND_CIFSR,    			TaskGsm_SendCifsr,        			TASKGSM_INITIALIZING,      	TASKGSM_INITIALIZING  },

	{EVENT_GSM_SEND_START,	   			TaskGsm_SendStart,					TASKGSM_INITIALIZING,      	TASKGSM_INITIALIZING 	},
	{EVENT_GSM_SEND_CIPTKA,	   			TaskGsm_SendCiptka,					TASKGSM_INITIALIZING,    	TASKGSM_INITIALIZING 	},

	{EVENT_GSM_SEND_BEARERSET1,	  		TaskGsm_SendBearerSet1,				TASKGSM_INITIALIZING,    	TASKGSM_COMMUNICATING },
	{EVENT_GSM_SEND_BEARERSET2,	  		TaskGsm_SendBearerSet2,				TASKGSM_INITIALIZING,    	TASKGSM_COMMUNICATING },
	{EVENT_GSM_SEND_BEARERSET21,  		TaskGsm_SendBearerSet21,			TASKGSM_INITIALIZING,    	TASKGSM_COMMUNICATING },
	{EVENT_GSM_SEND_BEARERSET22,  		TaskGsm_SendBearerSet22,			TASKGSM_INITIALIZING,    	TASKGSM_COMMUNICATING },


	{EVENT_GSM_SEND_BEARERSET3,	  		TaskGsm_SendBearerSet3,				TASKGSM_INITIALIZING,    	TASKGSM_INITIALIZING },
	/*{EVENT_GSM_GET_BEARER,	  	  	TaskGsm_GetBearer,					TASKGSM_INITIALIZING,    	TASKGSM_COMMUNICATING },*/
	/*{EVENT_GSM_SEND_BEARERSET33,  	TaskGsm_SendBearerSet33,			TASKGSM_INITIALIZING,    	TASKGSM_COMMUNICATING },*/

	{EVENT_GSM_SEND_HTTPINIT,	  		TaskGsm_SendHttpInit,				TASKGSM_INITIALIZING,    	TASKGSM_INITIALIZING },
	{EVENT_GSM_SEND_HTTPCID,	  		TaskGsm_SendHttpCid,				TASKGSM_INITIALIZING, 		TASKGSM_INITIALIZING },

	/*{EVENT_GSM_SEND_HTTPCONTENT,  	TaskGsm_SendHttpContent,			TASKGSM_INITIALIZING,    	TASKGSM_INITIALIZING },*/

	{EVENT_GSM_WEB_CONNECT,   			TaskGsm_WebConnect,       			TASKGSM_COMMUNICATING,        TASKGSM_INITIALIZING  },

	{EVENT_GSM_ERROR,	      			TaskGsm_Error,						TASKGSM_IDLING,  			  TASKGSM_IDLING		},
	{EVENT_GSM_NULL,          			TaskGsm_IgnoreEvent,      			TASKGSM_INITIALIZING,		  TASKGSM_INITIALIZING	}
};

static sStateMachineType const gasTaskGsm_Communicating[] =
{

    /* Event        Action routine      Next state */
    //  State specific transitions
	/*{EVENT_GSM_SEND_CIICR,    		TaskGsm_SendCiicr,        	TASKGSM_COMMUNICATING,    	TASKGSM_COMMUNICATING },
	{EVENT_GSM_SEND_CIFSR,    		TaskGsm_SendCifsr,        	TASKGSM_COMMUNICATING,    	TASKGSM_COMMUNICATING },*/

	/*{EVENT_GSM_WEB_CONNECT,   		TaskGsm_WebConnect,       	TASKGSM_COMMUNICATING,      TASKGSM_COMMUNICATING },*/
    /*{EVENT_GSM_OPENING_FILE_OK,   	TaskGsm_OpeningFileOk,    	TASKGSM_COMMUNICATING,    	TASKGSM_COMMUNICATING },*/

	{EVENT_GSM_SEND_HTTPURL,	  	TaskGsm_SendHttpUrl,			TASKGSM_COMMUNICATING,    	TASKGSM_COMMUNICATING },
	{EVENT_GSM_WEB_CONNECT,   		TaskGsm_WebConnect,       		TASKGSM_COMMUNICATING,      TASKGSM_COMMUNICATING  },


	/*{EVENT_GSM_SEND_START,	   		TaskGsm_SendStart,				TASKGSM_COMMUNICATING,    	TASKGSM_COMMUNICATING },
    {EVENT_GSM_SEND_CIPSEND,		TaskGsm_SendCipsend,			TASKGSM_COMMUNICATING,    	TASKGSM_COMMUNICATING },
    {EVENT_GSM_SEND_DATA,			TaskGsm_SendData,				TASKGSM_COMMUNICATING,    	TASKGSM_COMMUNICATING },*/

	/*{EVENT_GSM_SEND_BEARERSET1,	  	TaskGsm_SendBearerSet1,			TASKGSM_COMMUNICATING,    	TASKGSM_COMMUNICATING },
	{EVENT_GSM_SEND_BEARERSET2,	  	TaskGsm_SendBearerSet2,			TASKGSM_COMMUNICATING,    	TASKGSM_COMMUNICATING },
	{EVENT_GSM_SEND_BEARERSET21,  	TaskGsm_SendBearerSet21,		TASKGSM_COMMUNICATING,    	TASKGSM_COMMUNICATING },
	{EVENT_GSM_SEND_BEARERSET22,  	TaskGsm_SendBearerSet22,		TASKGSM_COMMUNICATING,    	TASKGSM_COMMUNICATING },*/

	/*{EVENT_GSM_SEND_BEARERSET3,	  	TaskGsm_SendBearerSet3,			TASKGSM_COMMUNICATING,    	TASKGSM_COMMUNICATING },*/
	/*{EVENT_GSM_GET_BEARER,	  	  TaskGsm_GetBearer,			TASKGSM_COMMUNICATING,    	TASKGSM_COMMUNICATING },*/
	/*{EVENT_GSM_SEND_BEARERSET33,  TaskGsm_SendBearerSet33,		TASKGSM_COMMUNICATING,    	TASKGSM_COMMUNICATING },*/

	/*{EVENT_GSM_SEND_HTTPINIT,	  	TaskGsm_SendHttpInit,			TASKGSM_COMMUNICATING,    	TASKGSM_COMMUNICATING },
	{EVENT_GSM_SEND_HTTPCID,	  	TaskGsm_SendHttpCid,			TASKGSM_COMMUNICATING, 		TASKGSM_COMMUNICATING },
	{EVENT_GSM_SEND_HTTPURL,	  	TaskGsm_SendHttpUrl,			TASKGSM_COMMUNICATING, 		TASKGSM_COMMUNICATING},

	{EVENT_GSM_SEND_HTTPCONTENT,  	TaskGsm_SendHttpContent,		TASKGSM_COMMUNICATING,    	TASKGSM_COMMUNICATING },*/
	/*{EVENT_GSM_SEND_HTTPPREPAREDATA,TaskGsm_SendHttpPrepareData,	TASKGSM_COMMUNICATING, 		TASKGSM_COMMUNICATING },
	{EVENT_GSM_SEND_HTTPDATA,		TaskGsm_SendHttpData,			TASKGSM_COMMUNICATING, 		TASKGSM_COMMUNICATING},
	{EVENT_GSM_SEND_HTTPSSL,		TaskGsm_SendHttpSsl,			TASKGSM_COMMUNICATING, 		TASKGSM_COMMUNICATING},*/
	{EVENT_GSM_SEND_HTTPACTION,		TaskGsm_SendHttpAction,			TASKGSM_COMMUNICATING, 		TASKGSM_COMMUNICATING},
	{EVENT_GSM_SEND_HTTPREAD,		TaskGsm_SendHttpRead,			TASKGSM_COMMUNICATING, 		TASKGSM_COMMUNICATING},
	{EVENT_GSM_SEND_HTTPTERM,		TaskGsm_SendHttpTerm,			TASKGSM_COMMUNICATING, 		TASKGSM_COMMUNICATING},
	{EVENT_GSM_SEND_BEARERSET4,		TaskGsm_SendBearerSet4,			TASKGSM_COMMUNICATING, 		TASKGSM_COMMUNICATING},

    /*{EVENT_GSM_LIST_SMS_MSG,        TaskGsm_ListSmsMsg,				TASKGSM_SMSREADING,       			TASKGSM_COMMUNICATING  	},*/
	{EVENT_GSM_ENDING,			  	TaskGsm_Ending,					TASKGSM_COMMUNICATING,  			TASKGSM_IDLING			},
	{EVENT_GSM_ERROR,			  	TaskGsm_Error,					TASKGSM_IDLING,  					TASKGSM_IDLING			},
    {EVENT_GSM_NULL,              	TaskGsm_IgnoreEvent,      		TASKGSM_COMMUNICATING,	  			TASKGSM_COMMUNICATING	}

};

#if 0
static sStateMachineType const gasTaskGsm_SmsReading[] =
{

    /* Event        Action routine      Next state */
    //  State specific transitions
	{EVENT_GSM_DECODE_SMS_MSG,      	TaskGsm_DecodeSmsMsg,				TASKGSM_SMSREADING,       TASKGSM_INITIALIZING  	},
	{EVENT_GSM_SEND_SMS_CSQ,      		TaskGsm_SendSmsCsq,					TASKGSM_SMSREADING,       TASKGSM_INITIALIZING  	},
    {EVENT_GSM_SEND_CSQ,      			TaskGsm_SendCsq,          			TASKGSM_SMSREADING,       TASKGSM_INITIALIZING  	},

	{EVENT_GSM_SEND_SMS_CCID,      		TaskGsm_SendSmsCcid,				TASKGSM_SMSREADING,       TASKGSM_INITIALIZING  	},
	{EVENT_GSM_SEND_SMS_GPS,      		TaskGsm_SendSmsGps,					TASKGSM_SMSREADING,       TASKGSM_INITIALIZING  	},

	{EVENT_GSM_PREPARE_SMS_RESP,      	TaskGsm_PrepareSmsResponse,			TASKGSM_SMSREADING,       TASKGSM_INITIALIZING  	},
	{EVENT_GSM_SEND_SMS_RESP,      		TaskGsm_SendSmsResponse,			TASKGSM_SMSREADING,       TASKGSM_INITIALIZING  	},
	{EVENT_GSM_DELETE_SMS_MSG,      	TaskGsm_DeleteSmsMsg,				TASKGSM_INITIALIZING,     TASKGSM_INITIALIZING  	},

	{EVENT_GSM_ERROR,			  		TaskGsm_Error,						TASKGSM_IDLING,  			TASKGSM_IDLING			},
    {EVENT_GSM_NULL,              		TaskGsm_IgnoreEvent,      			TASKGSM_SMSREADING,	  		TASKGSM_SMSREADING		}

};
#endif
static sStateMachineType const * const gpasTaskGsm_StateMachine[] =
{
    gasTaskGsm_Idling,
	gasTaskGsm_Initializing,
	gasTaskGsm_Communicating
	/*gasTaskGsm_SmsReading*/
};


void vTaskGsm( void *pvParameters )
{
	TickType_t elapsed_time;


	for( ;; )
	{
		elapsed_time = xTaskGetTickCount();
		if( xQueueReceive( xQueueGsm, &(stGsmMsg ),0 ) )
		{
            (void)eEventHandler ((unsigned char)SRC_GSM,gpasTaskGsm_StateMachine[ucCurrentStateGsm], &ucCurrentStateGsm, &stGsmMsg);
		}

		vTaskDelayUntil(&elapsed_time, 500 / portTICK_PERIOD_MS);
	}
}
