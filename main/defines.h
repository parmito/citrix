#ifndef _DEFINES_H
#define _DEFINES_H

#include <time.h>

#ifdef __cplusplus
 extern "C" {
#endif

#define SOFTWARE_VERSION 		(char*)"4.0.5"

#define MODEM_ATTEMPT			3


#define RX_BUF_SIZE				512
#define RX_BUF_SIZE_REDUCED		64

#define MAX_ROW					1

#define MAX_SMS_MESSAGES		5

#define NUM_QUEUE 7
#define NUM_TASKS 6


#define DEBUG_GPS 				1
#define DEBUG_SDCARD 			1
#define DEBUG_IO				1
#define DEBUG_WIFI				0
#define DEBUG_GSM				1

#define TASK_IO_TIMEOUT  		(unsigned long)(25)
#define TASK_GPS_TIMEOUT 		(unsigned long)(50)
#define TASK_WIFI_TIMEOUT  		(unsigned long)(75)
#define TASK_SD_TIMEOUT  		(unsigned long)(100)
#define TASK_GSM_TIMEOUT  		(unsigned long)(75)
#define TASK_USART_TIMEOUT  	(unsigned long)(1)

#define TASK_GPS_1_SECOND		(unsigned long)(1000/TASK_GPS_TIMEOUT)
#define TASK_WIFI_1_SECOND		(unsigned long)(1000/TASK_WIFI_TIMEOUT)
#define TASK_SD_1_SECOND		(unsigned long)(1000/TASK_SD_TIMEOUT)
#define TASK_IO_1_SECOND		(unsigned long)(1000/TASK_IO_TIMEOUT)
#define TASK_GSM_1_SECOND		(unsigned long)(1000/TASK_GSM_TIMEOUT)
#define TASK_GSM_2_SECOND		(unsigned long)(2000/TASK_GSM_TIMEOUT)

/* EVENTS GPS*/
#define   EVENT_GPS_NULL       		(unsigned char)0
#define   EVENT_GPS_INIT       		(unsigned char)1
#define   EVENT_GPS_SEARCHING		(unsigned char)2
#define   EVENT_GPS_PARSE			(unsigned char)3
#define   EVENT_GPS_ACQUIRING		(unsigned char)4
#define   EVENT_GPS_REQ_SLEEPING	(unsigned char)5
#define   EVENT_GPS_SLEEPING		(unsigned char)6
#define   EVENT_GPS_WAKEUP  		(unsigned char)7

#define   TASKGPS_INIT             	(unsigned char)0 
#define   TASKGPS_SEARCHING        	(unsigned char)1
#define   TASKGPS_ACQUIRING        	(unsigned char)2
#define   TASKGPS_SLEEPING        	(unsigned char)3

#define TIMER_GPS_SEARCHING 	    (unsigned char)0 
#define TIMER_GPS_ACQUIRING	 		(unsigned char)1 
#define TIMER_SLEEP_MODE			(unsigned char)2
#define TIMER_GPS_CALLBACK			(unsigned char)3
#define TIMER_GPS_GSM_TIMEOUT		(unsigned char)4
#define TIMER_GPS_LAST   			(unsigned char)5 /*Always the last one*/

#define GPS_CALLBACK_TIMEOUT		(unsigned long)((unsigned long)50/(unsigned long)TASK_GPS_TIMEOUT)/*50ms*/
#define GPS_SEARCHING_TIMEOUT		(unsigned long)((unsigned long)1000/(unsigned long)TASK_GPS_TIMEOUT)/*1000ms*/
#define GPS_ACQUIRING_TIMEOUT		(unsigned long)((unsigned long)5000/(unsigned long)TASK_GPS_TIMEOUT)/*5000ms*/
#define GPS_SLEEP_MODE_TIMEOUT 		(unsigned long)((unsigned long)100/(unsigned long)TASK_GPS_TIMEOUT)/*100ms*/

/* EVENTS SD*/
#define EVENT_SD_NULL           					(unsigned char)0
#define EVENT_SD_WRITING            				(unsigned char)1
#define EVENT_SD_OPENING          					(unsigned char)2
#define EVENT_SD_READING          					(unsigned char)3
#define EVENT_SD_MARKING          					(unsigned char)4
#define EVENT_SD_ENDING								(unsigned char)5
#define EVENT_SD_INIT           					(unsigned char)6
#define EVENT_SD_TRANSMITTING       				(unsigned char)7
#define EVENT_SD_REMOVING           				(unsigned char)8
#define EVENT_SD_REQ_SLEEPING						(unsigned char)9
#define EVENT_SD_WAKEUP  							(unsigned char)10
#define EVENT_SD_SET_CONFIG_APN					    (unsigned char)11
#define EVENT_SD_ERROR								(unsigned char)12

#define EVENT_SD_WRITING_TEL1       				(unsigned char)13
#define EVENT_SD_WRITING_TEL2       				(unsigned char)14
#define EVENT_SD_WRITING_TEL3       				(unsigned char)15
#define EVENT_SD_READWRITE_CONFIG      				(unsigned char)16


 /* EVENTS DEBUG*/
#define EVENT_DEGUG_NULL           					(unsigned char)0
#define EVENT_USART_DEBUG               			(unsigned char)1
#define EVENT_USART_MODEM               			(unsigned char)2

/* States*/
#define TASKSD_INITIALIZING    		(unsigned char)0
#define TASKSD_READING	    		(unsigned char)1
#define TASKSD_SLEEPING	    		(unsigned char)2

/* Timers*/
#define TIMER_SD_RESPONSE			(unsigned char)0
#define TIMER_SD_TIMEOUT			(unsigned char)1
#define TIMER_SD_CALLBACK			(unsigned char)2
#define TIMER_SD_LAST   			(unsigned char)3 /*Always the last one*/

/* Events*/
#define EVENT_IO_GSM_NULL           (unsigned char)0
#define EVENT_IO_GSM_INIT           (unsigned char)1
#define EVENT_IO_GSM_CONNECTING    	(unsigned char)2
#define EVENT_IO_GSM_COMMUNICATING	(unsigned char)3
#define EVENT_IO_GSM_OFF		    (unsigned char)4
#define EVENT_IO_GSM_ERROR	    	(unsigned char)5
#define EVENT_IO_GSM_UPLOAD_DONE    (unsigned char)6
#define EVENT_IO_SLEEPING			(unsigned char)7

#define EVENT_IO_GPS_FIXED		    (unsigned char)8
#define EVENT_IO_GPS_NOT_FIXED      (unsigned char)9

/* States*/
#define TASKIO_GSM_INITIALIZING    	(unsigned char)0
#define TASKIO_GSM_CONNECTING    	(unsigned char)1
#define TASKIO_GSM_COMMUNICATING   	(unsigned char)2
#define TASKIO_GSM_UPLOADING  		(unsigned char)3

 /* Timers*/
#define TIMER_IO_WIFI				(unsigned char)0
#define TIMER_IO_GPS				(unsigned char)1
#define TIMER_IO_SD					(unsigned char)2
#define TIMER_IO_SD_TIMEOUT			(unsigned char)3
#define TIMER_IO_CALLBACK			(unsigned char)4
#define TIMER_IO_CALLBACK_SLEEPING	(unsigned char)5
#define TIMER_IO_TEMP_READING		(unsigned char)6
#define TIMER_IO_WIFI_TIMEOUT		(unsigned char)7
#define TIMER_IO_STOPMODE_TIMEOUT	(unsigned char)8
#define TIMER_IO_LAST   			(unsigned char)9/*Always the last one*/


/* Events*/
#define EVENT_WIFI_NULL         	(unsigned char)0
#define EVENT_WIFI_INIT         	(unsigned char)1
#define EVENT_WIFI_SEND_RESET  			(unsigned char)2
#define EVENT_WIFI_WAIT_RESP  		(unsigned char)3
#define EVENT_WIFI_SEND_REQ_MODE  		(unsigned char)4
#define EVENT_WIFI_SEND_MODE		(unsigned char)5
#define EVENT_WIFI_SEND_MUX			(unsigned char)6
#define EVENT_WIFI_SEND_CIP 		(unsigned char)7
#define EVENT_WIFI_SEND_AP 			(unsigned char)8
#define EVENT_WIFI_SEND_START 		(unsigned char)9
#define EVENT_WIFI_SEND_CIPSEND 	(unsigned char)10
#define EVENT_WIFI_SEND_DATA 		(unsigned char)11
#define EVENT_WIFI_ENDING 			(unsigned char)12
#define EVENT_WIFI_WATCHING_AP		(unsigned char)13
#define EVENT_WIFI_AP_CONNECTED 	(unsigned char)14
#define EVENT_WIFI_AP_DISCONNECTED	(unsigned char)15
#define EVENT_WIFI_REQ_SLEEPING		(unsigned char)16
#define EVENT_WIFI_ERROR  			(unsigned char)17
#define EVENT_WIFI_WAKEUP  			(unsigned char)18
#define EVENT_WIFI_GET_STATUS		(unsigned char)19
#define EVENT_WIFI_SEND_RECONNECT	(unsigned char)20
#define EVENT_WIFI_IDLE  			(unsigned char)21
#define EVENT_WIFI_CONNECTION_OK	(unsigned char)22
#define EVENT_WIFI_IO_IGNON			(unsigned char)23
#define EVENT_WIFI_IO_IGNOFF		(unsigned char)24
#define EVENT_WIFI_OPENING_FILE_OK  (unsigned char)25
/* States*/
#define TASKWIFI_IDLING    			(unsigned char)0
#define TASKWIFI_INITIALIZING    	(unsigned char)1
#define TASKWIFI_COMMUNICATING		(unsigned char)2
#define TASKWIFI_WATCHING			(unsigned char)3
#define TASKWIFI_SLEEPING			(unsigned char)4

/* Events*/
#define EVENT_GSM_NULL         				(unsigned char)0
#define EVENT_GSM_INIT         				(unsigned char)1
#define EVENT_GSM_AT  						(unsigned char)2
#define EVENT_GSM_WAIT_RESP  				(unsigned char)3
#define EVENT_GSM_SEND_CPIN	  				(unsigned char)4
#define EVENT_GSM_SEND_CSQ					(unsigned char)5
#define EVENT_GSM_SEND_CIPSEND				(unsigned char)6
#define EVENT_GSM_SEND_CIP 					(unsigned char)7
#define EVENT_GSM_SEND_CSTT 				(unsigned char)8
#define EVENT_GSM_SEND_START 				(unsigned char)9
#define EVENT_GSM_SEND_CREG					(unsigned char)10
#define EVENT_GSM_SEND_DATA 		(unsigned char)11
#define EVENT_GSM_SEND_CIFSR 		(unsigned char)12
#define EVENT_GSM_SEND_CGATT		(unsigned char)13
#define EVENT_GSM_SEND_CGDCONT		(unsigned char)14
#define EVENT_GSM_SEND_CIPSHUT		(unsigned char)15
#define EVENT_GSM_REQ_SLEEPING		(unsigned char)16
#define EVENT_GSM_ERROR  			(unsigned char)17
#define EVENT_GSM_WAKEUP  			(unsigned char)18
#define EVENT_GSM_SEND_CIICR 		(unsigned char)19
#define EVENT_GSM_SEND_RECONNECT	(unsigned char)20
#define EVENT_GSM_IDLE  			(unsigned char)21
#define EVENT_GSM_SETBAUD			(unsigned char)22
#define EVENT_GSM_STARTING_TIMER 	(unsigned char)23
#define EVENT_GSM_ENDING_TIMER		(unsigned char)24
#define EVENT_GSM_DATA_OK			(unsigned char)25
#define EVENT_GSM_PREPARING			(unsigned char)26
#define EVENT_GSM_IO_IGNON			(unsigned char)27
#define EVENT_GSM_IO_IGNOFF			(unsigned char)28
#define EVENT_GSM_SMS_TEXT_MODE		(unsigned char)29
#define EVENT_GSM_LIST_SMS			(unsigned char)30
#define EVENT_GSM_SEND_CIPMUX		(unsigned char)31
#define EVENT_GSM_WEB_CONNECT		(unsigned char)32
#define EVENT_GSM_CONNECTION_OK   	(unsigned char)33
#define EVENT_GSM_ENDING 			(unsigned char)34
#define EVENT_GSM_OPENING_FILE_OK	(unsigned char)35


#define EVENT_GSM_SELECT_SMS_FORMAT			(unsigned char)36
#define EVENT_GSM_LIST_SMS_MSG				(unsigned char)37
#define EVENT_GSM_READ_SMS_MSG				(unsigned char)38
#define EVENT_GSM_DECODE_SMS_MSG			(unsigned char)39
#define EVENT_GSM_DELETE_SMS_MSG			(unsigned char)40
#define EVENT_GSM_PREPARE_SMS_RESP			(unsigned char)41
#define EVENT_GSM_SEND_SMS_RESP				(unsigned char)42

#define EVENT_GSM_GET_START 				(unsigned char)43

#define EVENT_GSM_SEND_HTTPINIT				(unsigned char)44
#define EVENT_GSM_SEND_HTTPCID				(unsigned char)45
#define EVENT_GSM_SEND_HTTPURL				(unsigned char)46
#define EVENT_GSM_SEND_HTTPCONTENT			(unsigned char)47
#define EVENT_GSM_SEND_HTTPPREPAREDATA		(unsigned char)48
#define EVENT_GSM_SEND_HTTPDATA				(unsigned char)49
#define EVENT_GSM_SEND_HTTPSSL 				(unsigned char)50
#define EVENT_GSM_SEND_HTTPACTION			(unsigned char)51

#define EVENT_GSM_SEND_BEARERSET1			(unsigned char)52
#define EVENT_GSM_SEND_BEARERSET2			(unsigned char)53
#define EVENT_GSM_SEND_BEARERSET3			(unsigned char)54

#define EVENT_GSM_SEND_HTTPREAD				(unsigned char)55
#define EVENT_GSM_SEND_HTTPTERM				(unsigned char)56
#define EVENT_GSM_SEND_BEARERSET4			(unsigned char)57
#define EVENT_GSM_SEND_BEARERSET21			(unsigned char)58
#define EVENT_GSM_SEND_BEARERSET22			(unsigned char)59

#define EVENT_GSM_READ_CSQ					(unsigned char)61
#define EVENT_GSM_SEND_CCID					(unsigned char)62

#define EVENT_GSM_SEND_BEARERSET33			(unsigned char)63
#define EVENT_GSM_GET_BEARER				(unsigned char)64
#define EVENT_GSM_SEND_CIPSTATUS			(unsigned char)65

#define EVENT_GSM_SEND_SMS_CSQ				(unsigned char)66
#define EVENT_GSM_SEND_SMS_CCID				(unsigned char)67
#define EVENT_GSM_SEND_SMS_GPS				(unsigned char)68

#define EVENT_GSM_SEND_CIPTKA				(unsigned char)69
#define EVENT_GSM_SEND_CIPMODE				(unsigned char)70
#define EVENT_GSM_GET_CCID					(unsigned char)71


#define EVENT_GSM_SEND_HTTPURL_TIMESTAMP	(unsigned char)72
#define EVENT_GSM_SEND_HTTPACTION_TIMESTAMP	(unsigned char)73
#define EVENT_GSM_SEND_HTTPREAD_TIMESTAMP	(unsigned char)74
#define EVENT_GSM_SEND_HTTPTERM_TIMESTAMP	(unsigned char)75

/* States*/
#define TASKGSM_IDLING    		    		(unsigned char)(0)
#define TASKGSM_INITIALIZING    			(unsigned char)(1)
#define	TASKGSM_COMMUNICATING				(unsigned char)(2)
#define	TASKGSM_SMSREADING					(unsigned char)(3)
#define TASKGSM_SLEEPING					(unsigned char)(4)

/* Timers*/
#define TIMER_GSM_RESPONSE					(unsigned char)0
/*#define TIMER_GSM_CALLBACK				(unsigned char)1*/
/*#define TIMER_GSM_INIT					(unsigned char)1*/
/*#define TIMER_GSM_LIST_SMS				(unsigned char)1*/
#define TIMER_GSM_TIMEOUT					(unsigned char)1
#define TIMER_GSM_SMS_SERVICE_TIMEOUT 		(unsigned char)2
#define TIMER_GSM_OPENING_FILE_TIMEOUT 		(unsigned char)3
#define TIMER_GSM_READING_FILE_TIMEOUT 		(unsigned char)4
#define TIMER_GSM_1SECOND_TIMEOUT			(unsigned char)5
#define TIMER_GSM_LAST   					(unsigned char)6/*Always the last one*/



 /* Events*/
#define EVENT_HTTPCLI_NULL         					(unsigned char)0
#define EVENT_HTTPCLI_INIT 							(unsigned char)1
#define EVENT_HTTPCLI_CONNECTING					(unsigned char)2
#define EVENT_HTTPCLI_CONNECTED 					(unsigned char)3
#define EVENT_HTTPCLI_POST         					(unsigned char)4
#define EVENT_HTTPCLI_POSTED         				(unsigned char)5
#define EVENT_HTTPCLI_DISCONNECTED					(unsigned char)6
#define EVENT_HTTPCLI_SYNCTS         				(unsigned char)7
#define EVENT_HTTPCLI_ENDING						(unsigned char)8
#define EVENT_HTTPCLI_ERROR							(unsigned char)9

 /* States*/
#define TASKHTTPCLI_IDLING			   				(unsigned char)(0)
#define TASKHTTPCLI_INITIALIZING    				(unsigned char)(1)
#define TASKHTTPCLI_COMMUNICATING    				(unsigned char)(2)


/* Installed sources*/
#define SRC_GSM     0
#define SRC_DEBUG   2
#define SRC_GPS     3
#define SRC_SD      4
#define	SRC_WIFI    5
#define SRC_HTTPCLI 6
#define SRC_BLE		0
#define SRC_IO		8
#define SRC_HTTPSRV 9

 /* The number of items the queue can hold.  This is 1 as the receive task
 will remove items as they are added, meaning the send task should always find
 the queue empty. */
#define debugQUEUE_LENGTH					( 5 )
#define sdQUEUE_LENGTH						( 5 )
#define gsmQUEUE_LENGTH						( 5 )
#define wifiQUEUE_LENGTH					( 5 )
#define httcliQUEUE_LENGTH					( 5	)
#define httsrvQUEUE_LENGTH					( 5	)
#define ultraQUEUE_LENGTH					( 5 )
#define bleQUEUE _LENGTH					( 10 )

 typedef struct{
 	unsigned long u32TimeToSleepInSec;
 	unsigned long u32PeriodLogInSec;
 	char cBuzzerStatus[RX_BUF_SIZE_REDUCED>>2];
 	char cWifiApName[RX_BUF_SIZE_REDUCED >>1];
 	char cWifiApPassword[RX_BUF_SIZE_REDUCED];
 	char cHttpDomain[RX_BUF_SIZE_REDUCED];
 	char cPageUrl[RX_BUF_SIZE_REDUCED];
 	char cModemApn[RX_BUF_SIZE_REDUCED];
 	char cModemApnLogin[RX_BUF_SIZE_REDUCED];
 	char cModemApnPassword[RX_BUF_SIZE_REDUCED];
 	unsigned long u32ModemPeriodTxInSec;
 	char cAppSource[RX_BUF_SIZE_REDUCED >>2];
 }tstConfiguration;

typedef struct
{
	unsigned char ucSelector;
 	unsigned char ucIgnition;
 	unsigned int u16AdMainBatteryVoltage;
 	float flAdMainBatteryVoltage;
 	unsigned int u16AdBackupBatteryVoltage;
 	float flAdBackupBatteryVoltage;
 	float flI2cTemperature;
 	unsigned int u16IntTemperature;
 	char cLatitude[20];
 	char cLongitude[20];
 	unsigned char ucNumSat;
 	unsigned char ulHdop;
 	int altitude;
 	int course;
 	int speedkmh;
	time_t timeSinceEpoch;
	struct tm t;
}tstIo;

typedef struct
{
    unsigned char   ucSrc;
    unsigned char   ucDest;
    unsigned char   ucEvent;
    char*           pcMessageData;
}sMessageType;

typedef struct
{
    /* event constructed from message  */
    unsigned char ucEvent;
    /* action routine invoked for the event */
    unsigned char (*ActionFun) (sMessageType *psMessage);
    /* next state in case of success*/
    unsigned char ucStateSuccess;
    /* next state in case of failure*/
    unsigned char ucStateFailure;
}sStateMachineType;

typedef struct{
    char cIndexMsg[4];
    char cPhoneNumber[20];/*"+5511991361858"*/
    char cMessageText[141];
}SMS_TYPE;

typedef struct{
    char cPhoneNumber[32];/*"+5511991361858"*/
}PHONE_TYPE;

typedef struct
{
	/* Command : GET or SET  */
	char *pcCommand;
	/* action routine invoked for the event */
	unsigned char (*FunctionPointer) (sMessageType *psMessage);
}sSmsCommandType;


/* IO MAPPING*/
#define GPIO_OUTPUT_GSM_DIAG    				12
#define GPIO_OUTPUT_GSM_DIAG_PIN_SEL  			((1ULL<<GPIO_OUTPUT_GSM_DIAG))

#define GPIO_OUTPUT_GSM_ENABLE    				23
#define GPIO_OUTPUT_GSM_ENABLE_PIN_SEL  		((1ULL<<GPIO_OUTPUT_GSM_ENABLE))

#define GPIO_ONE_WIRE							25

#define GPIO_OUTPUT_OUT1	    				26
#define GPIO_OUTPUT_OUT1_PIN_SEL  				((1ULL<<GPIO_OUTPUT_OUT1))

#define GPIO_OUTPUT_GPS_ENABLE    				14
#define GPIO_OUTPUT_GPS_ENABLE_PIN_SEL  		((1ULL<<GPIO_OUTPUT_GPS_ENABLE))

#define GPIO_OUTPUT_GSM_DIAG_EXT   				33
#define GPIO_OUTPUT_GSM_DIAG_EXT_PIN_SEL  		((1ULL<<GPIO_OUTPUT_GSM_DIAG_EXT))

#define GPIO_INPUT_IGNITION    					36
#define GPIO_INPUT_IGNITION_PIN_SEL  			((1ULL<<GPIO_INPUT_IGNITION))

#define GPIO_INPUT_ADC_MAINBATTERY 				39
#define GPIO_INPUT_ADC_PIN_SEL  				((1ULL<<GPIO_INPUT_ADC_MAINBATTERY))

#define GPIO_OUTPUT_ULTRA_TRIGGER  				27
#define GPIO_OUTPUT_ULTRA_TRIGGER_PIN_SEL  		((1ULL<<GPIO_OUTPUT_ULTRA_TRIGGER))

#define GPIO_INPUT_SELECTOR    					21
#define GPIO_INPUT_SELECTOR_PIN_SEL  			((1ULL<<GPIO_INPUT_SELECTOR))



#define MODEM_TX 					18/*	UART2-RX	*/
#define MODEM_RX 					19/*	UART2-TX	*/

#define GPS_TX	 					16/*	UART1-RX	*/
#define GPS_RX 						17/*	UART1-TX	*/



/*#define GPS_ENABLE		PB1
#define DIAG_GPS			PB12
#define IGNITION			PB15*/

#define MAIN_VOLTAGE		PA0 (MAIN VOLTAGE = (ADVALUE/4096*3.3)*10
#define BATTERY_VOLTAGE		PA1	(BATTERY VOLTAGE = (ADVALUE/4096*3.3)*10


#ifdef __cplusplus
}
#endif
#endif
