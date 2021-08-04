/*
 * Debug.c
 *
 *  Created on: 24/09/2018
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

#include <esp_system.h>
#include <esp_spi_flash.h>
#include <rom/spi_flash.h>
#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/ledc.h"
#include "esp_adc_cal.h"

#include "UartGsm.h"
#include "State.h"
#include "defines.h"
#include "Sd.h"
#include "Debug.h"
#include "Gsm.h"

#define LEDC_HS_TIMER          LEDC_TIMER_0
#define LEDC_HS_MODE           LEDC_HIGH_SPEED_MODE
#define LEDC_HS_CH0_GPIO       (18)
#define LEDC_HS_CH0_CHANNEL    LEDC_CHANNEL_0
#define LEDC_HS_CH1_GPIO       (19)
#define LEDC_HS_CH1_CHANNEL    LEDC_CHANNEL_1

#define LEDC_LS_TIMER          LEDC_TIMER_1
#define LEDC_LS_MODE           LEDC_LOW_SPEED_MODE
#define LEDC_LS_CH2_GPIO       (4)
#define LEDC_LS_CH2_CHANNEL    LEDC_CHANNEL_2
#define LEDC_LS_CH3_GPIO       (5)
#define LEDC_LS_CH3_CHANNEL    LEDC_CHANNEL_3

#define LEDC_TEST_CH_NUM       (2)
#define LEDC_TEST_DUTY         (4096)
#define LEDC_TEST_FADE_TIME    (1000)


//////////////////////////////////////////////
//
//
//            FUNCTION PROTOTYPES
//
//
//////////////////////////////////////////////
void vTaskDebug( void *pvParameters );

//////////////////////////////////////////////
//
//
//            VARIABLES
//
//
//////////////////////////////////////////////
static uint32_t ulCountPulseGsm = 0;
static uint32_t ulCountPeriodGsm = 0;

/*static uint32_t ulQtyPulseGsm = 0;
static uint32_t ulPeriodGsm = 0;

static uint32_t ulQtyPulseGps = 1;
static uint32_t ulPeriodGps = 4;*/

sMessageType stDebugMsg;
/*static bool boGpsHasFixed = false;*/
static const char *DEBUG_TASK_TAG = "DEBUG_TASK";

//////////////////////////////////////////////
//
//
//              Debug_Io_Configuration
//
//
//////////////////////////////////////////////
void Debug_Io_Configuration(void)
{
#if 0
	/////////////////
	// GSM DIAG PIN
	/////////////////
	gpio_config_t io_conf;
	//disable interrupt
	io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
	//set as output mode
	io_conf.mode = GPIO_MODE_OUTPUT;
	//bit mask of the pins that you want to set,e.g.GPIO18/19
	io_conf.pin_bit_mask = GPIO_OUTPUT_GSM_DIAG_PIN_SEL | GPIO_OUTPUT_GSM_DIAG_EXT_PIN_SEL;
	//disable pull-down mode
	io_conf.pull_down_en = 0;
	//disable pull-up mode
	io_conf.pull_up_en = 0;
	//configure GPIO with the given settings
	gpio_config(&io_conf);
#endif
#if 0
	int ch;
	/*
	 * Prepare and set configuration of timers
	 * that will be used by LED Controller
	 */
	ledc_timer_config_t ledc_timer = {
		.duty_resolution = LEDC_TIMER_13_BIT, // resolution of PWM duty
		.freq_hz = 5000,                      // frequency of PWM signal
		.speed_mode = LEDC_HS_MODE,           // timer mode
		.timer_num = LEDC_HS_TIMER            // timer index
	};
	// Set configuration of timer0 for high speed channels
	ledc_timer_config(&ledc_timer);

	// Prepare and set configuration of timer1 for low speed channels
	ledc_timer.speed_mode = LEDC_LS_MODE;
	ledc_timer.timer_num = LEDC_LS_TIMER;
	ledc_timer_config(&ledc_timer);

	ledc_channel_config_t ledc_channel[LEDC_TEST_CH_NUM] = {
		{
			.channel    = LEDC_LS_CH2_CHANNEL,
			.duty       = 0,
			.gpio_num   = GPIO_OUTPUT_GSM_DIAG,
			.speed_mode = LEDC_LS_MODE,
			.hpoint     = 0,
			.timer_sel  = LEDC_LS_TIMER
		},
		{
			.channel    = LEDC_LS_CH3_CHANNEL,
			.duty       = 0,
			.gpio_num   = GPIO_OUTPUT_GSM_DIAG_EXT,
			.speed_mode = LEDC_LS_MODE,
			.hpoint     = 0,
			.timer_sel  = LEDC_LS_TIMER
		},
	};

	// Set LED Controller with previously prepared configuration
	for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
		ledc_channel_config(&ledc_channel[ch]);
	}

    // Initialize fade service.

    ledc_fade_func_install(0);

    while (1) {
        printf("1. LEDC fade up to duty = %d\n", LEDC_TEST_DUTY);
        for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
            ledc_set_fade_with_time(ledc_channel[ch].speed_mode,
                    ledc_channel[ch].channel, LEDC_TEST_DUTY, LEDC_TEST_FADE_TIME);
            ledc_fade_start(ledc_channel[ch].speed_mode,
                    ledc_channel[ch].channel, LEDC_FADE_NO_WAIT);
        }
        vTaskDelay(LEDC_TEST_FADE_TIME / portTICK_PERIOD_MS);

        printf("2. LEDC fade down to duty = 0\n");
        for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
            ledc_set_fade_with_time(ledc_channel[ch].speed_mode,
                    ledc_channel[ch].channel, 0, LEDC_TEST_FADE_TIME);
            ledc_fade_start(ledc_channel[ch].speed_mode,
                    ledc_channel[ch].channel, LEDC_FADE_NO_WAIT);
        }
        vTaskDelay(LEDC_TEST_FADE_TIME / portTICK_PERIOD_MS);

        printf("3. LEDC set duty = %d without fade\n", LEDC_TEST_DUTY);
        for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
            ledc_set_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel, LEDC_TEST_DUTY);
            ledc_update_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel);
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        printf("4. LEDC set duty = 0 without fade\n");
        for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
            ledc_set_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel, 0);
            ledc_update_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel);
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
#endif

}

//////////////////////////////////////////////
//
//
//              Debug_Init
//
//
//////////////////////////////////////////////
void DebugInit(void)
{
	/*esp_log_level_set(DEBUG_TASK_TAG, ESP_LOG_INFO);*/
#if 0
    xQueueDebug = xQueueCreate(sdQUEUE_LENGTH,			/* The number of items the queue can hold. */
							sizeof( sMessageType ) );	/* The size of each item the queue holds. */
#endif

    xTaskCreate(vTaskDebug, "vTaskDebug", 1024*2, NULL, configMAX_PRIORITIES, NULL);
	/* Create the queue used by the queue send and queue receive tasks.
	http://www.freertos.org/a00116.html */

	Debug_Io_Configuration();
}


//////////////////////////////////////////////
//
//
//              TaskDebug_Gsm_Init
//
//
//////////////////////////////////////////////
unsigned char TaskDebug_Gsm_Init(sMessageType *psMessage)
{
    unsigned char boError = true;

/*	gpio_set_level(GPIO_OUTPUT_GSM_DIAG, 0);
	gpio_set_level(GPIO_OUTPUT_GSM_DIAG_EXT, 0);*/
    uint32_t u32FreqHz = 1;
	int ch;

	/*
	 * Prepare and set configuration of timers
	 * that will be used by LED Controller
	 */
	ledc_timer_config_t ledc_timer = {
		.duty_resolution = LEDC_TIMER_13_BIT, // resolution of PWM duty
		.freq_hz =  u32FreqHz,                // frequency of PWM signal
		.speed_mode = LEDC_HS_MODE,           // timer mode
		.timer_num = LEDC_HS_TIMER            // timer index
	};
	// Set configuration of timer0 for high speed channels
	ledc_timer_config(&ledc_timer);


	ledc_channel_config_t ledc_channel[LEDC_TEST_CH_NUM] = {
		{
			.channel    = LEDC_LS_CH2_CHANNEL,
			.duty       = 2000,
			.gpio_num   = GPIO_OUTPUT_GSM_DIAG,
			.speed_mode = LEDC_HS_MODE,
			.hpoint     = 0,
			.timer_sel  = LEDC_HS_TIMER
		},
		{
			.channel    = LEDC_LS_CH3_CHANNEL,
			.duty       = 2000,
			.gpio_num   = GPIO_OUTPUT_GSM_DIAG_EXT,
			.speed_mode = LEDC_HS_MODE,
			.hpoint     = 0,
			.timer_sel  = LEDC_HS_MODE
		},
	};

	// Set LED Controller with previously prepared configuration
	for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
		ledc_channel_config(&ledc_channel[ch]);
	}

	for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
		ledc_set_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel, LEDC_TEST_DUTY);
		ledc_update_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel);
	}
    return(boError);
}

//////////////////////////////////////////////
//
//
//              TaskDebug_Gsm_Connecting
//
//
//////////////////////////////////////////////
unsigned char TaskDebug_Gsm_Connecting(sMessageType *psMessage)
{
    unsigned char boError = true;

    ESP_LOGI(DEBUG_TASK_TAG, "TaskDebug_Gsm_Connecting\r\n");

    return(boError);
}
//////////////////////////////////////////////
//
//
//              TaskDebug_Gsm_Communicating
//
//
//////////////////////////////////////////////
unsigned char TaskDebug_Gsm_Communicating(sMessageType *psMessage)
{
    unsigned char boError = true;

    return(boError);
}

//////////////////////////////////////////////
//
//
//              TaskDebug_Gsm_UploadDone
//
//
//////////////////////////////////////////////
unsigned char TaskDebug_Gsm_UploadDone(sMessageType *psMessage)
{
    unsigned char boError = true;
    int ch;
    uint32_t u32FreqHz = 4;

    ESP_LOGI(DEBUG_TASK_TAG, "TaskDebug_Gsm_UploadDone\r\n");

	/*
	 * Prepare and set configuration of timers
	 * that will be used by LED Controller
	 */
	ledc_timer_config_t ledc_timer = {
		.duty_resolution = LEDC_TIMER_13_BIT, // resolution of PWM duty
		.freq_hz =  u32FreqHz,                // frequency of PWM signal
		.speed_mode = LEDC_HS_MODE,           // timer mode
		.timer_num = LEDC_HS_TIMER            // timer index
	};

	// Set configuration of timer0 for high speed channels
	ledc_timer_config(&ledc_timer);


	ledc_channel_config_t ledc_channel[LEDC_TEST_CH_NUM] = {
		{
			.channel    = LEDC_LS_CH2_CHANNEL,
			.duty       = 0,
			.gpio_num   = GPIO_OUTPUT_GSM_DIAG,
			.speed_mode = LEDC_HS_MODE,
			.hpoint     = 0,
			.timer_sel  = LEDC_HS_TIMER
		},
		{
			.channel    = LEDC_LS_CH3_CHANNEL,
			.duty       = 0,
			.gpio_num   = GPIO_OUTPUT_GSM_DIAG_EXT,
			.speed_mode = LEDC_HS_MODE,
			.hpoint     = 0,
			.timer_sel  = LEDC_HS_MODE
		},
	};

	// Set LED Controller with previously prepared configuration
	for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
		ledc_channel_config(&ledc_channel[ch]);
	}

	for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
		ledc_set_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel, LEDC_TEST_DUTY);
		ledc_update_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel);
	}

    return(boError);
}

//////////////////////////////////////////////
//
//
//              TaskDebug_Gps_NotFixed
//
//
//////////////////////////////////////////////
unsigned char TaskDebug_Gps_NotFixed(sMessageType *psMessage)
{
    unsigned char boError = true;

/*	gpio_set_level(GPIO_OUTPUT_GSM_DIAG, 0);
	gpio_set_level(GPIO_OUTPUT_GSM_DIAG_EXT, 0);*/
    uint32_t u32FreqHz = 1;
	int ch;

	/*
	 * Prepare and set configuration of timers
	 * that will be used by LED Controller
	 */
	ledc_timer_config_t ledc_timer = {
		.duty_resolution = LEDC_TIMER_13_BIT, // resolution of PWM duty
		.freq_hz =  u32FreqHz,                // frequency of PWM signal
		.speed_mode = LEDC_HS_MODE,           // timer mode
		.timer_num = LEDC_HS_TIMER            // timer index
	};
	// Set configuration of timer0 for high speed channels
	ledc_timer_config(&ledc_timer);


	ledc_channel_config_t ledc_channel[LEDC_TEST_CH_NUM] = {
		{
			.channel    = LEDC_LS_CH2_CHANNEL,
			.duty       = 0,
			.gpio_num   = GPIO_OUTPUT_GSM_DIAG,
			.speed_mode = LEDC_HS_MODE,
			.hpoint     = 0,
			.timer_sel  = LEDC_HS_TIMER
		},
		{
			.channel    = LEDC_LS_CH3_CHANNEL,
			.duty       = 0,
			.gpio_num   = GPIO_OUTPUT_GSM_DIAG_EXT,
			.speed_mode = LEDC_HS_MODE,
			.hpoint     = 0,
			.timer_sel  = LEDC_HS_MODE
		},
	};

	// Set LED Controller with previously prepared configuration
	for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
		ledc_channel_config(&ledc_channel[ch]);
	}

	for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
		ledc_set_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel, LEDC_TEST_DUTY);
		ledc_update_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel);
	}
    return(boError);
}

//////////////////////////////////////////////
//
//
//              TaskDebug_Gps_Fixed
//
//
//////////////////////////////////////////////
unsigned char TaskDebug_Gps_Fixed(sMessageType *psMessage)
{
    unsigned char boError = true;

/*	gpio_set_level(GPIO_OUTPUT_GSM_DIAG, 0);
	gpio_set_level(GPIO_OUTPUT_GSM_DIAG_EXT, 0);*/
    uint32_t u32FreqHz = 2;
	int ch;

	/*
	 * Prepare and set configuration of timers
	 * that will be used by LED Controller
	 */
	ledc_timer_config_t ledc_timer = {
		.duty_resolution = LEDC_TIMER_13_BIT, // resolution of PWM duty
		.freq_hz =  u32FreqHz,                // frequency of PWM signal
		.speed_mode = LEDC_HS_MODE,           // timer mode
		.timer_num = LEDC_HS_TIMER            // timer index
	};
	// Set configuration of timer0 for high speed channels
	ledc_timer_config(&ledc_timer);


	ledc_channel_config_t ledc_channel[LEDC_TEST_CH_NUM] = {
		{
			.channel    = LEDC_LS_CH2_CHANNEL,
			.duty       = 0,
			.gpio_num   = GPIO_OUTPUT_GSM_DIAG,
			.speed_mode = LEDC_HS_MODE,
			.hpoint     = 0,
			.timer_sel  = LEDC_HS_TIMER
		},
		{
			.channel    = LEDC_LS_CH3_CHANNEL,
			.duty       = 0,
			.gpio_num   = GPIO_OUTPUT_GSM_DIAG_EXT,
			.speed_mode = LEDC_HS_MODE,
			.hpoint     = 0,
			.timer_sel  = LEDC_HS_MODE
		},
	};

	// Set LED Controller with previously prepared configuration
	for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
		ledc_channel_config(&ledc_channel[ch]);
	}

	for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
		ledc_set_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel, LEDC_TEST_DUTY);
		ledc_update_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel);
	}
    return(boError);


    return(boError);
}

//////////////////////////////////////////////
//
//
//              vTimerCallbackSleep
//
//
//////////////////////////////////////////////
#if 0
void vTimerCallbackSleep( xTimerHandle xTimer )
 {
    if(ulCountPeriodSleep <= u32TimeToSleep)
    {
    	ulCountPeriodSleep++;
    }
    else
    {
    	stDebugMsg.ucSrc = SRC_DEBUG;
    	stDebugMsg.ucDest = SRC_DEBUG;
    	stDebugMsg.ucEvent = EVENT_IO_SLEEPING;
		xQueueSend( xQueueDebug, ( void * )&stDebugMsg, 0);
    }
 }
#endif
//////////////////////////////////////////////
//
//
//          	vHandleGsmDiag
//
//
//////////////////////////////////////////////
void vHandleGsmDiag (void)
{
	/*	static unsigned char ucOnState = true;

	if(ulCountPulseGsm < ulQtyPulseGsm)
	{
		if(ucOnState != false)
		{
			gpio_set_level(GPIO_OUTPUT_GSM_DIAG, 1);
			gpio_set_level(GPIO_OUTPUT_GSM_DIAG_EXT, 1);
			ulCountPulseGsm++;
			ucOnState = false;
		}
		else
		{
			gpio_set_level(GPIO_OUTPUT_GSM_DIAG, 0);
			gpio_set_level(GPIO_OUTPUT_GSM_DIAG_EXT, 0);
			ucOnState = true;
		}
	}
	else
	{
		ulCountPeriodGsm = 0;
		ulCountPulseGsm = 0;

		gpio_set_level(GPIO_OUTPUT_GSM_DIAG, 0);
		gpio_set_level(GPIO_OUTPUT_GSM_DIAG_EXT, 0);
		ucOnState = true;
		vTaskDelay(1000/portTICK_PERIOD_MS);
	}
*/
}
//////////////////////////////////////////////
//
//
//              TaskGsm_IgnoreEvent
//
//
//////////////////////////////////////////////
unsigned char TaskDebug_Gsm_IgnoreEvent(sMessageType *psMessage)
{
    unsigned char boError = false;
    return(boError);
}
//////////////////////////////////////////////
//
//
//             Io Gsm State Machine
//
//
//////////////////////////////////////////////

static sStateMachineType const gasTaskDebug_Gsm_Initializing[] =
{
	/* Event		Action routine		Next state */
	//	State specific transitions

	{EVENT_IO_GSM_INIT,	   			TaskDebug_Gsm_Init,				TASKIO_GSM_INITIALIZING, 		TASKIO_GSM_INITIALIZING 	},
	{EVENT_IO_GSM_CONNECTING,		TaskDebug_Gsm_Connecting,		TASKIO_GSM_INITIALIZING,		TASKIO_GSM_INITIALIZING 	},
	{EVENT_IO_GPS_NOT_FIXED,		TaskDebug_Gps_NotFixed,			TASKIO_GSM_INITIALIZING,		TASKIO_GSM_INITIALIZING		},
	{EVENT_IO_GPS_FIXED,			TaskDebug_Gps_Fixed,			TASKIO_GSM_INITIALIZING,		TASKIO_GSM_INITIALIZING 	},
	{EVENT_IO_GSM_COMMUNICATING,	TaskDebug_Gsm_Communicating,	TASKIO_GSM_INITIALIZING,		TASKIO_GSM_INITIALIZING		},
	{EVENT_IO_GSM_UPLOAD_DONE,		TaskDebug_Gsm_UploadDone,		TASKIO_GSM_INITIALIZING,		TASKIO_GSM_INITIALIZING		},
	{EVENT_IO_GSM_NULL,    			TaskDebug_Gsm_IgnoreEvent,		TASKIO_GSM_INITIALIZING,		TASKIO_GSM_INITIALIZING		}
};
#if 0
static sStateMachineType const gasTaskDebug_Gsm_Connecting[] =
{
	/* Event		Action routine		Next state */
	//	State specific transitions
	{EVENT_IO_GSM_COMMUNICATING,	TaskDebug_Gsm_Communicating,	TASKIO_GSM_COMMUNICATING,		TASKIO_GSM_CONNECTING 	},
	{EVENT_IO_GPS_NOT_FIXED,		TaskDebug_Gps_NotFixed,			TASKIO_GSM_CONNECTING,			TASKIO_GSM_CONNECTING	},
	{EVENT_IO_GPS_FIXED,			TaskDebug_Gps_Fixed,			TASKIO_GSM_CONNECTING,			TASKIO_GSM_CONNECTING 	},
	{EVENT_IO_GSM_NULL,    			TaskDebug_Gsm_IgnoreEvent,		TASKIO_GSM_CONNECTING,			TASKIO_GSM_CONNECTING	}
};

static sStateMachineType const gasTaskDebug_Gsm_Communicating[] =
{
	/* Event		Action routine		Next state */
	//	State specific transitions
	{EVENT_IO_GSM_UPLOAD_DONE,		TaskDebug_Gsm_UploadDone,		TASKIO_GSM_UPLOADING,			TASKIO_GSM_UPLOADING 		},
	{EVENT_IO_GSM_CONNECTING,		TaskDebug_Gsm_Connecting,		TASKIO_GSM_CONNECTING,			TASKIO_GSM_COMMUNICATING 	},
	{EVENT_IO_GPS_NOT_FIXED,		TaskDebug_Gps_NotFixed,			TASKIO_GSM_COMMUNICATING,		TASKIO_GSM_COMMUNICATING 	},
	{EVENT_IO_GPS_FIXED,			TaskDebug_Gps_Fixed,			TASKIO_GSM_COMMUNICATING,		TASKIO_GSM_COMMUNICATING 	},
	{EVENT_IO_GSM_NULL,    			TaskDebug_Gsm_IgnoreEvent,		TASKIO_GSM_COMMUNICATING,		TASKIO_GSM_COMMUNICATING	}
};

static sStateMachineType const gasTaskDebug_Gsm_Uploading[] =
{
	{EVENT_IO_GSM_COMMUNICATING,	TaskDebug_Gsm_Communicating,	TASKIO_GSM_COMMUNICATING,		TASKIO_GSM_UPLOADING	},
	{EVENT_IO_GPS_NOT_FIXED,		TaskDebug_Gps_NotFixed,			TASKIO_GSM_UPLOADING,			TASKIO_GSM_UPLOADING 	},
	{EVENT_IO_GPS_FIXED,			TaskDebug_Gps_Fixed,			TASKIO_GSM_UPLOADING,			TASKIO_GSM_UPLOADING 	},
	{EVENT_IO_GSM_NULL,    			TaskDebug_Gsm_IgnoreEvent,		TASKIO_GSM_UPLOADING,			TASKIO_GSM_UPLOADING	}
};
#endif

static sStateMachineType const *const gpasTaskDebug_Sd_StateMachine[] =
{
	gasTaskDebug_Gsm_Initializing
/*	gasTaskDebug_Gsm_Connecting,
	gasTaskDebug_Gsm_Communicating,
	gasTaskDebug_Gsm_Uploading*/
};


static unsigned char ucCurrentStateIoSd = TASKIO_GSM_INITIALIZING;


void vTaskDebug( void *pvParameters )
{
    ESP_LOGI(DEBUG_TASK_TAG, __DATE__);
    ESP_LOGI(DEBUG_TASK_TAG,__TIME__);
    ESP_LOGI(DEBUG_TASK_TAG,"SOFTWARE_VERSION:%s",SOFTWARE_VERSION);
    ESP_LOGI(DEBUG_TASK_TAG,"\r\n");

	for( ;; )
	{
		if( xQueueReceive( xQueueDebug, &( stDebugMsg ),0 ) )
		{
			(void)eEventHandler ((unsigned char)SRC_DEBUG,gpasTaskDebug_Sd_StateMachine[ucCurrentStateIoSd], &ucCurrentStateIoSd, &stDebugMsg);
		}

		/*vHandleGsmDiag();*/
		vTaskDelay(1000/portTICK_PERIOD_MS);
	}
}


