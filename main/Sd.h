/*
 * Sd.h
 *
 *  Created on: 20/09/2018
 *      Author: danilo
 */

#ifndef SD_H_
#define SD_H_


xQueueHandle xQueueSd;

void SdInit(void);
void vTaskSd( void *pvParameters );

#endif /* SD_H_ */
