/*
 * http_server.h
 *
 *  Created on: Jun 8, 2020
 *      Author: danilo
 */

#ifndef MAIN_HTTP_SERVER_H_
#define MAIN_HTTP_SERVER_H_

#include <esp_http_server.h>

xQueueHandle xQueueHttpSrv;
httpd_handle_t start_webserver(void);
void stop_webserver(httpd_handle_t server);



#endif /* MAIN_HTTP_SERVER_H_ */
