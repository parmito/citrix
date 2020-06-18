/* ESP HTTP SERVER Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <sys/param.h>
#include "nvs_flash.h"
#include "tcpip_adapter.h"
#include "esp_eth.h"

#include "freertos/queue.h"
#include "defines.h"
#include "Sd.h"

#include "http_server.h"

static const char *TAG = "HTTP_SERVER";
sMessageType stHttpSrvMsg;
extern char cConfigHttpRxBuffer[RX_BUF_SIZE];
extern tstConfiguration stConfigData;
/* An HTTP GET handler */
static esp_err_t Read_get_handler(httpd_req_t *req)
{
    char*  buf;
    size_t buf_len;

    /* Get header value string length and allocate memory for length + 1,
     * extra byte for null termination */
    buf_len = httpd_req_get_hdr_value_len(req, "Host") + 1;
    if (buf_len > 1) {
        buf = malloc(buf_len);
        /* Copy null terminated value string into buffer */
        if (httpd_req_get_hdr_value_str(req, "Host", buf, buf_len) == ESP_OK) {
            ESP_LOGI(TAG, "Found header => Host: %s", buf);
        }
        free(buf);
    }

    buf_len = httpd_req_get_hdr_value_len(req, "Test-Header-2") + 1;
    if (buf_len > 1) {
        buf = malloc(buf_len);
        if (httpd_req_get_hdr_value_str(req, "Test-Header-2", buf, buf_len) == ESP_OK) {
            ESP_LOGI(TAG, "Found header => Test-Header-2: %s", buf);
        }
        free(buf);
    }

    buf_len = httpd_req_get_hdr_value_len(req, "Test-Header-1") + 1;
    if (buf_len > 1) {
        buf = malloc(buf_len);
        if (httpd_req_get_hdr_value_str(req, "Test-Header-1", buf, buf_len) == ESP_OK) {
            ESP_LOGI(TAG, "Found header => Test-Header-1: %s", buf);
        }
        free(buf);
    }

    /* Read URL query string length and allocate memory for length + 1,
     * extra byte for null termination */
    buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len > 1) {
        buf = malloc(buf_len);
        if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
            ESP_LOGI(TAG, "Found URL query => %s", buf);
            char param[32];
            /*static char cKey[128];*/

            memset(cConfigHttpRxBuffer,0x00,sizeof(cConfigHttpRxBuffer));

            /* Get value of expected key from query string */
            if (httpd_query_key_value(buf, "ap", param, sizeof(param)) == ESP_OK) {
                ESP_LOGI(TAG, "Found URL query parameter => ap=%s", param);
            }
            if (httpd_query_key_value(buf, "appass", param, sizeof(param)) == ESP_OK) {
                ESP_LOGI(TAG, "Found URL query parameter => appass=%s", param);
            }
            if (httpd_query_key_value(buf, "log", param, sizeof(param)) == ESP_OK) {
                ESP_LOGI(TAG, "Found URL query parameter => log=%s", param);
            }

            if (httpd_query_key_value(buf, "tx", param, sizeof(param)) == ESP_OK) {
                ESP_LOGI(TAG, "Found URL query parameter => tx=%s", param);
            }

            if (httpd_query_key_value(buf, "src", param, sizeof(param)) == ESP_OK) {
                ESP_LOGI(TAG, "Found URL query parameter => src=%s", param);
            }

            if (httpd_query_key_value(buf, "apn", param, sizeof(param)) == ESP_OK) {
                ESP_LOGI(TAG, "Found URL query parameter => apn=%s", param);
            }

            if (httpd_query_key_value(buf, "login", param, sizeof(param)) == ESP_OK) {
                ESP_LOGI(TAG, "Found URL query parameter => login=%s", param);
            }

            if (httpd_query_key_value(buf, "mdpass", param, sizeof(param)) == ESP_OK) {
                ESP_LOGI(TAG, "Found URL query parameter => mdpass=%s", param);
            }

            if (httpd_query_key_value(buf, "sleep", param, sizeof(param)) == ESP_OK) {
                ESP_LOGI(TAG, "Found URL query parameter => sleep=%s", param);
            }
        }
        free(buf);
    }

    /* Set some custom headers */
    httpd_resp_set_hdr(req, "Custom-Header-1", "Custom-Value-1");
    httpd_resp_set_hdr(req, "Custom-Header-2", "Custom-Value-2");

    /* Send response with custom headers and body set as the
     * string passed in user context*/
    static char cWebpage[2048];
    strcpy(cWebpage,"<!DOCTYPE html> <html>\n");
    strcat(cWebpage,"<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=no\">\n");
    strcat(cWebpage,"<title>CITRIX WEB SERBER</title>\n");
    strcat(cWebpage,"<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}\n");
	strcat(cWebpage,"input[type=text] {");
	  strcat(cWebpage,"width: 50%;");
	  strcat(cWebpage,"padding: 12px 20px;");
	  strcat(cWebpage,"margin: 8px 0;");
	  strcat(cWebpage,"box-sizing: border-box;");
	  strcat(cWebpage,"border: none;");
	  strcat(cWebpage,"background-color: #34495e;");
	  strcat(cWebpage,"color: white;");
	strcat(cWebpage,"}");
    strcat(cWebpage,"body{margin-top: 50px;} h1 {color: #444444;margin: 50px auto 30px;} h3 {color: #444444;margin-bottom: 50px;}\n");
    strcat(cWebpage,".button {display: block;width: 120px;background-color: #3498db;border: none;color: white;padding: 13px 30px;text-decoration: none;font-size: 25px;margin: 0px auto 35px;cursor: pointer;border-radius: 4px;}\n");
    strcat(cWebpage,".button-on {background-color: #3498db;}\n");
    strcat(cWebpage,".button-on:active {background-color: #2980b9;}\n");
    strcat(cWebpage,".button-off {background-color: #34495e;}\n");
    strcat(cWebpage,".button-off:active {background-color: #2c3e50;}\n");
    strcat(cWebpage,".font-custom-type {font-size: 18px;color: #888;margin-bottom: 0px;}\n");
    strcat(cWebpage,"</style>\n");
    strcat(cWebpage,"</head>\n");
    strcat(cWebpage,"<body>\n");
    strcat(cWebpage,"<h1>CITRIX Web Server</h1>\n");
    strcat(cWebpage,"<h2>Using SOFT AP Mode</h2>\n");
	strcat(cWebpage,"<form action=\"/response\"  method=\"get\">");

	strcat(cWebpage,"<label for=\"sw\">SW:</label>");

	char cHtml[256];
	sprintf(cHtml,"<input type=\"text\" id=\"sw\" name=\"sw\" value=\"%s\"><br>",SOFTWARE_VERSION);
	strcat(cWebpage,cHtml);

	strcat(cWebpage,"<label for=\"wifiapname\">Access Point:</label>");
	sprintf(cHtml,"<input type=\"text\" id=\"wifiapname\" name=\"ap\" value=\"%s\"><br>",stConfigData.cWifiApName);
	strcat(cWebpage,cHtml);

	strcat(cWebpage,"<label for=\"wifiappassword\">Password:</label>");
	sprintf(cHtml,"<input type=\"text\" id=\"wifiappassword\" name=\"appass\" value=\"%s\"><br>",stConfigData.cWifiApPassword);
	strcat(cWebpage,cHtml);

	strcat(cWebpage,"<label for=\"periodlog\">Period Log:</label>");
	sprintf(cHtml,"<input type=\"text\" id=\"periodlog\" name=\"log\" value=\"%ld\"><br>",stConfigData.u32PeriodLogInSec);
	strcat(cWebpage,cHtml);

	strcat(cWebpage,"<label for=\"periodtx\">Period Tx:</label>");
	sprintf(cHtml,"<input type=\"text\" id=\"periodtx\" name=\"tx\" value=\"%ld\"><br>",stConfigData.u32ModemPeriodTxInSec);
	strcat(cWebpage,cHtml);

	strcat(cWebpage,"<label for=\"appsource\">Choose an way to upload data:</label>");
	strcat(cWebpage,"<select name=\"src\" id=\"appsource\">");
	  strcat(cWebpage,"<option label=\"WIFI\">WIFI</option>");
	  strcat(cWebpage,"<option label=\"GPRS\">GPRS</option>");
	strcat(cWebpage,"</select><br>");

	strcat(cWebpage,"<label for=\"modemapn\">APN:</label>");
	sprintf(cHtml,"<input type=\"text\" id=\"modemapn\" name=\"apn\" value=\"%s\"><br>",stConfigData.cModemApn);
	strcat(cWebpage,cHtml);

	strcat(cWebpage,"<label for=\"modemapnlogin\">APN Login:</label>");
	sprintf(cHtml,"<input type=\"text\" id=\"modemapnlogin\" name=\"login\" value=\"%s\"><br>",stConfigData.cModemApnLogin);
	strcat(cWebpage,cHtml);

	strcat(cWebpage,"<label for=\"modemapnpassword\">APN Password:</label>");
	sprintf(cHtml,"<input type=\"text\" id=\"modemapnpassword\" name=\"mdpass\" value=\"%s\"><br>",stConfigData.cModemApnPassword);
	strcat(cWebpage,cHtml);

	strcat(cWebpage,"<label for=\"timesleep\">Time to Sleep:</label>");
	sprintf(cHtml,"<input type=\"text\" id=\"timesleep\" name=\"sleep\" value=\"%ld\"><br><br>",stConfigData.u32TimeToSleepInSec);
	strcat(cWebpage,cHtml);

	strcat(cWebpage,"<button class=\"button button-off\">Send</button>");
	strcat(cWebpage,"</form>");
    strcat(cWebpage,"</body>\n");
	strcat(cWebpage,"</html>\n");

    const char* resp_str = (const char*) /*req->user_ctx*/cWebpage;
    httpd_resp_send(req, resp_str, strlen(resp_str));

    /* After sending the HTTP response the old HTTP request
     * headers are lost. Check if HTTP request headers can be read now. */
    if (httpd_req_get_hdr_value_len(req, "Host") == 0) {
        ESP_LOGI(TAG, "Request headers lost");
    }
    return ESP_OK;
}



static esp_err_t Response_get_handler(httpd_req_t *req)
{
    char*  buf;
    size_t buf_len;

    /* Get header value string length and allocate memory for length + 1,
     * extra byte for null termination */
    buf_len = httpd_req_get_hdr_value_len(req, "Host") + 1;
    if (buf_len > 1) {
        buf = malloc(buf_len);
        /* Copy null terminated value string into buffer */
        if (httpd_req_get_hdr_value_str(req, "Host", buf, buf_len) == ESP_OK) {
            ESP_LOGI(TAG, "Found header => Host: %s", buf);
        }
        free(buf);
    }

    buf_len = httpd_req_get_hdr_value_len(req, "Test-Header-2") + 1;
    if (buf_len > 1) {
        buf = malloc(buf_len);
        if (httpd_req_get_hdr_value_str(req, "Test-Header-2", buf, buf_len) == ESP_OK) {
            ESP_LOGI(TAG, "Found header => Test-Header-2: %s", buf);
        }
        free(buf);
    }

    buf_len = httpd_req_get_hdr_value_len(req, "Test-Header-1") + 1;
    if (buf_len > 1) {
        buf = malloc(buf_len);
        if (httpd_req_get_hdr_value_str(req, "Test-Header-1", buf, buf_len) == ESP_OK) {
            ESP_LOGI(TAG, "Found header => Test-Header-1: %s", buf);
        }
        free(buf);
    }

    /* Read URL query string length and allocate memory for length + 1,
     * extra byte for null termination */
    buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len > 1) {
        buf = malloc(buf_len);
        if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
            ESP_LOGI(TAG, "Found URL query => %s", buf);
            char param[32];
            static char cKey[128];

            memset(cConfigHttpRxBuffer,0x00,sizeof(cConfigHttpRxBuffer));

            /* Get value of expected key from query string */
            if (httpd_query_key_value(buf, "ap", param, sizeof(param)) == ESP_OK) {
                ESP_LOGI(TAG, "Found URL query parameter => ap=%s", param);

                if(strlen(param)> 0){
                const char *cWifiApName = "WIFIAPNAME:";
                memset(cKey,0,sizeof(cKey));
                strcpy(cKey,cWifiApName);
                strcat(cKey,param);
                strcat(cKey,"\r\n");

                strcat(cConfigHttpRxBuffer,"CONF:");
                strcat(cConfigHttpRxBuffer,cKey);
                ESP_LOGI(TAG, "%s\r\n", cConfigHttpRxBuffer);
                }
            }
            if (httpd_query_key_value(buf, "appass", param, sizeof(param)) == ESP_OK) {
                ESP_LOGI(TAG, "Found URL query parameter => appass=%s", param);

                if(strlen(param)> 0){
                const char *cWifiApPassword = "WIFIAPPASSWORD:";
                memset(cKey,0,sizeof(cKey));
                strcpy(cKey,cWifiApPassword);
                strcat(cKey,param);
                strcat(cKey,"\r\n");


                strcat(cConfigHttpRxBuffer,"CONF:");
                strcat(cConfigHttpRxBuffer,cKey);
                ESP_LOGI(TAG, "%s\r\n", cConfigHttpRxBuffer);
                }
            }
            if (httpd_query_key_value(buf, "log", param, sizeof(param)) == ESP_OK) {
                ESP_LOGI(TAG, "Found URL query parameter => log=%s", param);

                if(strlen(param)> 0){
                const char *cPeriodLogInSec = "PERIODLOGINSEC:";
                memset(cKey,0,sizeof(cKey));
                strcpy(cKey,cPeriodLogInSec);
                strcat(cKey,param);
                strcat(cKey,"\r\n");


                strcat(cConfigHttpRxBuffer,"CONF:");
                strcat(cConfigHttpRxBuffer,cKey);
                ESP_LOGI(TAG, "%s\r\n", cConfigHttpRxBuffer);
                }
            }

            if (httpd_query_key_value(buf, "tx", param, sizeof(param)) == ESP_OK) {
                ESP_LOGI(TAG, "Found URL query parameter => tx=%s", param);

                if(strlen(param)> 0){
                const char *cPeriodTx = "MODEMPERIODTXINSEC:";
                memset(cKey,0,sizeof(cKey));
                strcpy(cKey,cPeriodTx);
                strcat(cKey,param);
                strcat(cKey,"\r\n");

                strcat(cConfigHttpRxBuffer,"CONF:");
                strcat(cConfigHttpRxBuffer,cKey);
                ESP_LOGI(TAG, "%s\r\n", cConfigHttpRxBuffer);
                }
            }

            if (httpd_query_key_value(buf, "src", param, sizeof(param)) == ESP_OK) {
                ESP_LOGI(TAG, "Found URL query parameter => src=%s", param);

                if(strlen(param)> 0){
                const char *cAppSrc = "APPSOURCE:";
                memset(cKey,0,sizeof(cKey));
                strcpy(cKey,cAppSrc);
                strcat(cKey,param);
                strcat(cKey,"\r\n");

                strcat(cConfigHttpRxBuffer,"CONF:");
                strcat(cConfigHttpRxBuffer,cKey);
                ESP_LOGI(TAG, "%s\r\n", cConfigHttpRxBuffer);
                }
            }

            if (httpd_query_key_value(buf, "apn", param, sizeof(param)) == ESP_OK) {
                ESP_LOGI(TAG, "Found URL query parameter => apn=%s", param);

                if(strlen(param)> 0){
                const char *cModemApn = "MODEMAPN:";
                memset(cKey,0,sizeof(cKey));
                strcpy(cKey,cModemApn);
                strcat(cKey,param);
                strcat(cKey,"\r\n");


                strcat(cConfigHttpRxBuffer,"CONF:");
                strcat(cConfigHttpRxBuffer,cKey);
                ESP_LOGI(TAG, "%s\r\n", cConfigHttpRxBuffer);
                }
            }

            if (httpd_query_key_value(buf, "login", param, sizeof(param)) == ESP_OK) {
                ESP_LOGI(TAG, "Found URL query parameter => login=%s", param);

                if(strlen(param)> 0){
                const char *cModemLogin = "MODEMAPNLOGIN:";
                memset(cKey,0,sizeof(cKey));
                strcpy(cKey,cModemLogin);
                strcat(cKey,param);
                strcat(cKey,"\r\n");


                strcat(cConfigHttpRxBuffer,"CONF:");
                strcat(cConfigHttpRxBuffer,cKey);
                ESP_LOGI(TAG, "%s\r\n", cConfigHttpRxBuffer);
                }
            }

            if (httpd_query_key_value(buf, "mdpass", param, sizeof(param)) == ESP_OK) {
                ESP_LOGI(TAG, "Found URL query parameter => mdpass=%s", param);

                if(strlen(param)> 0){
                const char *cModemPassword = "MODEMAPNPASSWORD:";
                memset(cKey,0,sizeof(cKey));
                strcpy(cKey,cModemPassword);
                strcat(cKey,param);
                strcat(cKey,"\r\n");


                strcat(cConfigHttpRxBuffer,"CONF:");
                strcat(cConfigHttpRxBuffer,cKey);
                ESP_LOGI(TAG, "%s\r\n", cConfigHttpRxBuffer);
                }
            }

            if (httpd_query_key_value(buf, "sleep", param, sizeof(param)) == ESP_OK) {
                ESP_LOGI(TAG, "Found URL query parameter => sleep=%s", param);

                if(strlen(param)> 0){
                const char *cTimeToSleep = "TIMETOSLEEP:";
                memset(cKey,0,sizeof(cKey));
                strcpy(cKey,cTimeToSleep);
                strcat(cKey,param);
                strcat(cKey,"\r\n");


                strcat(cConfigHttpRxBuffer,"CONF:");
                strcat(cConfigHttpRxBuffer,cKey);
                ESP_LOGI(TAG, "%s\r\n", cConfigHttpRxBuffer);
                }
            }

            if(strlen(cConfigHttpRxBuffer) > strlen("CONF:")){
            /* Receive data over BT and pass it over to SD*/
            stHttpSrvMsg.ucSrc = SRC_HTTPSRV;
            stHttpSrvMsg.ucDest = SRC_SD;
            stHttpSrvMsg.ucEvent = EVENT_SD_READWRITE_CONFIG;
            stHttpSrvMsg.pcMessageData = (char*)cConfigHttpRxBuffer;

			xQueueSend( xQueueSd, ( void * )&stHttpSrvMsg, NULL);
            }

        }
        free(buf);
    }

    /* Set some custom headers */
    httpd_resp_set_hdr(req, "Custom-Header-1", "Custom-Value-1");
    httpd_resp_set_hdr(req, "Custom-Header-2", "Custom-Value-2");

    /* Send response with custom headers and body set as the
     * string passed in user context*/
    const char* resp_str = (const char*) req->user_ctx;
    httpd_resp_send(req, resp_str, strlen(resp_str));

    /* After sending the HTTP response the old HTTP request
     * headers are lost. Check if HTTP request headers can be read now. */
    if (httpd_req_get_hdr_value_len(req, "Host") == 0) {
        ESP_LOGI(TAG, "Request headers lost");
    }
    return ESP_OK;
}


/*
<p>LED1 Status: ON</p><a class=\"button button-off\" href=\"/led1off\">OFF</a>\n\
<p>LED2 Status: ON</p><a class=\"button button-off\" href=\"/led2off\">OFF</a>\n\
*/

static const httpd_uri_t response = {
    .uri       = "/response",
    .method    = HTTP_GET,
    .handler   = Response_get_handler,
    /* Let's pass response string in user
     * context to demonstrate it's usage */
    .user_ctx  =
    	    "<!DOCTYPE html> <html>\n\
    	    <head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=no\">\n\
    	    <title>CITRIX WEB SERBER</title>\n\
    	    <style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}\n\
    		input[type=text] {\
    		  width: 50%;\
    		  padding: 12px 20px;\
    		  margin: 8px 0;\
    		  box-sizing: border-box;\
    		  border: none;\
    		  background-color: #34495e;\
    		  color: white;\
    		}\
    	    body{margin-top: 50px;} h1 {color: #444444;margin: 50px auto 30px;} h3 {color: #444444;margin-bottom: 50px;}\n\
    	    .button {display: block;width: 120px;background-color: #3498db;border: none;color: white;padding: 13px 30px;text-decoration: none;font-size: 25px;margin: 0px auto 35px;cursor: pointer;border-radius: 4px;}\n\
    	    .button-on {background-color: #3498db;}\n\
    	    .button-on:active {background-color: #2980b9;}\n\
    	    .button-off {background-color: #34495e;}\n\
    	    .button-off:active {background-color: #2c3e50;}\n\
    	    p {font-size: 18px;color: #888;margin-bottom: 0px;}\n\
    	    </style>\n\
    	    </head>\n\
    	    <body>\n\
    	    <h1>CITRIX Web Server</h1>\n\
    	    <h2>Using SOFT AP Mode</h2>\n\
    	    <h3>Requested Parameters have been recorded</h3>\n\
    	    </body>\n\
    		</html>\n"
};

static const httpd_uri_t Read = {
    .uri       = "/get",
    .method    = HTTP_GET,
    .handler   = Read_get_handler,
    /* Let's pass response string in user
     * context to demonstrate it's usage */
    .user_ctx  = NULL
};


/* An HTTP POST handler */
static esp_err_t echo_post_handler(httpd_req_t *req)
{
    char buf[100];
    int ret, remaining = req->content_len;


    while (remaining > 0) {
    	ESP_LOGI(TAG, "remaining > 0 ");
        /* Read the data for the request */
        if ((ret = httpd_req_recv(req, buf,
                        MIN(remaining, sizeof(buf)))) <= 0) {
            if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
                /* Retry receiving if timeout occurred */
                continue;
            }
            return ESP_FAIL;
        }

        /* Send back the same data */
        httpd_resp_send_chunk(req, buf, ret);
        remaining -= ret;

        /* Log data received */
        ESP_LOGI(TAG, "=========== RECEIVED DATA ==========");
        ESP_LOGI(TAG, "%.*s", ret, buf);
        ESP_LOGI(TAG, "====================================");
    }

    // End response
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

static const httpd_uri_t echo = {
    .uri       = "/echo",
    .method    = HTTP_POST,
    .handler   = echo_post_handler,
    .user_ctx  = "HTTP_POST"
};

/* This handler allows the custom error handling functionality to be
 * tested from client side. For that, when a PUT request 0 is sent to
 * URI /ctrl, the /hello and /echo URIs are unregistered and following
 * custom error handler http_404_error_handler() is registered.
 * Afterwards, when /hello or /echo is requested, this custom error
 * handler is invoked which, after sending an error message to client,
 * either closes the underlying socket (when requested URI is /echo)
 * or keeps it open (when requested URI is /hello). This allows the
 * client to infer if the custom error handler is functioning as expected
 * by observing the socket state.
 */
esp_err_t http_404_error_handler(httpd_req_t *req, httpd_err_code_t err)
{
    if (strcmp("/hello", req->uri) == 0) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "/hello URI is not available");
        /* Return ESP_OK to keep underlying socket open */
        return ESP_OK;
    } else if (strcmp("/echo", req->uri) == 0) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "/echo URI is not available");
        /* Return ESP_FAIL to close underlying socket */
        return ESP_FAIL;
    }
    /* For any other URI send 404 and close socket */
    httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Some 404 error message");
    return ESP_FAIL;
}

/* An HTTP PUT handler. This demonstrates realtime
 * registration and deregistration of URI handlers
 */
static esp_err_t ctrl_put_handler(httpd_req_t *req)
{
    char buf;
    int ret;

    if ((ret = httpd_req_recv(req, &buf, 1)) <= 0) {
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
            httpd_resp_send_408(req);
        }
        return ESP_FAIL;
    }

    if (buf == '0') {
        /* URI handlers can be unregistered using the uri string */
        ESP_LOGI(TAG, "Unregistering /hello and /echo URIs");
        httpd_unregister_uri(req->handle, "/hello");
        httpd_unregister_uri(req->handle, "/echo");
        /* Register the custom error handler */
        httpd_register_err_handler(req->handle, HTTPD_404_NOT_FOUND, http_404_error_handler);
    }
    else {
        ESP_LOGI(TAG, "Registering /hello and /echo URIs");
        httpd_register_uri_handler(req->handle, &Read);
        httpd_register_uri_handler(req->handle, &echo);
        /* Unregister custom error handler */
        httpd_register_err_handler(req->handle, HTTPD_404_NOT_FOUND, NULL);
    }

    /* Respond with empty body */
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

static const httpd_uri_t ctrl = {
    .uri       = "/ctrl",
    .method    = HTTP_PUT,
    .handler   = ctrl_put_handler,
    .user_ctx  = NULL
};

httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    // Start the httpd server
    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        // Set URI handlers
        ESP_LOGI(TAG, "Registering URI handlers");
        httpd_register_uri_handler(server, &Read);
        httpd_register_uri_handler(server, &response);
        httpd_register_uri_handler(server, &echo);
        httpd_register_uri_handler(server, &ctrl);
        return server;
    }

    ESP_LOGI(TAG, "Error starting server!");
    return NULL;
}

void stop_webserver(httpd_handle_t server)
{
    // Stop the httpd server
    httpd_stop(server);
}


