#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_system.h>
#include "esp_netif.h"
#include <esp_http_server.h>

#include "http_server.h"
#include "wifi_manager.h"


/* @brief tag used for ESP serial console messages */
static const char TAG[] = "mihome_esp32_http_server";

/* @brief the HTTP server handle */
static httpd_handle_t httpd_handle = NULL;

/**
 * @brief embedded binary data.
 * @see file "component.mk"
 * @see https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html#embedding-binary-data
 */
extern const uint8_t style_css_start[] asm("_binary_style_css_start");
extern const uint8_t style_css_end[]   asm("_binary_style_css_end");
extern const uint8_t code_js_start[] asm("_binary_code_js_start");
extern const uint8_t code_js_end[] asm("_binary_code_js_end");
extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[] asm("_binary_index_html_end");


/* const httpd related values stored in ROM */
const static char http_200_hdr[] = "200 OK";
const static char http_302_hdr[] = "302 Found";
const static char http_400_hdr[] = "400 Bad Request";
const static char http_404_hdr[] = "404 Not Found";
const static char http_503_hdr[] = "503 Service Unavailable";
const static char http_location_hdr[] = "Location";
const static char http_content_type_html[] = "text/html";
const static char http_content_type_js[] = "text/javascript";
const static char http_content_type_css[] = "text/css";
const static char http_content_type_json[] = "application/json";
const static char http_cache_control_hdr[] = "Cache-Control";
const static char http_cache_control_no_cache[] = "no-store, no-cache, must-revalidate, max-age=0";
const static char http_cache_control_cache[] = "public, max-age=31536000";
const static char http_pragma_hdr[] = "Pragma";
const static char http_pragma_no_cache[] = "no-cache";



static esp_err_t http_server_delete_handler(httpd_req_t *req){

	ESP_LOGD(TAG, "DELETE %s", req->uri);

	if(strcmp(req->uri, "/connect.json") == 0){
		wifi_manager_disconnect_async();

		httpd_resp_set_status(req, http_200_hdr);
		httpd_resp_set_type(req, http_content_type_json);
		httpd_resp_set_hdr(req, http_cache_control_hdr, http_cache_control_no_cache);
		httpd_resp_set_hdr(req, http_pragma_hdr, http_pragma_no_cache);
		httpd_resp_send(req, NULL, 0);
	}
	else{
		httpd_resp_set_status(req, http_404_hdr);
		httpd_resp_send(req, NULL, 0);
	}

	return ESP_OK;
}


static esp_err_t http_server_post_handler(httpd_req_t *req){


	ESP_LOGD(TAG, "POST %s", req->uri);

	if(strcmp(req->uri, "/connect.json") == 0){


		/* buffers for the headers */
		size_t ssid_len = 0, password_len = 0;
		char *ssid = NULL, *password = NULL;

		/* len of values provided */
		ssid_len = httpd_req_get_hdr_value_len(req, "X-Custom-ssid");
		password_len = httpd_req_get_hdr_value_len(req, "X-Custom-pwd");

		if(ssid_len && ssid_len <= MAX_SSID_SIZE && password_len && password_len <= MAX_PASSWORD_SIZE){
			/* get the actual value of the headers */
			ssid = malloc(sizeof(char) * (ssid_len + 1));
			password = malloc(sizeof(char) * (password_len + 1));
			httpd_req_get_hdr_value_str(req, "X-Custom-ssid", ssid, ssid_len+1);
			httpd_req_get_hdr_value_str(req, "X-Custom-pwd", password, password_len+1);

			wifi_config_t* config = wifi_manager_get_wifi_sta_config();
			memset(config, 0x00, sizeof(wifi_config_t));
			memcpy(config->sta.ssid, ssid, ssid_len);
			memcpy(config->sta.password, password, password_len);
			ESP_LOGD(TAG, "http_server_post_handler: wifi_manager_connect_async() call");
			wifi_manager_connect_async();

			/* free memory */
			free(ssid);
			free(password);

			httpd_resp_set_status(req, http_200_hdr);
			httpd_resp_set_type(req, http_content_type_json);
			httpd_resp_set_hdr(req, http_cache_control_hdr, http_cache_control_no_cache);
			httpd_resp_set_hdr(req, http_pragma_hdr, http_pragma_no_cache);
			httpd_resp_send(req, NULL, 0);

		}
		else{
			/* bad request the authentification header is not complete/not the correct format */
			httpd_resp_set_status(req, http_400_hdr);
			httpd_resp_send(req, NULL, 0);
		}

	}
	else{
		httpd_resp_set_status(req, http_404_hdr);
		httpd_resp_send(req, NULL, 0);
	}

	return ESP_OK;
}


static esp_err_t http_server_get_handler(httpd_req_t *req){

    char* host = NULL;
    size_t buf_len;

    ESP_LOGD(TAG, "GET %s", req->uri);

    /* Get header value string length and allocate memory for length + 1,
     * extra byte for null termination */
    buf_len = httpd_req_get_hdr_value_len(req, "Host") + 1;
    if (buf_len > 1) {
    	host = malloc(buf_len);
    	if(httpd_req_get_hdr_value_str(req, "Host", host, buf_len) != ESP_OK){
    		/* if something is wrong we just 0 the whole memory */
    		memset(host, 0x00, buf_len);
    	}
    }

	/* determine if Host is from the STA IP address */
	wifi_manager_lock_sta_ip_string(portMAX_DELAY);
	bool access_from_sta_ip = host != NULL?strstr(host, wifi_manager_get_sta_ip_string()):false;
	wifi_manager_unlock_sta_ip_string();


	if (host != NULL && !strstr(host, DEFAULT_AP_IP) && !access_from_sta_ip) {

		/* Captive Portal functionality */
		/* 302 Redirect to IP of the access point */

		size_t location_size = 23; /* size: http://255.255.255.255 */
		char* buff_location = malloc(sizeof(char) * location_size);
		*buff_location = '\0';
		snprintf(buff_location, location_size, "http://%s", DEFAULT_AP_IP);
		httpd_resp_set_status(req, http_302_hdr);
		httpd_resp_set_hdr(req, http_location_hdr, buff_location);
		httpd_resp_send(req, NULL, 0);
		free(buff_location);

	}
	else{

		ESP_LOGD(TAG, "GET %s", req->uri);

		if(strcmp(req->uri, "/") == 0){
			httpd_resp_set_status(req, http_200_hdr);
			httpd_resp_set_type(req, http_content_type_html);
			httpd_resp_send(req, (char*)index_html_start, index_html_end - index_html_start);
		}
		else if(strcmp(req->uri, "/code.js") == 0){
			httpd_resp_set_status(req, http_200_hdr);
			httpd_resp_set_type(req, http_content_type_js);
			httpd_resp_send(req, (char*)code_js_start, code_js_end - code_js_start);
		}
		else if(strcmp(req->uri, "/style.css") == 0){
			httpd_resp_set_status(req, http_200_hdr);
			httpd_resp_set_type(req, http_content_type_css);
			httpd_resp_set_hdr(req, http_cache_control_hdr, http_cache_control_cache);
			httpd_resp_send(req, (char*)style_css_start, style_css_end - style_css_start);
		}
		else if(strcmp(req->uri, "/ap.json") == 0){

			/* if we can get the mutex, write the last version of the AP list */
			if(wifi_manager_lock_json_buffer(( TickType_t ) 10)){

				httpd_resp_set_status(req, http_200_hdr);
				httpd_resp_set_type(req, http_content_type_json);
				httpd_resp_set_hdr(req, http_cache_control_hdr, http_cache_control_no_cache);
				httpd_resp_set_hdr(req, http_pragma_hdr, http_pragma_no_cache);
				char* ap_buf = wifi_manager_get_ap_list_json();
				httpd_resp_send(req, ap_buf, strlen(ap_buf));
				wifi_manager_unlock_json_buffer();
			}
			else{
				httpd_resp_set_status(req, http_503_hdr);
				httpd_resp_send(req, NULL, 0);
				ESP_LOGE(TAG, "http_server_netconn_serve: GET /ap.json failed to obtain mutex");
			}

			/* request a wifi scan */
			wifi_manager_scan_async();
		}
		else if(strcmp(req->uri, "/status.json") == 0){

			if(wifi_manager_lock_json_buffer(( TickType_t ) 10)){
				char *buff = wifi_manager_get_ip_info_json();
				if(buff){
					httpd_resp_set_status(req, http_200_hdr);
					httpd_resp_set_type(req, http_content_type_json);
					httpd_resp_set_hdr(req, http_cache_control_hdr, http_cache_control_no_cache);
					httpd_resp_set_hdr(req, http_pragma_hdr, http_pragma_no_cache);
					httpd_resp_send(req, buff, strlen(buff));
					wifi_manager_unlock_json_buffer();
				}
				else{
					httpd_resp_set_status(req, http_503_hdr);
					httpd_resp_send(req, NULL, 0);
				}
			}
			else{
				httpd_resp_set_status(req, http_503_hdr);
				httpd_resp_send(req, NULL, 0);
				ESP_LOGE(TAG, "http_server_netconn_serve: GET /status.json failed to obtain mutex");
			}
		}
		else{
			httpd_resp_set_status(req, http_404_hdr);
			httpd_resp_send(req, NULL, 0);
		}

	}

    /* memory clean up */
    if(host != NULL){
    	free(host);
    }

    return ESP_OK;

}

/* URI wild card for any GET request */
static const httpd_uri_t http_server_get_request = {
    .uri       = "*",
    .method    = HTTP_GET,
    .handler   = http_server_get_handler
};

static const httpd_uri_t http_server_post_request = {
	.uri	= "/*",
	.method = HTTP_POST,
	.handler = http_server_post_handler
};

static const httpd_uri_t http_server_delete_request = {
	.uri	= "/*",
	.method = HTTP_DELETE,
	.handler = http_server_delete_handler
};


void http_server_start(){

	esp_err_t err;

	if(httpd_handle == NULL){

		httpd_config_t config = HTTPD_DEFAULT_CONFIG();

		/* this is an important option that isn't set up by default.
		 * We could register all URLs one by one, but this would not work while the fake DNS is active */
		config.uri_match_fn = httpd_uri_match_wildcard;

		err = httpd_start(&httpd_handle, &config);

	    if (err == ESP_OK) {
	        ESP_LOGD(TAG, "Registering URI handlers");
	        httpd_register_uri_handler(httpd_handle, &http_server_get_request);
	        httpd_register_uri_handler(httpd_handle, &http_server_post_request);
	        httpd_register_uri_handler(httpd_handle, &http_server_delete_request);
	    }
	}

}
