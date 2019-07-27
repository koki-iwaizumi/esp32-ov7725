#include <string.h>
#include <esp_wifi.h>
#include <esp_event_loop.h>
#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <sys/param.h>
#include <esp_http_server.h>
#include <esp_camera.h>

#include "mbedtls/base64.h"
#include "cJSON.h"

#include "sensor.h"
#include "sccb.h"

/********* io ***********/
#define CAM_PIN_PWDN	2
#define CAM_PIN_RESET	15
#define CAM_PIN_XCLK	27
#define CAM_PIN_SIOD	25
#define CAM_PIN_SIOC	23
#define CAM_PIN_VSYNC	22
#define CAM_PIN_HREF	26
#define CAM_PIN_PCLK	21
#define CAM_PIN_D9		19
#define CAM_PIN_D8		36
#define CAM_PIN_D7		18
#define CAM_PIN_D6		39
#define CAM_PIN_D5		5
#define CAM_PIN_D4		34
#define CAM_PIN_D3		32
#define CAM_PIN_D2		35

/********* wifi ***********/
#define EXAMPLE_WIFI_SSID "*"
#define EXAMPLE_WIFI_PASS "*"

static const char *TAG = "APP";
sensor_t* sensor = NULL;

static camera_config_t camera_config = {
	.pin_pwdn  = CAM_PIN_PWDN,
	.pin_reset = CAM_PIN_RESET,
	.pin_xclk = CAM_PIN_XCLK,
	.pin_sscb_sda = CAM_PIN_SIOD,
	.pin_sscb_scl = CAM_PIN_SIOC,

	.pin_d7 = CAM_PIN_D9,
	.pin_d6 = CAM_PIN_D8,
	.pin_d5 = CAM_PIN_D7,
	.pin_d4 = CAM_PIN_D6,
	.pin_d3 = CAM_PIN_D5,
	.pin_d2 = CAM_PIN_D4,
	.pin_d1 = CAM_PIN_D3,
	.pin_d0 = CAM_PIN_D2,
	.pin_vsync = CAM_PIN_VSYNC,
	.pin_href = CAM_PIN_HREF,
	.pin_pclk = CAM_PIN_PCLK,

	.xclk_freq_hz = 6000000,
	.ledc_timer = LEDC_TIMER_0,
	.ledc_channel = LEDC_CHANNEL_0,

	.pixel_format = PIXFORMAT_YUV422,
	.frame_size = FRAMESIZE_VGA,

	.jpeg_quality = 12,
	.fb_count = 1
};

esp_err_t index_get_handler(httpd_req_t *req)
{
	extern const unsigned char index_start[] asm("_binary_index_html_start");
	extern const unsigned char index_end[]   asm("_binary_index_html_end");
	const size_t index_size = (index_end - index_start);

	httpd_resp_send_chunk(req, (const char *)index_start, index_size);
	httpd_resp_sendstr_chunk(req, NULL);

	return ESP_OK;
}

esp_err_t image_post_handler(httpd_req_t *req)
{
	int content_buf_size = 10 * 1000;
	char *content_buf = calloc(content_buf_size, sizeof(char));
    if(content_buf == NULL){
		ESP_LOGE(TAG, "Failed to allocate frame buffer - content_buf");
		httpd_resp_send_500(req);
		return ESP_FAIL;
    }

	if(httpd_req_recv(req, content_buf, req->content_len) <= 0){
		ESP_LOGE(TAG, "httpd_req_recv failed");
		httpd_resp_send_500(req);
		return ESP_FAIL;
	}

	ESP_LOGE(TAG, "httpd_req_recv content=%s", content_buf);

	cJSON *content_json = cJSON_Parse(content_buf);

	for(int i = 0; i <= 172; i++){
		char key[4];
		itoa(i,key,10);
		int value = strtol(cJSON_GetObjectItem(content_json, key)->valuestring, NULL, 16);

		SCCB_Write(sensor->slv_addr, i, value);
	}

	cJSON_Delete(content_json);
	free(content_buf);

	camera_fb_t *fb = NULL;
	esp_err_t res = ESP_OK;
	int64_t fr_start = esp_timer_get_time();

	fb = esp_camera_fb_get();
	if( ! fb){
		ESP_LOGE(TAG, "Camera capture failed");
		httpd_resp_send_500(req);
		return ESP_FAIL;
	}

	uint8_t *buf = NULL;
	size_t buf_len = 0;
	bool converted = frame2bmp(fb, &buf, &buf_len);
	esp_camera_fb_return(fb);
	if( ! converted){
		ESP_LOGE(TAG, "BMP conversion failed");
		httpd_resp_send_500(req);
		return ESP_FAIL;
	}

	ESP_LOGI(TAG, "esp_get_free_heap_size (%d bytes)", esp_get_free_heap_size());
	ESP_LOGI(TAG, "esp_get_minimum_free_heap_size (%d bytes)", esp_get_minimum_free_heap_size());
	ESP_LOGI(TAG, "heap_caps_get_free_size(MALLOC_CAP_8BIT) (%d bytes)", heap_caps_get_free_size(MALLOC_CAP_8BIT));
	ESP_LOGI(TAG, "heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT) (%d bytes)", heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT));
	ESP_LOGI(TAG, "heap_caps_get_largest_free_block(MALLOC_CAP_8BIT) (%d bytes)", heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));

	int image_buf_size = 1300 * 1000;
	uint8_t *image_buf = calloc(image_buf_size, sizeof(char));
    if(image_buf == NULL){
		free(buf);
		ESP_LOGE(TAG, "Failed to allocate frame buffer - image_buf");
		httpd_resp_send_500(req);
		return ESP_FAIL;
    }

	size_t olen = 0;
	int base64_err = mbedtls_base64_encode(image_buf, image_buf_size, &olen, buf, buf_len);
	free(buf);
	if (base64_err != 0) {
		ESP_LOGE(TAG, "error base64 encoding, error %d, buff size: %d", base64_err, olen);
		httpd_resp_send_500(req);
		return ESP_FAIL;
	}

	ESP_LOGI(TAG, "esp_get_free_heap_size (%d bytes)", esp_get_free_heap_size());
	ESP_LOGI(TAG, "esp_get_minimum_free_heap_size (%d bytes)", esp_get_minimum_free_heap_size());
	ESP_LOGI(TAG, "heap_caps_get_free_size(MALLOC_CAP_8BIT) (%d bytes)", heap_caps_get_free_size(MALLOC_CAP_8BIT));
	ESP_LOGI(TAG, "heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT) (%d bytes)", heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT));
	ESP_LOGI(TAG, "heap_caps_get_largest_free_block(MALLOC_CAP_8BIT) (%d bytes)", heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));

	uint8_t *image_json = calloc(image_buf_size, sizeof(char));
    if(image_json == NULL){
		free(image_buf);
		ESP_LOGE(TAG, "Failed to allocate frame buffer - image_json");
		httpd_resp_send_500(req);
		return ESP_FAIL;
    }

	sprintf((char *)image_json, "{\"image\":\"%s\"}", (const char *)image_buf);
	free(image_buf);
	ESP_LOGI(TAG, "image_json length=%d", strlen((const char*)image_json));

	res = httpd_resp_set_type(req, "application/json") || httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*") || httpd_resp_send(req, (const char *)image_json, strlen((const char*)image_json));
	free(image_json);

	int64_t fr_end = esp_timer_get_time();
	ESP_LOGI(TAG, "BMP: %uKB %ums", (uint32_t)(buf_len/1024), (uint32_t)((fr_end - fr_start)/1000));
	return res;
}

esp_err_t config_get_handler(httpd_req_t *req)
{
	int data_json_size = 10 * 1000;
	char *data_json = calloc(data_json_size, sizeof(char));
    if(data_json == NULL){
		ESP_LOGE(TAG, "Failed to allocate frame buffer - data_json");
		httpd_resp_send_500(req);
		return ESP_FAIL;
    }

	strcat(data_json, "{");
	for(int i = 0; i <= 172; i++){
		char data_json_buf[255] = {'\0'};
		sprintf(data_json_buf, "\"%d\":\"%d\"", i, SCCB_Read(sensor->slv_addr, i));
		strcat(data_json, data_json_buf);
		if(i != 172) strcat(data_json, ",");
	}
	strcat(data_json, "}");

	ESP_LOGI(TAG, "data_json=%s", data_json);
	ESP_LOGI(TAG, "data_json length=%d", strlen((const char*)data_json));

	esp_err_t res = httpd_resp_set_type(req, "application/json") || httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*") || httpd_resp_send(req, (const char *)data_json, strlen((const char*)data_json));
	free(data_json);

	return res;
}

httpd_uri_t index_uri = {
	.uri = "/",
	.method = HTTP_GET,
	.handler = index_get_handler,
};

httpd_uri_t image_uri = {
	.uri = "/image",
	.method = HTTP_POST,
	.handler = image_post_handler,
};

httpd_uri_t config_uri = {
	.uri = "/config",
	.method = HTTP_GET,
	.handler = config_get_handler,
};

httpd_handle_t start_webserver(void)
{
	httpd_handle_t server = NULL;
	httpd_config_t config = HTTPD_DEFAULT_CONFIG();

	ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
	if(httpd_start(&server, &config) == ESP_OK){
		ESP_LOGI(TAG, "Registering URI handlers");
		httpd_register_uri_handler(server, &index_uri);
		httpd_register_uri_handler(server, &image_uri);
		httpd_register_uri_handler(server, &config_uri);
		return server;
	}

	ESP_LOGI(TAG, "Error starting server!");
	return NULL;
}

void stop_webserver(httpd_handle_t server)
{
	httpd_stop(server);
}

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
	httpd_handle_t *server = (httpd_handle_t *) ctx;

	switch(event->event_id){
		case SYSTEM_EVENT_STA_START:
			ESP_LOGI(TAG, "SYSTEM_EVENT_STA_START");
			ESP_ERROR_CHECK(esp_wifi_connect());
			break;
		case SYSTEM_EVENT_STA_GOT_IP:
			ESP_LOGI(TAG, "SYSTEM_EVENT_STA_GOT_IP");
			ESP_LOGI(TAG, "Got IP: '%s'", ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));

			if(*server == NULL){
				*server = start_webserver();
			}
			break;
		case SYSTEM_EVENT_STA_DISCONNECTED:
			ESP_LOGI(TAG, "SYSTEM_EVENT_STA_DISCONNECTED");
			ESP_ERROR_CHECK(esp_wifi_connect());

			if(*server){
				stop_webserver(*server);
				*server = NULL;
			}
			break;
		default:
			break;
	}
	return ESP_OK;
}

static void initialise_wifi(void *arg)
{
	tcpip_adapter_init();
	ESP_ERROR_CHECK(esp_event_loop_init(event_handler, arg));
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&cfg));
	ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
	wifi_config_t wifi_config = {
		.sta = {
			.ssid = EXAMPLE_WIFI_SSID,
			.password = EXAMPLE_WIFI_PASS,
		},
	};
	ESP_LOGI(TAG, "Setting WiFi configuration SSID %s...", wifi_config.sta.ssid);
	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
	ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
	ESP_ERROR_CHECK(esp_wifi_start());
}

void app_main()
{
	static httpd_handle_t server = NULL;
	ESP_ERROR_CHECK(nvs_flash_init());

	esp_err_t err = esp_camera_init(&camera_config);
	if(err != ESP_OK){
		ESP_LOGE(TAG, "Camera Init Failed");
		return;
	}

	sensor = esp_camera_sensor_get();

	initialise_wifi(&server);
}
