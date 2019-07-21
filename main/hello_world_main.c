#include <esp_wifi.h>
#include <esp_event_loop.h>
#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <sys/param.h>
#include <esp_http_server.h>
#include <esp_camera.h>

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

	.pixel_format = PIXFORMAT_RGB565,
	.frame_size = FRAMESIZE_VGA,

	.jpeg_quality = 12,
	.fb_count = 1
};

esp_err_t hello_get_handler(httpd_req_t *req)
{
	size_t query_buf_len = httpd_req_get_url_query_len(req) + 1;
	if(query_buf_len > 1){
		char key[8], value[8], read[8];
		char* query_buf = malloc(query_buf_len);
		if (httpd_req_get_url_query_str(req, query_buf, query_buf_len) == ESP_OK){
			ESP_LOGI(TAG, "Found URL query => %s", query_buf);

			// write
			if(httpd_query_key_value(query_buf, "key", key, sizeof(key)) == ESP_OK && httpd_query_key_value(query_buf, "value", value, sizeof(value)) == ESP_OK){
				int key_int = atoi(key);
				int value_int = atoi(value);

				SCCB_Write(sensor->slv_addr, key_int, value_int);
				ESP_LOGI(TAG, "write => key=%02X, value=%02X", key_int, value_int);
			}

			// read
			if(httpd_query_key_value(query_buf, "read", read, sizeof(read)) == ESP_OK){
				int read_int = atoi(read);
				ESP_LOGI(TAG, "read => regist=%02X, value=%02X", read_int, SCCB_Read(sensor->slv_addr, read_int));
			}
		}
		free(query_buf);
	}

	camera_fb_t * fb = NULL;
	esp_err_t res = ESP_OK;
	int64_t fr_start = esp_timer_get_time();

	fb = esp_camera_fb_get();
	if( ! fb){
		ESP_LOGE(TAG, "Camera capture failed");
		httpd_resp_send_500(req);
		return ESP_FAIL;
	}

	uint8_t * buf = NULL;
	size_t buf_len = 0;
	bool converted = frame2bmp(fb, &buf, &buf_len);
	esp_camera_fb_return(fb);
	if( ! converted){
		ESP_LOGE(TAG, "BMP conversion failed");
		httpd_resp_send_500(req);
		return ESP_FAIL;
	}

	res = httpd_resp_set_type(req, "image/bmp") || httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.bmp") || httpd_resp_send(req, (const char *)buf, buf_len);
	free(buf);
	int64_t fr_end = esp_timer_get_time();
	ESP_LOGI(TAG, "BMP: %uKB %ums", (uint32_t)(buf_len/1024), (uint32_t)((fr_end - fr_start)/1000));
	return res;
}

httpd_uri_t hello = {
	.uri = "/hello",
	.method = HTTP_GET,
	.handler = hello_get_handler,
	.user_ctx = "Hello World!"
};

httpd_handle_t start_webserver(void)
{
	httpd_handle_t server = NULL;
	httpd_config_t config = HTTPD_DEFAULT_CONFIG();

	ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
	if(httpd_start(&server, &config) == ESP_OK){
		ESP_LOGI(TAG, "Registering URI handlers");
		httpd_register_uri_handler(server, &hello);
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
