#include <esp_system.h>
#include <nvs_flash.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_camera.h"
#include "esp_http_server.h"
#include "esp_timer.h"
#include "camera_pins.h"
#include "connect_wifi.h"
#include "driver/uart.h"

#define LED                     4
#define EX_UART_NUM             UART_NUM_0
#define BUF_SIZE                (1024)
#define RD_BUF_SIZE             (BUF_SIZE)
#define PART_BOUNDARY           "123456789000000000000987654321"
#define CONFIG_XCLK_FREQ        20000000 
#define TXD_PIN                 1
#define RXD_PIN                 3
#define WEB_SERVER              "api.thingspeak.com"
#define WEB_PORT                "80"

char REQUEST[512];
char recv_buf[512];
static const char *TAG = "ESP32-CAM";
static QueueHandle_t queue_data;
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";


httpd_handle_t setup_server(void);
esp_err_t jpg_stream_httpd_handler(httpd_req_t *req);
static void uart_event_task(void *pvParameters);
static void Init_Hardware(void);
static esp_err_t init_camera(void);
static void read_uart(void *arg);
static uint32_t GetMQ4Value(uint8_t *buf);
static void http_get_task(void *pvParameters);
static void camera_task(void *pvParameters);

httpd_uri_t uri_get = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = jpg_stream_httpd_handler,
    .user_ctx = NULL
};

void app_main()
{
    Init_Hardware();
    connect_wifi(); 
	xTaskCreate(&camera_task, "camera task", 1024*3, NULL, 3, NULL);        
	xTaskCreate(&read_uart, "UART task", 1024*3, NULL, 3, NULL);        
    xTaskCreate(&http_get_task, "http_get_task", 1024*4, NULL, 3, NULL);
}
static void camera_task(void *pvParameters)
{
    if (wifi_connect_status){
        ESP_ERROR_CHECK(init_camera());
        httpd_handle_t stream_httpd = setup_server();
        ESP_LOGI(TAG, "Web Server is up and running\n");
    }else{
        ESP_LOGI(TAG, "Failed to connecte with Wi-Fi, check your network credentials\n");
    }  
    while(1){
        vTaskDelay(10/portTICK_PERIOD_MS);
    }
}
static void read_uart(void *pvParameters)
{
    uint8_t temp = 0, hum = 0;
    uint32_t MQ4 = 0;
    uint8_t buff[RD_BUF_SIZE];
    bzero(buff, RD_BUF_SIZE);
    while(1){
        if(uart_read_bytes(EX_UART_NUM, buff, RD_BUF_SIZE, 2000 / portTICK_PERIOD_MS) > 0){
            temp = buff[0];
            hum = buff[1];
            MQ4 = GetMQ4Value(buff);
            uint32_t data[3] = {temp, hum, MQ4};
            if(xQueueSend(queue_data, (void*)data, 0) != pdTRUE){
                printf("Failed to send data to queue!\n");
            }
            printf("- Temp: %d \n- Hum: %d \n- MQ4: %ld \n\n", temp, hum, MQ4);
        }
        vTaskDelay(10/portTICK_PERIOD_MS);
    }
}

static void Init_Hardware(void){
    gpio_set_direction(LED, GPIO_MODE_OUTPUT);
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND){
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_flash_init();
    }
    //Install UART driver, and get the queue.
    uart_driver_install(EX_UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, NULL, 0);
    uart_param_config(EX_UART_NUM, &(uart_config_t) {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    });
    uart_set_pin(EX_UART_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    queue_data = xQueueCreate(10, sizeof(uint32_t)*3);
}

httpd_handle_t setup_server(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t stream_httpd  = NULL;
    if (httpd_start(&stream_httpd , &config) == ESP_OK){
        httpd_register_uri_handler(stream_httpd , &uri_get);
    }
    return stream_httpd;
}

static esp_err_t init_camera(void)
{
    camera_config_t camera_config = {
        .pin_pwdn  = CAM_PIN_PWDN,
        .pin_reset = CAM_PIN_RESET,
        .pin_xclk = CAM_PIN_XCLK,
        .pin_sccb_sda = CAM_PIN_SIOD,
        .pin_sccb_scl = CAM_PIN_SIOC,
        .pin_d7 = CAM_PIN_D7,
        .pin_d6 = CAM_PIN_D6,
        .pin_d5 = CAM_PIN_D5,
        .pin_d4 = CAM_PIN_D4,
        .pin_d3 = CAM_PIN_D3,
        .pin_d2 = CAM_PIN_D2,
        .pin_d1 = CAM_PIN_D1,
        .pin_d0 = CAM_PIN_D0,
        .pin_vsync = CAM_PIN_VSYNC,
        .pin_href = CAM_PIN_HREF,
        .pin_pclk = CAM_PIN_PCLK,
        .xclk_freq_hz = CONFIG_XCLK_FREQ,
        .ledc_timer = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,
        .pixel_format = PIXFORMAT_JPEG,
        .frame_size = FRAMESIZE_VGA,
        .jpeg_quality = 10,
        .fb_count = 1,
        .grab_mode = CAMERA_GRAB_WHEN_EMPTY
    };
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK){
        return err;
    }
    return ESP_OK;
}

esp_err_t jpg_stream_httpd_handler(httpd_req_t *req){
    camera_fb_t * fb = NULL;
    esp_err_t res = ESP_OK;
    size_t _jpg_buf_len;
    uint8_t * _jpg_buf;
    char * part_buf[64];
    static int64_t last_frame = 0;
    if(!last_frame) {
        last_frame = esp_timer_get_time();
    }
    res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    if(res != ESP_OK){
        return res;
    }

    while(true){
        fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGE(TAG, "Camera capture failed");
            res = ESP_FAIL;
            break;
        }
        if(fb->format != PIXFORMAT_JPEG){
            bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
            if(!jpeg_converted){
                ESP_LOGE(TAG, "JPEG compression failed");
                esp_camera_fb_return(fb);
                res = ESP_FAIL;
            }
        } else {
            _jpg_buf_len = fb->len;
            _jpg_buf = fb->buf;
        }

        if(res == ESP_OK){
            res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
        }
        if(res == ESP_OK){
            size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);

            res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
        }
        if(res == ESP_OK){
            res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
        }
        if(fb->format != PIXFORMAT_JPEG){
            free(_jpg_buf);
        }
        esp_camera_fb_return(fb);
        if(res != ESP_OK){
            break;
        }
        int64_t fr_end = esp_timer_get_time();
        int64_t frame_time = fr_end - last_frame;
        last_frame = fr_end;
        frame_time /= 1000;
    }
    last_frame = 0;
    return res;
}
static uint32_t GetMQ4Value(uint8_t *buf)
{
    return ((uint32_t)buf[2] << 24) | ((uint32_t)buf[3] << 16) | ((uint32_t)buf[4] << 8) | ((uint32_t)buf[5] << 0);
}

static void http_get_task(void *pvParameters)
{
    const struct addrinfo hints = {
        .ai_family = AF_INET,
        .ai_socktype = SOCK_STREAM,
    };
    struct addrinfo *res;
    struct in_addr *addr;
    int s, r;  
    uint32_t data[3];
    uint32_t temp_ = 0;
    uint32_t hum_  = 0;
    uint32_t MQ4_  = 0;
    while(1) {
        int err = getaddrinfo(WEB_SERVER, WEB_PORT, &hints, &res);

        if(err != 0 || res == NULL) {
            ESP_LOGE(TAG, "DNS lookup failed err=%d res=%p", err, res);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }

        /* Code to print the resolved IP.
        Note: inet_ntoa is non-reentrant, look at ipaddr_ntoa_r for "real" code */
        addr = &((struct sockaddr_in *)res->ai_addr)->sin_addr;
        ESP_LOGI(TAG, "DNS lookup succeeded. IP=%s", inet_ntoa(*addr));

        s = socket(res->ai_family, res->ai_socktype, 0);
        if(s < 0) {
            ESP_LOGE(TAG, "... Failed to allocate socket.");
            freeaddrinfo(res);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }
        ESP_LOGI(TAG, "... allocated socket");

        if(connect(s, res->ai_addr, res->ai_addrlen) != 0) {
            ESP_LOGE(TAG, "... socket connect failed errno=%d", errno);
            close(s);
            freeaddrinfo(res);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
            continue;
        }
        ESP_LOGI(TAG, "... connected");
        freeaddrinfo(res);
        // if(xQueueReceive(queue_data, data, portMAX_DELAY) == pdTRUE){
        //     uint32_t temp_ = data[0];
        //     uint32_t hum_  = data[1];
        //     uint32_t MQ4_  = data[2];
        // }
        // sprintf(REQUEST, "GET http://api.thingspeak.com/update.json?api_key=HHLZA8LPCME8DHX3&field1=%d&field2=%d&field3=%d\n\n", temp_, hum_, MQ4_);
        sprintf(REQUEST, "GET http://api.thingspeak.com/update.json?api_key=0FHA4BWA7O4MEUYU&field1=%ld&field2=%ld&field3=%ld\n\n", (long int)99, (long int)99, (long int)1034);

        if (write(s, REQUEST, strlen(REQUEST)) < 0) {
            ESP_LOGE(TAG, "... socket send failed");
            close(s);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
            continue;
        }
        ESP_LOGI(TAG, "... socket send success");

        struct timeval receiving_timeout;
        receiving_timeout.tv_sec = 5;
        receiving_timeout.tv_usec = 0;
        if (setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &receiving_timeout,
                sizeof(receiving_timeout)) < 0) {
            ESP_LOGE(TAG, "... failed to set socket receiving timeout");
            close(s);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
            continue;
        }
        ESP_LOGI(TAG, "... set socket receiving timeout success");

        /* Read HTTP response */
        do {
            bzero(recv_buf, sizeof(recv_buf));
            r = read(s, recv_buf, sizeof(recv_buf)-1);
            for(int i = 0; i < r; i++) {
                putchar(recv_buf[i]);
            }
        } while(r > 0);

        ESP_LOGI(TAG, "... done reading from socket. Last read return=%d errno=%d.", r, errno);
        close(s);
        for(int countdown = 10; countdown >= 0; countdown--) {
            ESP_LOGI(TAG, "%d... ", countdown);
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
        ESP_LOGI(TAG, "Starting again!");
    }
}
