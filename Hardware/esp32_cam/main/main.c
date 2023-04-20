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
#include "esp_http_client.h"
#include "esp_tls.h"


#define LED                     4
#define EX_UART_NUM             UART_NUM_0
#define BUF_SIZE                (1024)
#define HTTP_MAX_BUf            2048
#define RD_BUF_SIZE             (BUF_SIZE)
#define PART_BOUNDARY           "HCMUT"
#define CONFIG_XCLK_FREQ        20000000 
#define TXD_PIN                 1
#define RXD_PIN                 3
#define WEB_SERVER              "api.thingspeak.com"
#define WEB_PORT                "80"
#define TELEGRAM_ENABLE         1
#define TOKEN                   "5874407007:AAGksmiPfSyaePhCJC_ohMl1EL05_bfP9Pw"
#define CHAT_ID                 "1928092894"

char url_string[512] = "https://api.telegram.org/bot";
char REQUEST[512];
char recv_buf[512];

static const char *TAG = "ESP32-CAM";
static QueueHandle_t queue_data;
// static QueueHandle_t queue_leak_gas;
// static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;

static const char* _STREAM_CONTENT_TYPE = "image/jpeg";
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";
extern const char telegram_certificate_pem_start[] asm("_binary_telegram_certificate_pem_start");
extern const char telegram_certificate_pem_end[]   asm("_binary_telegram_certificate_pem_end");

/* Prototype */
httpd_handle_t setup_server(void);
// esp_err_t jpg_stream_httpd_handler(httpd_req_t *req);
static void uart_event_task(void *pvParameters);
static void Init_Hardware(void);
static esp_err_t init_camera(void);
static void read_uart(void *pvParameters);
static uint32_t GetMQ4Value(uint8_t *buf);
static void http_get_thingspeak_task(void *pvParameters);
static void camera_task(void *pvParameters);
// static void https_telegram_getMe_perform(void);
esp_http_client_handle_t telegram_send_message_start(void);

esp_http_client_handle_t telegram_picture_start(void);
static void telegram_task(void *pvParameters);
static void telegram_stop(esp_http_client_handle_t client);
static void telegram_send_message(esp_http_client_handle_t client, char *message);
static void telegram_send_picture(esp_http_client_handle_t client, char *chat_id);
esp_err_t camera_capture(void);
esp_err_t _http_event_handler(esp_http_client_event_t *evt);
esp_err_t take_photo(httpd_req_t *req);


httpd_uri_t uri_get = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = take_photo,    //jpg_stream_httpd_handler
    .user_ctx = NULL
};

void app_main()
{
    Init_Hardware();
    connect_wifi(); 
    init_camera();

	// xTaskCreate(&camera_task, "camera task", 1024*4, NULL, 3, NULL);
	xTaskCreate(&read_uart, "UART task", 1024*3, NULL, 3, NULL);        
    xTaskCreate(&http_get_thingspeak_task, "http_get_task", 1024*4, NULL, 3, NULL);
    xTaskCreate(&telegram_task, "http_telegram_task", 8192*4, NULL, 3, NULL);

}

static void telegram_task(void *pvParameters)
{   
    strcat(url_string, TOKEN);
    esp_http_client_handle_t client;
    
    client = telegram_picture_start();
    telegram_send_picture(client, CHAT_ID);

    // client = telegram_send_message_start();
    // telegram_send_message(client, "Hello VietNam. \nHow are you?");

    while(1){
        vTaskDelay(10/portTICK_PERIOD_MS);
    }
    
}

static void camera_task(void *pvParameters)
{
    if (wifi_connect_status){
        setup_server();
        ESP_LOGI(TAG, "Web Server is up and running");
    }else{
        ESP_LOGE(TAG, "Failed to connect with Wi-Fi, check your network credentials");
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
        if(uart_read_bytes(EX_UART_NUM, buff, RD_BUF_SIZE, 0) > 0){
            temp = buff[0];
            hum = buff[1];
            MQ4 = GetMQ4Value(buff);
            uint32_t data[3] = {temp, hum, MQ4};
            if(xQueueSend(queue_data, (void*)data, 0) != pdTRUE){
                printf("Failed to send data to queue!\n");
            }
            // printf("- Temp: %d \n- Hum: %d \n- MQ4: %ld \n\n", temp, hum, MQ4);
        }
        vTaskDelay(10/portTICK_PERIOD_MS);
    }
}

static void Init_Hardware(void){
    gpio_set_direction(LED, GPIO_MODE_OUTPUT);
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

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
        printf("Start HTTPD!\n");
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
        .jpeg_quality = 3, //0-63 lower number means higher quality
        .fb_count = 1,
        .grab_mode = CAMERA_GRAB_WHEN_EMPTY
    };
    //initialize the camera
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK){
        ESP_LOGE(TAG, "Camera Init Failed");
        return err;
    }
    return ESP_OK;
}

// esp_err_t jpg_stream_httpd_handler(httpd_req_t *req){
//     camera_fb_t * fb = NULL;
//     esp_err_t res = ESP_OK;
//     size_t _jpg_buf_len;
//     uint8_t * _jpg_buf;
//     char * part_buf[64];
//     static int64_t last_frame = 0;
//     if(!last_frame) {
//         last_frame = esp_timer_get_time();
//     }
//     res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
//     if(res != ESP_OK){
//         return res;
//     }

//     while(true){
//         fb = esp_camera_fb_get();
//         if (!fb) {
//             ESP_LOGE(TAG, "Camera capture failed");
//             res = ESP_FAIL;
//             break;
//         }
//         if(fb->format != PIXFORMAT_JPEG){
//             bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
//             if(!jpeg_converted){
//                 ESP_LOGE(TAG, "JPEG compression failed");
//                 esp_camera_fb_return(fb);
//                 res = ESP_FAIL;
//             }
//         } else {

//             _jpg_buf_len = fb->len;
//             _jpg_buf = fb->buf;
//         }

//         if(res == ESP_OK){
//             res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
//         }
//         if(res == ESP_OK){
//             size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);

//             res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
//         }
//         if(res == ESP_OK){
//             res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
//         }
//         if(fb->format != PIXFORMAT_JPEG){
//             free(_jpg_buf);
//         }
//         esp_camera_fb_return(fb);
//         if(res != ESP_OK){
//             break;
//         }
//         int64_t fr_end = esp_timer_get_time();
//         int64_t frame_time = fr_end - last_frame;
//         last_frame = fr_end;
//         frame_time /= 1000;
//     }
//     last_frame = 0;
//     return res;
// }

esp_err_t take_photo(httpd_req_t *req)
{
    camera_fb_t * fb = NULL;
    esp_err_t res = ESP_OK;

    fb = esp_camera_fb_get();
    if (!fb)
    {
      ESP_LOGE(TAG, "Camera capture failed");
      httpd_resp_send_500(req);
      return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Camera capture successfully");
    res = httpd_resp_set_type(req, "image/jpeg");
    if (res == ESP_OK)
    {
      res = httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
    }
    if (res == ESP_OK)
    {
      res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
    }
    esp_camera_fb_return(fb);
    return res;
}


static uint32_t GetMQ4Value(uint8_t *buf)
{
    return ((uint32_t)buf[2] << 24) | ((uint32_t)buf[3] << 16) | ((uint32_t)buf[4] << 8) | ((uint32_t)buf[5] << 0);
}

static void http_get_thingspeak_task(void *pvParameters)
{
    const struct addrinfo hints = {
        .ai_family = AF_INET,
        .ai_socktype = SOCK_STREAM,
    };
    struct addrinfo *res;
    struct in_addr *addr;
    int s, r;
    uint32_t data[3];
    uint32_t temp_ = 0, hum_ = 0, MQ4_ = 0;
    while (1)
    {
        int err = getaddrinfo(WEB_SERVER, WEB_PORT, &hints, &res);
        if (err != 0 || res == NULL){
            ESP_LOGE(TAG, "DNS lookup failed err=%d res=%p", err, res);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }
        addr = &((struct sockaddr_in *)res->ai_addr)->sin_addr;
        ESP_LOGI(TAG, "DNS lookup succeeded. IP=%s", inet_ntoa(*addr));

        s = socket(res->ai_family, res->ai_socktype, 0);
        if (s < 0){
            ESP_LOGE(TAG, "... Failed to allocate socket.");
            freeaddrinfo(res);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }
        ESP_LOGI(TAG, "... allocated socket");

        if (connect(s, res->ai_addr, res->ai_addrlen) != 0){
            ESP_LOGE(TAG, "... socket connect failed errno=%d", errno);
            close(s);
            freeaddrinfo(res);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
            continue;
        }
        ESP_LOGI(TAG, "... connected");
        freeaddrinfo(res);

        if (xQueueReceive(queue_data, data, portMAX_DELAY) == pdTRUE){
            temp_ = data[0];
            hum_  = data[1];
            MQ4_  = data[2];
            printf("\nValue send to Thingspeak:\n- Temp: %ld \n- Hum: %ld \n- MQ4: %ld \n\n", temp_, hum_, MQ4_);
            sprintf(REQUEST, "GET http://api.thingspeak.com/update.json?api_key=0FHA4BWA7O4MEUYU&field1=%ld&field2=%ld&field3=%ld\n\n", (long int)temp_, (long int)hum_, (long int)MQ4_);
            if (write(s, REQUEST, strlen(REQUEST)) < 0){
                ESP_LOGE(TAG, "... socket send failed");
                close(s);
                vTaskDelay(10 / portTICK_PERIOD_MS); //4000
                continue;
            }           
            ESP_LOGI(TAG, "... socket send success"); 
            struct timeval receiving_timeout;
            receiving_timeout.tv_sec = 0; //5
            receiving_timeout.tv_usec = 0;
            if (setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &receiving_timeout,
                           sizeof(receiving_timeout)) < 0)
            {
                ESP_LOGE(TAG, "... failed to set socket receiving timeout");
                close(s);
                vTaskDelay(100 / portTICK_PERIOD_MS); //4000
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
            
        }
        /*delay 15s for thingspeak receive data*/
        for(int countdown = 15; countdown > 0; countdown--) {
            ESP_LOGI(TAG, "%d... ", countdown);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }
}

esp_http_client_handle_t telegram_send_message_start(void) 
{
	char url[512] = {0};
    char output_buffer[HTTP_MAX_BUf] = {0};   // Buffer to store response of http request
    esp_http_client_config_t config = {
        .url = "https://api.telegram.org",
        .transport_type = HTTP_TRANSPORT_OVER_SSL,
        .event_handler = _http_event_handler,
        .cert_pem = telegram_certificate_pem_start,
		.user_data = output_buffer,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    strcat(url, url_string);
    strcat(url, "/sendMessage");
    esp_http_client_set_url(client, url); //url = https://api.telegram.org/bot{TOKEN}/sendMessage

    printf("Ready to send message to telegram\n");
    return client;
}





esp_err_t _http_event_handler(esp_http_client_event_t *evt) 
{
    static char *output_buffer;  // Buffer to store response of http request from event handler
    static int output_len;       // Stores number of bytes read
    switch(evt->event_id) {
        case HTTP_EVENT_ERROR:
            printf("HTTP_EVENT_ERROR\n");
            break;
        case HTTP_EVENT_ON_CONNECTED:
            printf("HTTP_EVENT_ON_CONNECTED\n");
            break;
        case HTTP_EVENT_HEADER_SENT:
            printf("HTTP_EVENT_HEADER_SENT\n");
            break;
        case HTTP_EVENT_ON_HEADER:
            printf("HTTP_EVENT_ON_HEADER, key=%s, value=%s\n", evt->header_key, evt->header_value);
            break;
        case HTTP_EVENT_ON_DATA:
            printf("HTTP_EVENT_ON_DATA, len=%d\n", evt->data_len);
            /*
             *  Check for chunked encoding is added as the URL for chunked encoding used in this example returns binary data.
             *  However, event handler can also be used in case chunked encoding is used.
             */
            if (!esp_http_client_is_chunked_response(evt->client)) {
                // If user_data buffer is configured, copy the response into the buffer
                if (evt->user_data) {
                    memcpy(evt->user_data + output_len, evt->data, evt->data_len);
                } else {
                    if (output_buffer == NULL) {
                        output_buffer = (char *) malloc(esp_http_client_get_content_length(evt->client));
                        output_len = 0;
                        if (output_buffer == NULL) {
                            printf("Failed to allocate memory for output buffer\n");
                            return ESP_FAIL;
                        }
                    }
                    memcpy(output_buffer + output_len, evt->data, evt->data_len);
                }
                output_len += evt->data_len;
            }

            break;
        case HTTP_EVENT_ON_FINISH:
            printf("HTTP_EVENT_ON_FINISH\n");
            if (output_buffer != NULL) {
                // Response is accumulated in output_buffer. Uncomment the below line to print the accumulated response
                // ESP_LOG_BUFFER_HEX(TAG, output_buffer, output_len);
                free(output_buffer);
                output_buffer = NULL;
            }
            output_len = 0;
            break;
        case HTTP_EVENT_DISCONNECTED:
            printf("HTTP_EVENT_DISCONNECTED\n");
            int mbedtls_err = 0;
            esp_err_t err = esp_tls_get_and_clear_last_error(evt->data, &mbedtls_err, NULL);
            if (err != 0) {
                if (output_buffer != NULL) {
                    free(output_buffer);
                    output_buffer = NULL;
                }
                output_len = 0;
                printf("Last esp error code: 0x%x\n", err);
                printf("Last mbedtls failure: 0x%x\n", mbedtls_err);
            }
            break;
        case HTTP_EVENT_REDIRECT:
        default:
            break;
    }
    return ESP_OK;
}

static void telegram_send_message(esp_http_client_handle_t client, char *message)
{
    char post_data[512] = "";
	sprintf(post_data, "{\"chat_id\":%s,\"text\":\"%s\"}", CHAT_ID, message);
    esp_http_client_set_method(client, HTTP_METHOD_POST);
    esp_http_client_set_header(client, "Content-Type", "application/json");
    // esp_http_client_set_header(client, "Content-Length", strlen(post_data));
    esp_http_client_set_post_field(client, post_data, strlen(post_data));
    esp_err_t err = esp_http_client_perform(client);
}




esp_http_client_handle_t telegram_picture_start(void)
{
    char url[512] = {0};
    char output_buffer[HTTP_MAX_BUf] = {0};   // Buffer to store response of http request
    esp_http_client_config_t config = {
        .url = "https://api.telegram.org",
        .transport_type = HTTP_TRANSPORT_OVER_SSL,
        .event_handler = _http_event_handler,
        .cert_pem = telegram_certificate_pem_start,
		.user_data = output_buffer,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    strcat(url, url_string);
    strcat(url,"/sendPhoto");
    esp_http_client_set_url(client, url);

    printf("Ready to send picture to telegram\n");
    return client;
}
static void telegram_send_picture(esp_http_client_handle_t client, char *chat_id)
{
    camera_fb_t *fb = NULL;
    esp_err_t res = ESP_OK;
    char part_buf[64];
    //Take a picture
    fb = esp_camera_fb_get();
    if (!fb) {
        ESP_LOGE(TAG, "Camera capture failed");
        return;
    }
    esp_http_client_set_method(client, HTTP_METHOD_POST);
    esp_http_client_set_header(client, "Content-Type", "multipart/form-data; boundary=RandomNerdTutorials");
    char *head = "--RandomNerdTutorials\r\nContent-Disposition: form-data; name=\"chat_id\"\r\n\r\n";
    char *head_ = "\r\n--RandomNerdTutorials\r\nContent-Disposition: form-data; name=\"photo\"; filename=\"esp32-cam.jpg\"\r\nContent-Type: image/jpeg\r\n\r\n";
    char *tail = "\r\n--RandomNerdTutorials--\r\n";
    size_t head_len = strlen(head);
    size_t head__len = strlen(head_);
    size_t tail_len = strlen(tail);
    size_t total_len = head_len + tail_len + head__len + fb->len + strlen(chat_id);
    char content_length[512];
    snprintf(content_length, sizeof(content_length), "%d", total_len);
    esp_http_client_set_header(client, "Content-Length", content_length);
    esp_err_t err = esp_http_client_open(client, total_len);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open HTTP connection: %s", esp_err_to_name(err));
        esp_camera_fb_return(fb);
        return;
    }
    esp_http_client_write(client, head, head_len);
    esp_http_client_write(client, chat_id, strlen(chat_id));
    esp_http_client_write(client, head_, head__len);
    
    uint8_t *fbBuf = fb->buf;
    size_t fbLen = fb->len;
    for (size_t n = 0; n < fbLen; n = n + 1024) {
        if (n + 1024 < fbLen) {
            esp_http_client_write(client, (const char *)fbBuf, 1024);
            fbBuf += 1024;
        } else if (fbLen % 1024 > 0) {
            size_t remainder = fbLen % 1024;
            esp_http_client_write(client, (const char *)fbBuf, remainder);
        }
    }
    esp_http_client_write(client, tail, tail_len);






    // char text1[1024] = "POST /bot5874407007:AAGksmiPfSyaePhCJC_ohMl1EL05_bfP9Pw/sendPhoto HTTP/1.1\r\n";
    // char text2[1024] = "Host: api.telegram.org\r\n";
    // char text3[1024] = "Content-Type: multipart/form-data; boundary=RandomNerdTutorials\r\n\r\n";

    // char head[1024] = "--RandomNerdTutorials\r\nContent-Disposition: form-data; name=\"chat_id\"\r\n\r\n1928092894\r\n--RandomNerdTutorials\r\nContent-Disposition: form-data; name=\"photo\"; filename=\"esp32-cam.jpg\"\r\nContent-Type: image/jpeg\r\n\r\n";
    // char tail[1024] = "\r\n--RandomNerdTutorials--\r\n";
    // uint32_t totalLen = fb->len + strlen(head) + strlen(tail);
    // // esp_http_client_set_method(client, HTTP_METHOD_POST);
    // // Set the Content-Type header
    // char content_length[512];
    // snprintf(content_length, sizeof(content_length), "%ld", totalLen);
    // printf("%s\n", content_length);

    // esp_http_client_set_post_field(client, text1, strlen(text1));
    // esp_http_client_set_post_field(client, text2, strlen(text2));
    // esp_http_client_set_header(client, "Content-Length", content_length);
    // esp_http_client_set_post_field(client, text3, strlen(text3));

    // // esp_http_client_set_header(client, "Content-Type", "multipart/form-data; boundary=--RandomNerdTutorials");
    // esp_http_client_set_post_field(client, head, strlen(head));
    // // Write the image data to the post data buffer
    // esp_http_client_write(client, (const char *)fb->buf, (int)fb->len);
    // // Write the closing boundary string
    // esp_http_client_set_post_field(client, tail, strlen(tail));

    // Perform the HTTP POST request
    res = esp_http_client_perform(client);

    // Free the camera frame buffer
    esp_camera_fb_return(fb);

    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send HTTP request: %s", esp_err_to_name(res));
        return;
    }
}


// static void telegram_send_picture(esp_http_client_handle_t client, char *chat_id)
// {
//     camera_fb_t * fb = NULL;
//     esp_err_t res = ESP_OK;
//     char part_buf[64];
//     fb = esp_camera_fb_get();
//     if (!fb) {
//         ESP_LOGE(TAG, "Camera capture failed");
//         return;
//     }

//     //Set boundary
//     snprintf(part_buf, sizeof(part_buf), PART_BOUNDARY);
//     esp_http_client_set_method(client, HTTP_METHOD_POST);
//     esp_http_client_set_header(client, "Content-Type", "multipart/form-data; boundary=123456789000000000000987654321");
//     // esp_http_client_set_header(client, "Content-Length", itoa(fb->len, part_buf, 10));

//     //Set the chat_id and content length
//     int content_length = fb->len + strlen(chat_id) + strlen("--\r\n");
//     esp_http_client_set_header(client, "Content-Length", itoa(content_length, part_buf, 10));
//     char post_data[512] = "";
//     snprintf(post_data, sizeof(post_data), "Content-Disposition: form-data; name=\"chat_id\"; \r\n\r\n%s\r\n", chat_id);
//     esp_http_client_set_post_field(client, post_data, strlen(post_data));
//     // Set the picture data
//     // snprintf(post_data, sizeof(post_data), "Content-Disposition: form-data; name=\"photo\"; filename=\"photo.jpg\"\r\nContent-Type: image/jpeg\r\n\r\n");
//     sprintf(post_data, "---------------------------\r\nContent-Disposition: form-data; name=\"chat_id\"\r\n\r\n%s\r\n---------------------------\r\nContent-Disposition: form-data; name=\"photo\"; filename=\"photo.jpg\"\r\nContent-Type: image/jpeg\r\n\r\n", chat_id);
//     esp_http_client_write(client, post_data, strlen(post_data));
//     esp_http_client_write(client, (const char *)fb->buf, (int)fb->len);
//     esp_http_client_write(client, "\r\n--\r\n", strlen("\r\n--\r\n"));

//     res = esp_http_client_perform(client);

//     esp_camera_fb_return(fb);

//     if (res != ESP_OK) {
//         ESP_LOGE(TAG, "Failed to send HTTP request: %s", esp_err_to_name(res));
//         return;
//     }


//     // //Send the picture
//     // res = esp_http_client_set_post_field(client, (const char *)fb->buf, (int)fb->len);
//     // if (res != ESP_OK) {
//     //     ESP_LOGE(TAG, "Failed to set HTTP post field: %s", esp_err_to_name(res));
//     //     return;
//     // }
//     // // snprintf((char *)part_buf, sizeof(part_buf), "multipart/form-data; boundary=%s", PART_BOUNDARY);

//     // //Perform the HTTP request
//     // res = esp_http_client_perform(client);
//     // // printf("DEBUG--\n");
//     // if (res != ESP_OK) {
//     //     ESP_LOGE(TAG, "Failed to send HTTP request: %s", esp_err_to_name(res));
//     //     return;
//     // }
//     // // printf("DEBUG--\n");
//     // esp_camera_fb_return(fb); 
// }


static void telegram_stop(esp_http_client_handle_t client)
{
    esp_http_client_close(client);
    esp_http_client_cleanup(client);
} 
esp_err_t camera_capture(void)
{
    camera_fb_t *fb = esp_camera_fb_get();
    if(!fb){
        ESP_LOGE(TAG, "Camera Captured Failed");
        return ESP_FAIL;
    }
    esp_camera_fb_return(fb);
    return ESP_OK;
}
