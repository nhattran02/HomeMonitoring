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
#define HTTP_MAX_BUF            2048
#define RD_BUF_SIZE             (BUF_SIZE)
#define PART_BOUNDARY           "HCMUT"
#define CONFIG_XCLK_FREQ        20000000 
#define TXD_PIN                 1
#define RXD_PIN                 3
#define GAS_THRESHOLD           900
#define WEB_SERVER              "api.thingspeak.com"
#define WEB_PORT                "80"
#define TELEGRAM_TOKEN          "5874407007:AAGksmiPfSyaePhCJC_ohMl1EL05_bfP9Pw"
#define TELEGRAM_CHAT_ID        "1928092894"
#define PIR_MOTION_PIN          2
char url_string[512] = "https://api.telegram.org/bot";
char REQUEST[512];
char recv_buf[512];

static const char *TAG = "ESP32-CAM";
static QueueHandle_t queue_data;
static char* _PICTURE_CONTENT_TYPE = "multipart/form-data; boundary="PART_BOUNDARY;
static char* _PICTURE_TAIL = "\r\n--"PART_BOUNDARY"--\r\n";
static char* _PICTURE_HEADER_1 = "--"PART_BOUNDARY"\r\nContent-Disposition: form-data; name=\"chat_id\"\r\n\r\n";
static char* _PICTURE_HEADER_2 = "\r\n--"PART_BOUNDARY"\r\nContent-Disposition: form-data; name=\"photo\"; filename=\"esp32-cam.jpg\"\r\nContent-Type: image/jpeg\r\n\r\n";
bool flag = false;

extern const char telegram_certificate_pem_start[] asm("_binary_telegram_certificate_pem_start");
extern const char telegram_certificate_pem_end[]   asm("_binary_telegram_certificate_pem_end");

/* Prototype */
static void Init_Hardware(void);
static void Init_PIR(void);
static esp_err_t Init_camera(void);
static void read_uart(void *pvParameters);
static uint32_t GetMQ4Value(uint8_t *buf);
static void http_get_thingspeak_task(void *pvParameters);
esp_http_client_handle_t telegram_message_start(void);
static void telegram_send_message(esp_http_client_handle_t client, char *message);
esp_http_client_handle_t telegram_picture_start(void);
static void telegram_send_picture(esp_http_client_handle_t client, char *chat_id);
static void telegram_pic_task(void *pvParameters);
static void telegram_stop(esp_http_client_handle_t client);
esp_err_t _http_event_handler(esp_http_client_event_t *evt);
static void telegram_gas_task(void *pvParameters);


static void IRAM_ATTR gpio_interrupt_handler(void *pvParameters)
{
    flag = true;
}

void app_main()
{
    Init_Hardware();
    connect_wifi();
    Init_camera();
    Init_PIR();   
	xTaskCreate(&read_uart, "UART task", 1024*3, NULL, 3, NULL);
    xTaskCreate(&http_get_thingspeak_task, "thingspeak task", 1024*4, NULL, 3, NULL);
    xTaskCreate(&telegram_pic_task, "telegram camera task", 8192*4, NULL, 3, NULL);
    xTaskCreate(&telegram_gas_task, "telegram gas task", 8192*2, NULL, 3, NULL);

    while(1){
        vTaskDelay(10/portTICK_PERIOD_MS);
    }
}

static void telegram_pic_task(void *pvParameters)
{   
    strcat(url_string, TELEGRAM_TOKEN);
    esp_http_client_handle_t client1, client2;
    while(1){
        if(flag){
            client2 = telegram_picture_start();
            telegram_send_picture(client2, TELEGRAM_CHAT_ID);
            telegram_stop(client2);
            client1 = telegram_message_start();
            telegram_send_message(client1, "Motion Detected!!!");
            telegram_stop(client1);
            flag = false;            
        }
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

    //Create queue
    queue_data = xQueueCreate(10, sizeof(uint32_t)*3);

}

static void Init_PIR(void)
{
    //Enable PIR_MOTION interrupt
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << PIR_MOTION_PIN),
        .intr_type = GPIO_INTR_POSEDGE,
        .pull_down_en = 1,
        .pull_up_en = 0
    };
    gpio_config(&io_conf);
    // gpio_install_isr_service(0);
    //  gpio_install_isr_service()
    gpio_isr_handler_add(PIR_MOTION_PIN, gpio_interrupt_handler, NULL);
    printf("\nInit PIR successfully\n");
}

static esp_err_t Init_camera(void)
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
        .pixel_format = PIXFORMAT_JPEG, //PIXFORMAT_RGB565 PIXFORMAT_RGB555 PIXFORMAT_YUV422 PIXFORMAT_GRAYSCALE
        .frame_size = FRAMESIZE_QSXGA,  //FRAMESIZE_VGA
        .jpeg_quality = 12, //0-63 lower number means higher quality
        .fb_count = 1,
        .grab_mode = CAMERA_GRAB_LATEST
    };
    //initialize the camera
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK){
        ESP_LOGE(TAG, "Camera Init Failed");
        return err;
    }
    ESP_LOGI(TAG, "Init successfully");
    return ESP_OK;
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

esp_http_client_handle_t telegram_message_start(void) 
{
	char url[512] = {0};
    char output_buffer[HTTP_MAX_BUF] = {0};   // Buffer to store response of http request
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
            if (!esp_http_client_is_chunked_response(evt->client)) {
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
	sprintf(post_data, "{\"chat_id\":%s,\"text\":\"%s\"}", TELEGRAM_CHAT_ID, message);
    esp_http_client_set_method(client, HTTP_METHOD_POST);
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_post_field(client, post_data, strlen(post_data));
    esp_http_client_perform(client);
}

esp_http_client_handle_t telegram_picture_start(void)
{
    char url[512] = {0};
    char output_buffer[HTTP_MAX_BUF] = {0};   // Buffer to store response of http request
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
    return client;
}
static void telegram_send_picture(esp_http_client_handle_t client, char *chat_id)
{
    camera_fb_t *fb = NULL;
    esp_err_t ret = ESP_OK;

    //Take a picture
    gpio_set_level(LED, 1);
    fb = esp_camera_fb_get();
    if (!fb) {
        ESP_LOGE(TAG, "Camera capture failed");
        esp_camera_fb_return(fb);
        return;
    }
    esp_http_client_set_method(client, HTTP_METHOD_POST);
    esp_http_client_set_header(client, "Content-Type", _PICTURE_CONTENT_TYPE);

    char *head1  = _PICTURE_HEADER_1;
    char *head2 = _PICTURE_HEADER_2;
    char *tail  = _PICTURE_TAIL;
    size_t head1_len = strlen(head1);
    size_t head2_len = strlen(head2);
    size_t tail_len = strlen(tail);
    size_t total_len = head1_len + tail_len + head2_len + fb->len + strlen(chat_id);
    char content_length[6];
    snprintf(content_length, sizeof(content_length), "%d", total_len);
    esp_http_client_set_header(client, "Content-Length", content_length);

    ret = esp_http_client_open(client, total_len);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open HTTP connection: %s", esp_err_to_name(ret));
        esp_camera_fb_return(fb);
        return;
    }
    gpio_set_level(LED, 0);

    esp_http_client_write(client, head1, head1_len);
    esp_http_client_write(client, chat_id, strlen(chat_id));
    esp_http_client_write(client, head2, head2_len);

    esp_http_client_write(client, (const char *)fb->buf, fb->len);
    esp_http_client_write(client, tail, tail_len);

    // Perform the HTTP POST 
    ESP_ERROR_CHECK(esp_http_client_perform(client));

    // Free the camera frame buffer
    esp_camera_fb_return(fb);
}


static void telegram_stop(esp_http_client_handle_t client)
{
    esp_http_client_close(client);
    esp_http_client_cleanup(client);
} 

static void telegram_gas_task(void *pvParameters)
{
    uint32_t data[3];
    esp_http_client_handle_t client;
    while(1){
        if (xQueueReceive(queue_data, data, portMAX_DELAY) == pdTRUE){
            if(data[2] > GAS_THRESHOLD){
                client = telegram_message_start();
                telegram_send_message(client, "Gas Leak Detected!!!");
                telegram_stop(client);
            }
        }   
        vTaskDelay(10/portTICK_PERIOD_MS);
    }
}

