#include <string.h>
#include <sys/param.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "driver/i2c.h"
#include "i2cdev.h"
#include "ds3231.h"


#define EXAMPLE_ESP_WIFI_SSID      "ICES"
#define EXAMPLE_ESP_WIFI_PASS      "12345678"
#define EXAMPLE_ESP_MAXIMUM_RETRY  5
#define HOST_IP_ADDR               "192.168.97.76"
#define CONFIG_EXAMPLE_IPV4
#define PORT                        8080
#define MAX_MESSAGES                3         
#define MAX_MSG_LENGTH              128      


static char stored_messages[MAX_MESSAGES][MAX_MSG_LENGTH];
static int message_index = 0;

typedef struct {
       i2c_config_t pinconfig1;
    i2c_dev_t dev1;
    int store_value1;
    int do_i_flash;
}task_1_params_t;



struct tm time_read;

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static const char *TAG = "wifi station";

static int s_retry_num = 0;

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:%s",
                 ip4addr_ntoa(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}
struct tm time_read;


void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    tcpip_adapter_init();

    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS
        },
    };

    /* Setting a password implies station will connect to all security modes including WEP/WPA.
        * However these modes are deprecated and not advisable to be used. Incase your Access point
        * doesn't support WPA2, these mode can be enabled by commenting below line */

    if (strlen((char *)wifi_config.sta.password)) {
        wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }

    ESP_ERROR_CHECK(esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler));
    ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler));
    vEventGroupDelete(s_wifi_event_group);
}


static void tcp_client_task(){ 
    char rx_buffer[MAX_MSG_LENGTH];
    char addr_str[128];
    int addr_family;
    int ip_protocol;

#ifdef CONFIG_EXAMPLE_IPV4
    struct sockaddr_in destAddr;
    destAddr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
    destAddr.sin_family = AF_INET;
    destAddr.sin_port = htons(PORT);
    addr_family = AF_INET;
    ip_protocol = IPPROTO_IP;
    inet_ntoa_r(destAddr.sin_addr, addr_str, sizeof(addr_str) - 1);
#else // IPV6
    struct sockaddr_in6 destAddr;
    inet6_aton(HOST_IP_ADDR, &destAddr.sin6_addr);
    destAddr.sin6_family = AF_INET6;
    destAddr.sin6_port = htons(PORT);
    destAddr.sin6_scope_id = tcpip_adapter_get_netif_index(TCPIP_ADAPTER_IF_STA);
    addr_family = AF_INET6;
    ip_protocol = IPPROTO_IPV6;
    inet6_ntoa_r(destAddr.sin6_addr, addr_str, sizeof(addr_str) - 1);
#endif

    int sock = socket(addr_family, SOCK_STREAM, ip_protocol);
    if (sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        return;
    }
    ESP_LOGI(TAG, "Socket created");

    int err = connect(sock, (struct sockaddr *)&destAddr, sizeof(destAddr));
    if (err != 0) {
        ESP_LOGE(TAG, "Socket unable to connect: errno %d", errno);
        close(sock);
        return;
    }
    ESP_LOGI(TAG, "Successfully connected");

    while (1) {
        int len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
        
        if (len < 0) { // Error occurred during receiving
            ESP_LOGE(TAG, "recv failed: errno %d", errno);
            break;
        } else if (len > 0) { // Data received
            rx_buffer[len] = 0; // Null-terminate the received data
            ESP_LOGI(TAG, "Received %d bytes from %s:", len, addr_str);
            ESP_LOGI(TAG, "%s", rx_buffer);

            // Store the received message
            snprintf(stored_messages[message_index], MAX_MSG_LENGTH, "%s", rx_buffer);
            message_index = (message_index + 1) % MAX_MESSAGES; // Cycle index to avoid overflow

            // Print stored messages
            ESP_LOGI(TAG, "Stored messages:");
            for (int i = 0; i < MAX_MESSAGES; i++) {
                if (stored_messages[i][0] != '\0') { // Print only non-empty messages
                    ESP_LOGI(TAG, "Message %d: %s", i, stored_messages[i]);
                }
            }

            // Check if the maximum number of messages has been reached
            if (message_index == 0) {
                ESP_LOGI(TAG, "Received 10 messages. Closing socket...");
                ESP_LOGI(TAG, "Shutting down socket...");
                shutdown(sock, 0);
                close(sock);
                ESP_LOGI(TAG, "Shut Down.");

               
                break;
            }
            ESP_LOGI(TAG, "Here.");

        }

        vTaskDelay(2000 / portTICK_PERIOD_MS); // Delay to avoid tight loop if no data is received
    }


                ESP_LOGI(TAG, "Here.2");



}

static void perodic_check(void *pvParameters) {
    char rx_buffer[MAX_MSG_LENGTH];
    char addr_str[128];
    int addr_family;
    int ip_protocol;
    char send_buffer[MAX_MSG_LENGTH];


    // Determine IP address and protocol based on IPv4 or IPv6 configuration
#ifdef CONFIG_EXAMPLE_IPV4
    struct sockaddr_in destAddr;
    destAddr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
    destAddr.sin_family = AF_INET;
    destAddr.sin_port = htons(PORT);
    addr_family = AF_INET;
    ip_protocol = IPPROTO_IP;
    inet_ntoa_r(destAddr.sin_addr, addr_str, sizeof(addr_str) - 1);
#else // IPV6
    struct sockaddr_in6 destAddr;
    inet6_aton(HOST_IP_ADDR, &destAddr.sin6_addr);
    destAddr.sin6_family = AF_INET6;
    destAddr.sin6_port = htons(PORT);
    destAddr.sin6_scope_id = tcpip_adapter_get_netif_index(TCPIP_ADAPTER_IF_STA);
    addr_family = AF_INET6;
    ip_protocol = IPPROTO_IPV6;
    inet6_ntoa_r(destAddr.sin6_addr, addr_str, sizeof(addr_str) - 1);
#endif

    while (1) {
        // Create a socket each time we check for messages
        int sock = socket(addr_family, SOCK_STREAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            vTaskDelay(10000 / portTICK_PERIOD_MS); // Delay before retrying
            continue; // Try again after delay
        }
        ESP_LOGI(TAG, "Socket created");

        // Try to connect to the server
        int err = connect(sock, (struct sockaddr *)&destAddr, sizeof(destAddr));
        if (err != 0) {
            ESP_LOGE(TAG, "Socket unable to connect: errno %d", errno);
            close(sock);
            vTaskDelay(10000 / portTICK_PERIOD_MS); // Delay before retrying
            continue; // Try again after delay
        }
        ESP_LOGI(TAG, "Successfully connected to %s", addr_str);

        // Try to receive a message from the server
        int len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
        
        if (len < 0) {
            ESP_LOGE(TAG, "recv failed: errno %d", errno);
        } else if (len > 0) {
            rx_buffer[len] = 0; // Null-terminate the received data
            ESP_LOGI(TAG, "Received %d bytes from %s:", len, addr_str);
            ESP_LOGI(TAG, "%s", rx_buffer);  // Print the received message
        } else {
            ESP_LOGI(TAG, "No data received from server");
        }
            ESP_LOGI(TAG, "time to send");

        snprintf(send_buffer, sizeof(send_buffer), 
                 "Hello\nYear: %d\nMonth: %d\nDay: %d\nHour: %d\nMinute: %d\nSecond: %d\n",
                 time_read.tm_year + 1900,  // Year
                 time_read.tm_mon + 1,      // Month (tm_mon is 0-11, so add 1)
                 time_read.tm_mday,         // Day
                 time_read.tm_hour,         // Hour
                 time_read.tm_min,          // Minute
                 time_read.tm_sec);         // Second


        
        // Send the message to the server
        len = send(sock, send_buffer, strlen(send_buffer), 0);
        if (len < 0) {
            ESP_LOGE(TAG, "Send failed: errno %d", errno);
        } else {
            ESP_LOGI(TAG, "Sent %d bytes to %s", len, addr_str);
        }


        // Close the socket after receiving or if no data
        shutdown(sock, 0);
        close(sock);
        ESP_LOGI(TAG, "Socket closed");

        // Delay 10 seconds before checking again
        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
}


void Read_frm_clock(void *pvParameters) {

    task_1_params_t *params = (task_1_params_t *)pvParameters;
    i2c_dev_t dev=params->dev1;
    int interval=params->store_value1;

esp_err_t checker;


    while (1) {

                // Perform the I2C read operation
        checker = ds3231_get_time(&dev, &time_read);
        
        if (checker == ESP_OK) {
           
          

            // Print the time
            printf("Year: %d\n", time_read.tm_year + 1900);
            printf("Month: %d\n", time_read.tm_mon + 1);
            printf("Day: %d\n", time_read.tm_mday);
            printf("Hour: %d\n", time_read.tm_hour);
            printf("Minute: %d\n", time_read.tm_min);
            printf("Second: %d\n", time_read.tm_sec);
        } else {
            printf("Failed to read time from DS3231\n");
        }

        // Delay for the specified interval (in ms converted to ticks)
        vTaskDelay(interval*1000 / portTICK_PERIOD_MS);
    }
}

void Check_and_Buzz(void *pvParameters){
    task_1_params_t *params = (task_1_params_t *)pvParameters;

    int checker=params->do_i_flash;
    i2c_dev_t dev=params->dev1;
    int interval=params->store_value1;
while(1){
    if(checker==1){

ds3231_set_squarewave_freq(&dev,DS3231_SQWAVE_1HZ);
        ESP_LOGI(TAG, "Buzz");

ds3231_enable_squarewave(&dev);

        vTaskDelay((interval*1000) / portTICK_PERIOD_MS);

    }

    else{

ds3231_disable_squarewave(&dev);
        ESP_LOGI(TAG, "No Buzz");


        vTaskDelay(interval*1000 / portTICK_PERIOD_MS);
    }
            vTaskDelay(30*1000 / portTICK_PERIOD_MS);

}
}


void app_main()
{




    ESP_ERROR_CHECK(nvs_flash_init());
    TickType_t Start,End,Ex;
    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    Start=xTaskGetTickCount();

    wifi_init_sta();

    End=xTaskGetTickCount();

    Ex=End-Start;

    printf("Execution time: %d ticks\n", Ex);

            // Convert ticks to milliseconds
    int execution_time_ms = Ex * portTICK_PERIOD_MS;
    printf("Execution time: %d milliseconds\n", execution_time_ms);


   tcp_client_task();

    ESP_LOGI(TAG, "Out of Func.");

      int store_value = 30;
      int i=0;
      bool temp;
    
        if (strstr(stored_messages[i], "a") != NULL) {
            store_value = 15;  // Assign value for Option A
        } else if (strstr(stored_messages[i], "b") != NULL) {
            store_value = 30;  // Assign value for Option B
        } else if (strstr(stored_messages[i], "c") != NULL) {
            store_value = 45;  // Assign value for Option C
        }

        ESP_LOGI(TAG, "the Delay is %d seconds",store_value);

        i++;
        printf("%s \n",stored_messages[i]);
        if (strstr(stored_messages[i], "yes") != NULL) {
            temp = 1;  // Assign value for Option A
        } else if (strstr(stored_messages[i], "no") != NULL) {
            temp = 0;  // Assign value for Option B
        }
        else{
            temp=0;
        }


        printf("Can the serial Please fucking work \n");

        
    i2cdev_init();
    i2c_config_t pinconfig;


    pinconfig.mode=I2C_MODE_MASTER;

    //disabling pullups
    pinconfig.sda_pullup_en=GPIO_PULLUP_DISABLE;
    pinconfig.scl_pullup_en=GPIO_PULLUP_DISABLE;


    //Setting SCL as IO 0
    pinconfig.scl_io_num=GPIO_NUM_0;

    //Setting SDA as IO2
    pinconfig.sda_io_num=GPIO_NUM_2;

    //wait for 1000 tics
    pinconfig.clk_stretch_tick = 1000;


    i2c_dev_t dev;
    memset(&dev, 0, sizeof(i2c_dev_t));
    dev.addr=DS3231_ADDR;
    dev.cfg=pinconfig;
    dev.port=I2C_NUM_0;

esp_err_t installer = ds3231_init_desc(&dev,0,GPIO_NUM_2,GPIO_NUM_0);
    if (installer==ESP_OK){

        printf("Set up Properly \n");
    }
esp_err_t read;
    read = ds3231_get_time(&dev, &time_read);
        
        if (read == ESP_OK) {
           
          

            // Print the time
            printf("Year: %d\n", time_read.tm_year + 1900);
            printf("Month: %d\n", time_read.tm_mon + 1);
            printf("Day: %d\n", time_read.tm_mday);
            printf("Hour: %d\n", time_read.tm_hour);
            printf("Minute: %d\n", time_read.tm_min);
            printf("Second: %d\n", time_read.tm_sec);
        } else {
            printf("Failed to read time from DS3231\n");
        }


    task_1_params_t params;
    memset(&params, 0, sizeof(task_1_params_t));

    params.dev1=dev;
    params.pinconfig1=pinconfig;
    params.store_value1=store_value;
    if(temp){
        
        params.do_i_flash=1;
        
        }

        else{

            params.do_i_flash=0;
        }

    xTaskCreate(Read_frm_clock, "Read frm clock", 2048, &params, 5, NULL);
    xTaskCreate(Check_and_Buzz,"Check and buzz",2048, &params, 6, NULL);
    xTaskCreate(perodic_check,"Periodic Check",2048, &params, 4, NULL);
}
