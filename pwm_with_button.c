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
#include "esp_spiffs.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "driver/gpio.h"
#include "driver/pwm.h"

static const char *TAG = "PWM_Test";

gpio_config_t button_config = {
    .pin_bit_mask = (1ULL << GPIO_NUM_0),
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE
};

static void pwm_task(void *pvParameters) {
    while(1) {
        if (gpio_get_level(GPIO_NUM_0) != 1) {
            pwm_start();
            ESP_LOGI(TAG, "PWM ON");
            vTaskDelay(pdMS_TO_TICKS(5000)); // Keep PWM on for 5 seconds

            pwm_stop(0); // 0 means graceful stop; 1 would be immediate
            ESP_LOGI(TAG, "PWM OFF");
            vTaskDelay(pdMS_TO_TICKS(5000)); // Keep PWM off for 5 seconds
        }
    }
}

void app_main() {
    ESP_LOGI(TAG, "Start PWM Test\n");

    // Initialize GPIO for button
    gpio_config(&button_config);
    vTaskDelay(1000 / portTICK_RATE_MS);

    // PWM Configuration
    uint32_t duties[] = {250};      // Duty array for one channel
    uint32_t pin_num[] = {15};      // GPIO15 as PWM output
    float phases[] = {0};           // Phase array for one channel

    pwm_init(500, duties, 1, pin_num);
    pwm_set_phases(phases);

    ESP_LOGI(TAG, "Start RTOS Task\n");

    xTaskCreate(pwm_task, "pwm_task", 2048, NULL, 5, NULL);
}
