#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_flash.h"
#include "esp_spi_flash.h"
#include "esp_heap_caps.h"
#include "esp_random.h"

#define BUFFER_SIZE 1024

TaskHandle_t sensor_task_handle;
TaskHandle_t network_task_handle;

void sensor_task(void *pvParameters)
{
    ESP_LOGI("SENSOR_TASK", "Sensor task executing for pin %d", *(uint8_t *)pvParameters);
    // Log the task core id
    ESP_LOGI("SENSOR_TASK", "Sensor task core id: %d", xPortGetCoreID());

    while (1)
    {
        if (network_task_handle != NULL && eTaskGetState(network_task_handle) != eDeleted)
        {
            ESP_LOGI("SENSOR_TASK", "Sending task notification to network task from sensor task 1");
            
            // NOTE: Set first bit of notification value to 1
            xTaskNotify(network_task_handle, 0x01, eSetBits);
            vTaskDelay(1000 / portTICK_PERIOD_MS);

            // NOTE: Set second bit of notification value to 1
            xTaskNotify(network_task_handle, 0x02, eSetBits);
            vTaskDelay(1000 / portTICK_PERIOD_MS);

            // Note: Send notification value 123 to network task with overwrite
            //xTaskNotify(network_task_handle, 123, eSetValueWithOverwrite);
            //vTaskDelay(1000 / portTICK_PERIOD_MS);

            /*
            ESP_LOGI("SENSOR_TASK", "Sending task notification to network task from sensor task 1");
            // NOTE: Send task notification to network task
            xTaskNotifyGive(network_task_handle);
            */
        } else {
            ESP_LOGW("SENSOR_TASK", "Network task is not ready!");
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void network_task(void *pvParameters)
{
    ESP_LOGI("NETWORK_TASK", "Network task executing");
    // Log the task core id
    ESP_LOGI("NETWORK_TASK", "Network task core id: %d", xPortGetCoreID());

    int notification_value = 0;

    while (1)
    {   
        xTaskNotifyWait(0x01, 0x02, &notification_value, portMAX_DELAY);
        
        switch (notification_value)
        {
            case 0x01:
                ESP_LOGI("NETWORK_TASK", "First bit of notification value is set");
                break;
            case 0x02:
                ESP_LOGI("NETWORK_TASK", "Second bit of notification value is set");
                break;
            default:
                ESP_LOGI("NETWORK_TASK", "Notification value: %d", notification_value);
                break;
        }

        /*
        // NOTE: Wait for task notification from sensor task
        notification_count = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        ESP_LOGI("NETWORK_TASK", "Notification count: %d", notification_count);
        */
    }
}

void app_main(void)
{   
    esp_log_level_set("*", ESP_LOG_INFO);

    uint8_t pin_number_2 = 13;

    ESP_LOGI("MAIN", "Sensor task priority: %d", CONFIG_SENSOR_TASK_PRIORITY);
    ESP_LOGI("MAIN", "Network task priority: %d", CONFIG_NETWORK_TASK_PRIORITY);
    ESP_LOGI("MAIN", "Network task stack size: %d", CONFIG_NETWORK_TASK_STACK_SIZE);    

    xTaskCreatePinnedToCore(&sensor_task, "SENSOR_TASK", 2048, (void *)&pin_number_2, CONFIG_SENSOR_TASK_PRIORITY, NULL, 0);

    xTaskCreatePinnedToCore(&network_task, "NETWORK_TASK", CONFIG_NETWORK_TASK_STACK_SIZE, NULL, CONFIG_NETWORK_TASK_PRIORITY, &network_task_handle, 1);

    while (1)
    {
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}
