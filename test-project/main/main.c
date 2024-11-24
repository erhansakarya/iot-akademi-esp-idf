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
            // NOTE: Send task notification to network task
            xTaskNotifyGive(network_task_handle);
        } else {
            ESP_LOGW("SENSOR_TASK", "Network task is not ready!");
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void sensor_2_task(void *pvParameters)
{
    ESP_LOGI("SENSOR_2_TASK", "Sensor task executing for pin %d", *(uint8_t *)pvParameters);
    // Log the task core id
    ESP_LOGI("SENSOR_2_TASK", "Sensor task core id: %d", xPortGetCoreID());

    while (1)
    {
        if (network_task_handle != NULL && eTaskGetState(network_task_handle) != eDeleted)
        {
            ESP_LOGI("SENSOR_2_TASK", "Sending task notification to network task from sensor task 2");
            // NOTE: Send task notification to network task
            xTaskNotifyGive(network_task_handle);
            // NOTE: Send task notification to network task
            xTaskNotifyGive(network_task_handle);
        } else {
            ESP_LOGW("SENSOR_TASK", "Network task is not ready!");
        }

        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

void network_task(void *pvParameters)
{
    ESP_LOGI("NETWORK_TASK", "Network task executing");
    // Log the task core id
    ESP_LOGI("NETWORK_TASK", "Network task core id: %d", xPortGetCoreID());

    unsigned int notification_count = 0;

    while (1)
    {   
        // NOTE: Wait for task notification from sensor task
        notification_count = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        ESP_LOGI("NETWORK_TASK", "Notification count: %d", notification_count);
    }
}

void app_main(void)
{   
    esp_log_level_set("*", ESP_LOG_INFO);

    uint8_t pin_number_2 = 13;

    xTaskCreatePinnedToCore(&sensor_task, "SENSOR_TASK", 2048, (void *)&pin_number_2, 5, NULL, 0);
    xTaskCreatePinnedToCore(&sensor_2_task, "SENSOR_2_TASK", 2048, (void *)&pin_number_2, 5, NULL, 0);

    xTaskCreatePinnedToCore(&network_task, "NETWORK_TASK", 2048, NULL, 6, &network_task_handle, 1);

    while (1)
    {
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}
