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
    while (1)
    {
        ESP_LOGI("SENSOR_TASK", "Sensor task executing for pin %d", *(uint8_t *)pvParameters);
        // Log the task core id
        ESP_LOGI("SENSOR_TASK", "Sensor task core id: %d", xPortGetCoreID());

        vTaskDelay(1000 / portTICK_PERIOD_MS);
        
        // NOTE: Send task notification to network task
        xTaskNotifyGive(network_task_handle);

        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

void network_task(void *pvParameters)
{
    while (1)
    {
        ESP_LOGI("NETWORK_TASK", "Network task executing");
        // Log the task core id
        ESP_LOGI("NETWORK_TASK", "Network task core id: %d", xPortGetCoreID());
        
        // NOTE: Wait for task notification from sensor task
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        ESP_LOGI("NETWORK_TASK", "Received task notification from sensor task");
    }
}

void app_main(void)
{   
    esp_log_level_set("*", ESP_LOG_INFO);

    uint8_t pin_number_2 = 13;

    xTaskCreatePinnedToCore(&sensor_task, "SENSOR_TASK_2", 2048, (void *)&pin_number_2, 5, &sensor_task_handle, 0);
    xTaskCreatePinnedToCore(&network_task, "NETWORK_TASK", 2048, NULL, 5, &network_task_handle, 1);

    while (1)
    {
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}
