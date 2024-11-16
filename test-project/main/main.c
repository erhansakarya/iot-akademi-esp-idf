#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_flash.h"
#include "esp_spi_flash.h"
#include "esp_heap_caps.h"
#include "esp_random.h"

//#define TAG_1 "LED_TASK_1"

#define BUFFER_SIZE 1024

void sensor_task(void *pvParameters)
{
    while (1)
    {
        ESP_LOGI("SENSOR_TASK", "Sensor task executing for pin %d", *(uint8_t *)pvParameters);
        // Log the task core id
        ESP_LOGI("SENSOR_TASK", "Sensor task core id: %d", xPortGetCoreID());
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
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{   
    esp_log_level_set("*", ESP_LOG_INFO);

    uint8_t pin_number_1 = 12;
    uint8_t pin_number_2 = 13;

    TaskHandle_t sensor_task_1_handle;
    TaskHandle_t sensor_task_2_handle;
    TaskHandle_t network_task_handle;

    xTaskCreate(&sensor_task, "SENSOR_TASK_1", 2048, (void *)&pin_number_1, 5, &sensor_task_1_handle);
    //xTaskCreate(&sensor_task, "SENSOR_TASK_2", 2048, (void *)&pin_number_2, 5, &sensor_task_2_handle);
    xTaskCreatePinnedToCore(&sensor_task, "SENSOR_TASK_2", 2048, (void *)&pin_number_2, 5, &sensor_task_2_handle, 0);
    //xTaskCreate(&network_task, "NETWORK_TASK", 2048, NULL, 5, &network_task_handle);
    xTaskCreatePinnedToCore(&network_task, "NETWORK_TASK", 2048, NULL, 5, &network_task_handle, 1);

    vTaskDelay(5000 / portTICK_PERIOD_MS);
    vTaskSuspend(sensor_task_1_handle);
    ESP_LOGI("MAIN", "Sensor task 1 suspended");

    vTaskDelay(5000 / portTICK_PERIOD_MS);
    vTaskResume(sensor_task_1_handle);
    ESP_LOGI("MAIN", "Sensor task 1 resumed");

    vTaskDelay(5000 / portTICK_PERIOD_MS);
    vTaskDelete(sensor_task_1_handle);
    ESP_LOGI("MAIN", "Sensor task 1 deleted");

    eTaskState state = eTaskGetState(sensor_task_1_handle);
    ESP_LOGI("MAIN", "Sensor task 1 state: %d", state);

    while (1)
    {
        /*
        UBaseType_t free_heap_size = uxTaskGetStackHighWaterMark(sensor_task_1_handle);
        ESP_LOGI("MAIN", "Free heap size: %d for sensor task 1", free_heap_size);
        */

        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}
