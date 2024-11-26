#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_flash.h"
#include "esp_spi_flash.h"
#include "esp_heap_caps.h"
#include "esp_random.h"
#include "nvs_flash.h"

#define BUFFER_SIZE 1024

TaskHandle_t sensor_task_handle;
TaskHandle_t network_task_handle;

// NOTE: Create a semaphore to synchronize access to the sensor data
SemaphoreHandle_t sensor_data_semaphore;
// NOTE: Create a sensor data variable
int sensor_data = 0;   

// NOTE: Dummy read sensor function
void dummy_read_sensor()
{
    ESP_LOGI("DUMMY_READ_SENSOR", "Dummy read sensor function started");
    sensor_data++;
    ESP_LOGI("DUMMY_READ_SENSOR", "Dummy read sensor function finished");
}

// NOTE: Dummy EEPROM write function
void dummy_eeprom_write()
{
    ESP_LOGI("DUMMY_EEPROM_WRITE", "Dummy EEPROM write function started");
    ESP_LOGI("DUMMY_EEPROM_WRITE", "Sensor data: %d", sensor_data);
    ESP_LOGI("DUMMY_EEPROM_WRITE", "Dummy EEPROM write function finished");
}

void sensor_task(void *pvParameters)
{
    ESP_LOGI("SENSOR_TASK", "Sensor task executing for pin %d", *(uint8_t *)pvParameters);
    // Log the task core id
    ESP_LOGI("SENSOR_TASK", "Sensor task core id: %d", xPortGetCoreID());

    while (1)
    {
        // NOTE: Read the sensor data
        dummy_read_sensor();

        // NOTE: Give the semaphore to the EEPROM task
        xSemaphoreGive(sensor_data_semaphore);

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void eeprom_task(void *pvParameters)
{
    // Log the task core id
    ESP_LOGI("EEPROM_TASK", "Sensor task core id: %d", xPortGetCoreID());

    while (1)
    {
        if (xSemaphoreTake(sensor_data_semaphore, portMAX_DELAY) == pdTRUE)
        {
            dummy_eeprom_write();
        }
    }
}

void network_task(void *pvParameters)
{
    ESP_LOGI("NETWORK_TASK", "Network task executing");
    // Log the task core id
    ESP_LOGI("NETWORK_TASK", "Network task core id: %d", xPortGetCoreID());

    while (1)
    {   
        ESP_LOGI("NETWORK_TASK", "Network task executing");
        vTaskDelay(1000 / portTICK_PERIOD_MS);  
    }
}

void app_main(void)
{   
    esp_log_level_set("*", ESP_LOG_INFO);

    // NOTE: Init NVS Flash
    ESP_ERROR_CHECK(nvs_flash_init());  

    nvs_handle_t storage_handle;
    int32_t restart_counter = 0;

    ESP_ERROR_CHECK(nvs_open("storage", NVS_READWRITE, &storage_handle));

    esp_err_t err = nvs_get_i32(storage_handle, "restart_counter", &restart_counter);
    if (err == ESP_ERR_NVS_NOT_FOUND)
    {
        ESP_LOGI("MAIN", "Restart counter not found, setting to 0");
        restart_counter = 0;
    }
    else if (err != ESP_OK)
    {
        ESP_LOGE("MAIN", "Error reading restart counter: %s", esp_err_to_name(err));
    }

    // NOTE: Increment the restart counter
    restart_counter++;
    ESP_LOGI("MAIN", "Restart counter: %ld", restart_counter);

    // NOTE: Write the restart counter to NVS
    ESP_ERROR_CHECK(nvs_set_i32(storage_handle, "restart_counter", restart_counter));  
    ESP_ERROR_CHECK(nvs_commit(storage_handle));
    nvs_close(storage_handle);

    vTaskDelay(5000 / portTICK_PERIOD_MS);

    // NOTE: Execute software reset
    esp_restart();

    uint8_t pin_number_2 = 13;

    ESP_LOGI("MAIN", "Sensor task priority: %d", CONFIG_SENSOR_TASK_PRIORITY);
    ESP_LOGI("MAIN", "Network task priority: %d", CONFIG_NETWORK_TASK_PRIORITY);
    ESP_LOGI("MAIN", "Network task stack size: %d", CONFIG_NETWORK_TASK_STACK_SIZE);    

    sensor_data_semaphore = xSemaphoreCreateBinary();
    if (sensor_data_semaphore == NULL)
    {
        ESP_LOGE("MAIN", "Failed to create semaphore");
        return;
    }

    xTaskCreatePinnedToCore(&sensor_task, "SENSOR_TASK", 2048, (void *)&pin_number_2, CONFIG_SENSOR_TASK_PRIORITY, NULL, 0);
    xTaskCreatePinnedToCore(&eeprom_task, "EEPROM_TASK", 2048, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(&network_task, "NETWORK_TASK", CONFIG_NETWORK_TASK_STACK_SIZE, NULL, CONFIG_NETWORK_TASK_PRIORITY, &network_task_handle, 1);

    while (1)
    {
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}
