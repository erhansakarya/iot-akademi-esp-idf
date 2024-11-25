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

#define BUFFER_SIZE 1024

TaskHandle_t sensor_task_handle;
TaskHandle_t network_task_handle;

// NOTE: Create a semaphore to synchronize access to the I2C
SemaphoreHandle_t i2c_semaphore;

// NOTE: Dummy I2C read function
void dummy_i2c_read()
{
    ESP_LOGI("DUMMY_I2C_READ", "Dummy I2C read function started");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    ESP_LOGI("DUMMY_I2C_READ", "Dummy I2C read function finished");
}

// NOTE: Dummy I2C write function
void dummy_i2c_write()
{
    ESP_LOGI("DUMMY_I2C_WRITE", "Dummy I2C write function started");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    ESP_LOGI("DUMMY_I2C_WRITE", "Dummy I2C write function finished");
}

void sensor_task(void *pvParameters)
{
    ESP_LOGI("SENSOR_TASK", "Sensor task executing for pin %d", *(uint8_t *)pvParameters);
    // Log the task core id
    ESP_LOGI("SENSOR_TASK", "Sensor task core id: %d", xPortGetCoreID());

    while (1)
    {
        if (xSemaphoreTake(i2c_semaphore, portMAX_DELAY) == pdTRUE)
        {
            ESP_LOGI("SENSOR_TASK", "I2C semaphore taken");
            dummy_i2c_read();
            xSemaphoreGive(i2c_semaphore);
            ESP_LOGI("SENSOR_TASK", "I2C semaphore given");
        }
        else
        {
            ESP_LOGW("SENSOR_TASK", "Failed to take I2C semaphore");
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void eeprom_task(void *pvParameters)
{
    // Log the task core id
    ESP_LOGI("EEPROM_TASK", "Sensor task core id: %d", xPortGetCoreID());

    while (1)
    {
        if (xSemaphoreTake(i2c_semaphore, portMAX_DELAY) == pdTRUE)
        {
            ESP_LOGI("EEPROM_TASK", "I2C semaphore taken");
            dummy_i2c_read();
            xSemaphoreGive(i2c_semaphore);
            ESP_LOGI("EEPROM_TASK", "I2C semaphore given");
        }
        else
        {
            ESP_LOGW("EEPROM_TASK", "Failed to take I2C semaphore");
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS);
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

    uint8_t pin_number_2 = 13;

    ESP_LOGI("MAIN", "Sensor task priority: %d", CONFIG_SENSOR_TASK_PRIORITY);
    ESP_LOGI("MAIN", "Network task priority: %d", CONFIG_NETWORK_TASK_PRIORITY);
    ESP_LOGI("MAIN", "Network task stack size: %d", CONFIG_NETWORK_TASK_STACK_SIZE);    

    // NOTE: Create a semaphore to synchronize access to the I2C
    i2c_semaphore = xSemaphoreCreateMutex();
    if (i2c_semaphore == NULL)
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
