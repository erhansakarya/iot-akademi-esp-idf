#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_flash.h"
#include "esp_spi_flash.h"
#include "esp_heap_caps.h"
#include "esp_random.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"

#define BUFFER_SIZE 1024

#define NUM_SSIDs 2
#define MAX_AP_COUNT 2
static const char *TAG = "WIFI_SCANNER";
const char *target_ssids[NUM_SSIDs] = {"AP", "ESP32-AP-2"};

TaskHandle_t sensor_task_handle;
TaskHandle_t network_task_handle;

// NOTE: Create a semaphore to synchronize access to the sensor data
SemaphoreHandle_t sensor_data_semaphore;
// NOTE: Create a sensor data variable
int sensor_data = 0;   
static QueueHandle_t sensor_data_queue;

// NOTE: Dummy read sensor function
void dummy_read_sensor()
{
    ESP_LOGI("DUMMY_READ_SENSOR", "Dummy read sensor function started");
    sensor_data++;
    ESP_LOGI("DUMMY_READ_SENSOR", "Dummy read sensor function finished");
}

// NOTE: Dummy EEPROM write function
void dummy_eeprom_write(int received_data)
{
    ESP_LOGI("DUMMY_EEPROM_WRITE", "Dummy EEPROM write function started");
    ESP_LOGI("DUMMY_EEPROM_WRITE", "Sensor data from queue: %d", received_data);
    ESP_LOGI("DUMMY_EEPROM_WRITE", "Dummy EEPROM write function finished");
}

void sensor_task(void *pvParameters)
{
    ESP_LOGI("SENSOR_TASK", "Sensor task executing on core %d", xPortGetCoreID());

    while (1)
    {
        // Read the sensor data
        dummy_read_sensor();

        // Send data to queue
        if (xQueueSend(sensor_data_queue, &sensor_data, pdMS_TO_TICKS(100)) == pdPASS)
        {
            ESP_LOGI("SENSOR_TASK", "Sensor data sent to queue: %d", sensor_data);
        }
        else
        {
            ESP_LOGE("SENSOR_TASK", "Failed to send sensor data to queue, queue is full!");
        }

        // Give the semaphore to signal EEPROM task
        xSemaphoreGive(sensor_data_semaphore);

        // Wait for 1 second
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// EEPROM Task
void eeprom_task(void *pvParameters)
{
    ESP_LOGI("EEPROM_TASK", "EEPROM task executing on core %d", xPortGetCoreID());
    int received_data = 0;

    while (1)
    {
        // Wait for the semaphore
        if (xSemaphoreTake(sensor_data_semaphore, portMAX_DELAY) == pdTRUE)
        {
            // Receive data from queue
            if (xQueueReceive(sensor_data_queue, &received_data, pdMS_TO_TICKS(100)) == pdPASS)
            {
                // Write data to EEPROM
                dummy_eeprom_write(received_data);
            }
            else
            {
                ESP_LOGE("EEPROM_TASK", "Failed to receive data from queue!");
            }
        }
    }
}

static void process_scan_results(uint16_t ap_count, wifi_ap_record_t *ap_records)
{
    bool ssid_found = false;

    for (int i = 0; i < ap_count; i++)
    {
        ESP_LOGW(TAG, "SSID: %s, RSSI: %d, Channel: %d", ap_records[i].ssid, ap_records[i].rssi, ap_records[i].primary);    
        for (int j = 0; j < NUM_SSIDs; j++)
        {
            if (strcmp((const char *)ap_records[i].ssid, target_ssids[j]) == 0)
            {
                ssid_found = true;
                break;
            }
        }
    }

    uint32_t notification_value = ssid_found ? 1 : 0;
    if (network_task_handle != NULL)
    {
        if (eTaskGetState(network_task_handle) != eDeleted) {
            xTaskNotify(network_task_handle, notification_value, eSetValueWithOverwrite);
        } else {
            ESP_LOGW(TAG, "Network task is deleted");
        }
    } else {
        ESP_LOGW(TAG, "Network task handle is NULL");   
    }
}   

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_SCAN_DONE) {
        uint16_t ap_count = MAX_AP_COUNT;
        wifi_ap_record_t ap_records[MAX_AP_COUNT];
        if (esp_wifi_scan_get_ap_records(&ap_count, ap_records) == ESP_OK) {
            process_scan_results(ap_count, ap_records);
        } else {
            ESP_LOGW(TAG, "Failed to get AP records");
        }
    }
}   

void scan_wifi(wifi_scan_config_t *scan_config)
{
    ESP_LOGI(TAG, "Starting wifi scan...");
    if (esp_wifi_scan_start(scan_config, false) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to start wifi scan");
    } else {
        ESP_LOGI(TAG, "Wifi scan started");
    }
}   

void network_task(void *pvParameters)
{
    // Log the task core id
    ESP_LOGI("NETWORK_TASK", "Network task core id: %d", xPortGetCoreID());

    wifi_scan_config_t scan_config;
    uint32_t notification_value = 0;
    static int ssid_index = 0;

    while (1)
    {   
        // NOTE: Set scan config
        scan_config.ssid = (uint8_t *)target_ssids[ssid_index];
        scan_config.bssid = NULL;
        scan_config.channel = 0;
        scan_config.show_hidden = false;
        scan_config.scan_type = WIFI_SCAN_TYPE_PASSIVE;
        scan_config.scan_time.passive = 3000;

        // NOTE: Start scan
        scan_wifi(&scan_config);  
        
        // NOTE: Wait for notification
        if (xTaskNotifyWait(0, 0, &notification_value, portMAX_DELAY) == pdTRUE) {
            if (notification_value == 1) {
                ESP_LOGW(TAG, "SSID %s found", target_ssids[ssid_index]);
            } else {
                ESP_LOGW(TAG, "SSID %s not found", target_ssids[ssid_index]);
            }
        } else {
            ESP_LOGW(TAG, "Failed to wait for notification");
        }

        // NOTE: Increment the SSID index
        ssid_index = (ssid_index + 1) % NUM_SSIDs;
        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{   
    esp_log_level_set("*", ESP_LOG_INFO);

    // NOTE: Init NVS Flash
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }

#if 0
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
#endif
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

    // Create Queue
    sensor_data_queue = xQueueCreate(5, sizeof(int));
    if (sensor_data_queue == NULL)
    {
        ESP_LOGE(TAG, "Failed to create queue");
        return;
    }

    // NOTE: Init WiFi Netif
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    // NOTE: Init WiFi
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_start()); 

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, WIFI_EVENT_SCAN_DONE, wifi_event_handler, NULL, NULL));

    xTaskCreatePinnedToCore(&sensor_task, "SENSOR_TASK", 2048, (void *)&pin_number_2, CONFIG_SENSOR_TASK_PRIORITY, NULL, 0);
    xTaskCreatePinnedToCore(&eeprom_task, "EEPROM_TASK", 2048, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(&network_task, "NETWORK_TASK", CONFIG_NETWORK_TASK_STACK_SIZE, NULL, CONFIG_NETWORK_TASK_PRIORITY, &network_task_handle, 1);

    while (1)
    {
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}