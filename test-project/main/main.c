#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"

#define NUM_SSIDs 2
#define MAX_AP_COUNT 10
static const char *TAG = "WIFI_SCANNER";
const char *target_ssids[NUM_SSIDs] = {"AP", "ESP32-AP-2"};

TaskHandle_t network_task_handle = NULL;
SemaphoreHandle_t sensor_data_semaphore;
int sensor_data = 0;

void dummy_read_sensor() {
    ESP_LOGI("DUMMY_READ_SENSOR", "Sensor data updated to: %d", ++sensor_data);
}

void dummy_eeprom_write() {
    ESP_LOGI("DUMMY_EEPROM_WRITE", "Writing sensor data to EEPROM: %d", sensor_data);
}

void sensor_task(void *pvParameters) {
    ESP_LOGI("SENSOR_TASK", "Sensor task running on core: %d", xPortGetCoreID());
    while (1) {
        dummy_read_sensor();
        xSemaphoreGive(sensor_data_semaphore);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void eeprom_task(void *pvParameters) {
    ESP_LOGI("EEPROM_TASK", "EEPROM task running on core: %d", xPortGetCoreID());
    while (1) {
        if (xSemaphoreTake(sensor_data_semaphore, portMAX_DELAY)) {
            dummy_eeprom_write();
        }
    }
}

static void process_scan_results(uint16_t ap_count, wifi_ap_record_t *ap_records) {
    bool ssid_found = false;
    for (int i = 0; i < ap_count; i++) {
        ESP_LOGI(TAG, "Found SSID: %s", ap_records[i].ssid);
        for (int j = 0; j < NUM_SSIDs; j++) {
            if (strcmp((char *)ap_records[i].ssid, target_ssids[j]) == 0) {
                ESP_LOGI(TAG, "Target SSID '%s' detected!", target_ssids[j]);
                ssid_found = true;
                break;
            }
        }
    }

    uint32_t notification_value = ssid_found ? 1 : 0;
    if (network_task_handle != NULL) {
        if (eTaskGetState(network_task_handle) != eDeleted) {
            xTaskNotify(network_task_handle, notification_value, eSetValueWithOverwrite);
        } else {
            ESP_LOGE(TAG, "Network task is deleted!");
        }
    } else {
        ESP_LOGE(TAG, "Network task handle is NULL!");
    }
}


static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_SCAN_DONE) {
        uint16_t ap_count = MAX_AP_COUNT;
        wifi_ap_record_t ap_records[MAX_AP_COUNT];
        if (esp_wifi_scan_get_ap_records(&ap_count, ap_records) == ESP_OK) {
            process_scan_results(ap_count, ap_records);
        } else {
            ESP_LOGE(TAG, "Failed to fetch scan results!");
        }
    }
}

void scan_wifi(const wifi_scan_config_t *scan_config) {
    ESP_LOGI(TAG, "Starting WiFi scan...");
    if (esp_wifi_scan_start(scan_config, false) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start WiFi scan!");
    } else {
        ESP_LOGI(TAG, "WiFi scan started.");
    }
}

void network_task(void *pvParameters) {
    ESP_LOGI(TAG, "Network task running on core: %d", xPortGetCoreID());

    wifi_scan_config_t scan_config;
    uint32_t notification_value = 0;
    static int ssid_index = 0;

    while (1) {
        scan_config.ssid = (uint8_t *)target_ssids[ssid_index];
        scan_config.bssid = NULL;
        scan_config.channel = 0;
        scan_config.show_hidden = false;
        scan_config.scan_type = WIFI_SCAN_TYPE_PASSIVE; // Tarama tipi: pasif
        scan_config.scan_time.passive = 3000;  // Her kanalda bekleme süresi (ms)

        scan_wifi(&scan_config);

        if (xTaskNotifyWait(0, 0, &notification_value, portMAX_DELAY)) {
            if (notification_value == 1) {
                ESP_LOGI(TAG, "Target SSID '%s' found!", target_ssids[ssid_index]);
            } else {
                ESP_LOGW(TAG, "Target SSID '%s' not found.", target_ssids[ssid_index]);
            }
        } else {
            ESP_LOGW(TAG, "WiFi scan timed out for SSID: %s", target_ssids[ssid_index]);
        }

        ssid_index = (ssid_index + 1) % NUM_SSIDs;
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "Initializing NVS...");
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    sensor_data_semaphore = xSemaphoreCreateBinary();
    if (!sensor_data_semaphore) {
        ESP_LOGE(TAG, "Failed to create sensor_data_semaphore!");
        return;
    }

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, WIFI_EVENT_SCAN_DONE, wifi_event_handler, NULL, NULL));

    xTaskCreatePinnedToCore(sensor_task, "SENSOR_TASK", 2048, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(eeprom_task, "EEPROM_TASK", 2048, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(network_task, "NETWORK_TASK", 8192, NULL, 5, &network_task_handle, 0);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
