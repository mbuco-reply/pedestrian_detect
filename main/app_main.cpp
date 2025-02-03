// File: main/app_main.cpp

#include <string>
#include <inttypes.h> // For PRIu32
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_flash.h"      // For esp_flash_get_size
#include "esp_heap_caps.h"
#include "esp_wifi.h"       // If needed after Wi-Fi init, but for now we get default MAC from efuse
#include "esp_chip_info.h"

// Your project headers
#include "app_camera.hpp"
#include "app_detect.hpp"
#include "app_lcd.hpp"
#include "app_wifi.hpp"
#include "http_server.hpp"
#include "../components/common.hpp"

static const char *TAG = "MAIN";

// Declare external functions if needed
extern "C" void app_main(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret); // Ensure NVS is initialized successfully

    ESP_LOGI(TAG, "===== System Information =====");

    // Print IDF Version
    ESP_LOGI(TAG, "ESP-IDF version: %s", esp_get_idf_version());

    // Print chip info
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    ESP_LOGI(TAG, "Chip info:");
    ESP_LOGI(TAG, " - Model: %s", (chip_info.model == CHIP_ESP32) ? "ESP32" :
                                   (chip_info.model == CHIP_ESP32S2) ? "ESP32-S2" :
                                   (chip_info.model == CHIP_ESP32S3) ? "ESP32-S3" :
                                   (chip_info.model == CHIP_ESP32C3) ? "ESP32-C3" : "Unknown");
    ESP_LOGI(TAG, " - Cores: %d", chip_info.cores);
    ESP_LOGI(TAG, " - Features: %s%s%s",
             (chip_info.features & CHIP_FEATURE_BT) ? "BT " : "",
             (chip_info.features & CHIP_FEATURE_BLE) ? "BLE " : "",
             (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "Embedded-Flash" : "External-Flash");
    ESP_LOGI(TAG, " - Revision: %d", chip_info.revision);

    // Get flash size
    uint32_t flash_size = 0;
    if (esp_flash_get_size(NULL, &flash_size) == ESP_OK) {
        ESP_LOGI(TAG, "Flash size: %" PRIu32 " MB", flash_size / (1024 * 1024));
    } else {
        ESP_LOGW(TAG, "Cannot determine flash size");
    }

    // Heap info
    uint32_t free_heap = (uint32_t)esp_get_free_heap_size();
    uint32_t min_free_heap = (uint32_t)esp_get_minimum_free_heap_size();
    ESP_LOGI(TAG, "Heap Info:");
    ESP_LOGI(TAG, " - Free heap size: %" PRIu32 " bytes", free_heap);
    ESP_LOGI(TAG, " - Minimum free heap size: %" PRIu32 " bytes", min_free_heap);

    ESP_LOGI(TAG, "==============================");

    // Initialize Wi-Fi
    std::string wifi_ssid = "TP-Link_3E20";
    std::string wifi_password = "password";
    AppWiFi wifi(wifi_ssid, wifi_password);
    wifi.run();

    // Create Queues
    QueueHandle_t camera_to_detect_queue = xQueueCreate(3, sizeof(camera_fb_t *));
    QueueHandle_t camera_to_lcd_queue = xQueueCreate(3, sizeof(camera_fb_t *));
    QueueHandle_t camera_to_stream_queue = xQueueCreate(3, sizeof(res_frame_t));

    if (camera_to_detect_queue == NULL || camera_to_lcd_queue == NULL || camera_to_stream_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create queues");
        return;
    }

    // Initialize Components
    AppCamera *camera = new AppCamera(PIXFORMAT_JPEG, FRAMESIZE_VGA, 2, camera_to_detect_queue);
    AppDetect *detector = new AppDetect(camera_to_detect_queue, camera_to_stream_queue);
    // AppLCD *lcd = new AppLCD(camera_to_lcd_queue, nullptr);  // No further queue

    // Initialize HTTP Server with camera_to_stream_queue
    esp_err_t server_status = start_http_server(camera_to_stream_queue);
    if (server_status != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server");
    }

    // Run Camera and Detector
    camera->run();
    detector->run();
    // lcd->run();
}
