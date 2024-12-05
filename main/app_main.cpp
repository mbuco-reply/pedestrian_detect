#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "app_camera.hpp"
#include "app_detect.hpp"
#include "app_lcd.hpp"
#include "app_wifi.hpp"
#include "http_server.hpp"

static const char *TAG = "MAIN";

// Structure to hold frame buffer information
typedef struct {
    uint8_t *buffer; // Pointer to the frame data (e.g., JPEG)
    size_t len;      // Length of the frame data
} frame_t;

// Structure to pass camera frames
typedef struct {
    camera_fb_t *frame;
} camera_frame_t;

// Forwarder Task Prototype
static void frame_forwarder_task(void *pvParameters);

// Frame Forwarder Task Implementation
static void frame_forwarder_task(void *pvParameters)
{
    // Parameters: Tuple of Queues
    QueueHandle_t *queues = (QueueHandle_t *)pvParameters;
    QueueHandle_t camera_to_detect_queue = queues[0];
    QueueHandle_t camera_to_lcd_queue = queues[1];
    QueueHandle_t camera_to_stream_queue = queues[2];

    while (true) {
        camera_frame_t incoming_frame;
        if (xQueueReceive(camera_to_detect_queue, &incoming_frame, portMAX_DELAY)) {
            // Forward to LCD
            camera_frame_t lcd_frame = { .frame = incoming_frame.frame };
            if (xQueueSend(camera_to_lcd_queue, &lcd_frame, portMAX_DELAY) != pdPASS) {
                ESP_LOGE(TAG, "Failed to forward frame to LCD queue");
            }

            // Forward to Stream
            camera_frame_t stream_frame = { .frame = incoming_frame.frame };
            if (xQueueSend(camera_to_stream_queue, &stream_frame, portMAX_DELAY) != pdPASS) {
                ESP_LOGE(TAG, "Failed to forward frame to Stream queue");
            }
        }
    }

    vTaskDelete(NULL);
}

extern "C" void app_main(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret); // Ensure NVS is initialized successfully

    // Initialize Wi-Fi
    std::string wifi_ssid = "BucoNexusHQ";
    std::string wifi_password = "79452952864284852700";
    AppWiFi wifi(wifi_ssid, wifi_password);
    wifi.run();

    // Create Queues
    QueueHandle_t camera_to_detect_queue = xQueueCreate(5, sizeof(camera_frame_t));
    QueueHandle_t camera_to_lcd_queue = xQueueCreate(5, sizeof(camera_frame_t));
    QueueHandle_t camera_to_stream_queue = xQueueCreate(5, sizeof(frame_t));

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

    // Initialize Frame Forwarder Task
    // QueueHandle_t forwarder_queues[3] = { camera_to_detect_queue, camera_to_lcd_queue, camera_to_stream_queue };
    // xTaskCreate(&frame_forwarder_task, "FrameForwarder", 4096, (void *)forwarder_queues, 5, NULL);

    // Run Camera and Detector
    camera->run();
    detector->run();
    // lcd->run();
}
