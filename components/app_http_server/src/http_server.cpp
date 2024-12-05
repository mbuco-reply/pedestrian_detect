// File: components/app_http_server/http_server.cpp
#include "http_server.hpp"
#include "rgb888_to_bmp.hpp" // Include the BMP conversion header
#include "../../common.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "esp_http_server.h"

// ---------------------- Configuration ----------------------

#define HTTP_PORT 80

#define FRAME_QUEUE_SIZE 10

#define FRAME_SIZE   (FRAME_WIDTH * FRAME_HEIGHT * 3) // RGB888: 3 bytes per pixel

// ---------------------- Global Definitions ----------------------

static const char *TAG = "App/HTTPServer";

// Global variables for frame management
static res_frame_t current_frame;
static SemaphoreHandle_t frame_mutex;
static QueueHandle_t frame_queue;

// ---------------------- Function Prototypes ----------------------

static esp_err_t stream_handler(httpd_req_t *req);
static esp_err_t root_handler(httpd_req_t *req);
static esp_err_t count_handler(httpd_req_t *req);
static httpd_handle_t start_webserver(void);
static void frame_task(void *arg);
esp_err_t publish_frame(uint8_t *buffer, size_t len);

// ---------------------- Function Implementations ----------------------

esp_err_t publish_frame(uint8_t *buffer, size_t len)
{
    if (len != FRAME_SIZE) {
        ESP_LOGE(TAG, "Invalid frame size: expected %d, got %zu", FRAME_SIZE, len);
        free(buffer); // Free the buffer to prevent memory leak
        return ESP_FAIL;
    }

    res_frame_t new_frame;
    new_frame.buffer = buffer;
    new_frame.len = len;
    new_frame.detect_results.count = 0; // Initialize count to 0, can be updated by producer

    if (xQueueSend(frame_queue, &new_frame, portMAX_DELAY) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to send frame to queue");
        free(buffer); // Free the buffer if it can't be sent
        return ESP_FAIL;
    }
    return ESP_OK;
}

void print_received_results(res_detect_t *detect_results)
{
    for(int i = 0; i < detect_results->count; i++)
    {
        ESP_LOGI(TAG,
                    "[x1: %d, y1: %d, x2: %d, y2: %d]\n",
                    detect_results->results[i].corners[0],
                    detect_results->results[i].corners[1],
                    detect_results->results[i].corners[2],
                    detect_results->results[i].corners[3]);
    }
}

static void frame_task(void *arg)
{
    res_frame_t incoming_frame;

    while (1) 
    {
        if (xQueueReceive(frame_queue, &incoming_frame, portMAX_DELAY) == pdTRUE) 
        {
            // Convert RGB888 to BMP
            uint8_t* bmp_buffer = NULL;
            size_t bmp_size = 0;
            
            bool success = rgb888_to_bmp(incoming_frame.buffer, FRAME_WIDTH, FRAME_HEIGHT, FRAME_WIDTH_OUT, FRAME_HEIGHT_OUT, &bmp_buffer, &bmp_size);
            draw_detected_rectangles_on_bmp(bmp_buffer, bmp_size, FRAME_WIDTH_OUT, FRAME_HEIGHT_OUT, FRAME_WIDTH, FRAME_HEIGHT, &incoming_frame.detect_results);
            // print_received_results(&incoming_frame.detect_results);
            
            // Free the incoming RGB888 buffer
            free(incoming_frame.buffer);

            if (!success) {
                ESP_LOGE(TAG, "Failed to convert RGB888 to BMP");
                continue; // Skip this frame
            }

            // Update the current frame under mutex protection
            if (xSemaphoreTake(frame_mutex, portMAX_DELAY) == pdTRUE) {
                // Free previous frame buffer if it exists
                if (current_frame.buffer) {
                    free(current_frame.buffer);
                }
                current_frame.buffer = bmp_buffer;
                current_frame.len = bmp_size;
                current_frame.detect_results = incoming_frame.detect_results; // Keep the detection results
                xSemaphoreGive(frame_mutex);
            } else {
                // Could not take mutex, free the bmp_buffer
                ESP_LOGE(TAG, "Failed to take frame mutex");
                free(bmp_buffer);
            }
        }
    }
}

static esp_err_t stream_handler(httpd_req_t *req)
{
    // Set response headers for BMP image
    httpd_resp_set_type(req, "image/bmp");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=\"image.bmp\"");
    httpd_resp_set_hdr(req, "Connection", "close");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    httpd_resp_set_hdr(req, "Pragma", "no-cache");

    uint8_t* bmp_buffer = NULL;
    size_t bmp_len = 0;

    if (xSemaphoreTake(frame_mutex, pdMS_TO_TICKS(500)) == pdTRUE) {
        if (current_frame.buffer && current_frame.len > 0) {
            // Copy the current frame buffer
            bmp_buffer = (uint8_t*)malloc(current_frame.len);
            if (bmp_buffer) {
                memcpy(bmp_buffer, current_frame.buffer, current_frame.len);
                bmp_len = current_frame.len;
            } else {
                ESP_LOGE(TAG, "Failed to allocate memory for BMP copy");
            }
        } else {
            ESP_LOGE(TAG, "No frame available");
        }
        xSemaphoreGive(frame_mutex);
    } else {
        ESP_LOGE(TAG, "Failed to take frame mutex");
    }

    if (bmp_buffer && bmp_len > 0) {
        // int64_t start_time = esp_timer_get_time();
        
        esp_err_t res = httpd_resp_send(req, (const char*)bmp_buffer, bmp_len);
        free(bmp_buffer); // Free the copied BMP buffer
        if (res != ESP_OK) {
            ESP_LOGE(TAG, "Failed to send BMP data");
            return res;
        }
        
        // int64_t end_time = esp_timer_get_time();
        // ESP_LOGI(TAG, "In Stream handler: http response time: %lld ms", (end_time - start_time) / 1000);
    } else {
        ESP_LOGE(TAG, "No BMP frame data available");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No BMP frame data available");
        return ESP_FAIL;
    }

    return ESP_OK;
}

static esp_err_t count_handler(httpd_req_t *req)
{
    // Return the current detection count as plain text
    uint16_t count = 0;

    if (xSemaphoreTake(frame_mutex, pdMS_TO_TICKS(500)) == pdTRUE) {
        count = current_frame.detect_results.count;
        xSemaphoreGive(frame_mutex);
    } else {
        ESP_LOGE(TAG, "Failed to take frame mutex for count");
    }

    char count_str[16];
    snprintf(count_str, sizeof(count_str), "%u", count);
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    httpd_resp_send(req, count_str, strlen(count_str));
    return ESP_OK;
}

static esp_err_t root_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "Client connected to /");

    const char* resp_str =
        "<!DOCTYPE html>\n"
        "<html>\n"
        "<head>\n"
        "    <title>ESP32 Detection</title>\n"
        "</head>\n"
        "<body>\n"
        "    <h1>ESP32 Detection</h1>\n"
        "    <img id=\"bmpImage\" src=\"/stream\" alt=\"Detection\" />\n"
        "    <div id=\"countDisplay\">Detect count: 0</div>\n"
        "    <script>\n"
        "        function updateImage() {\n"
        "            const img = document.getElementById(\"bmpImage\");\n"
        "            const timestamp = new Date().getTime();\n"
        "            img.src = \"/stream?t=\" + timestamp;\n"
        "            fetch('/count').then(response => response.text()).then(text => {\n"
        "                document.getElementById(\"countDisplay\").innerText = \"Detect count: \" + text;\n"
        "            });\n"
        "        }\n"
        "        setInterval(updateImage, 1000);\n"
        "    </script>\n"
        "</body>\n"
        "</html>";

    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, resp_str, strlen(resp_str));
    return ESP_OK;
}

static const httpd_uri_t stream_uri = {
    .uri       = "/stream",
    .method    = HTTP_GET,
    .handler   = stream_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t root_uri = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = root_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t count_uri = {
    .uri       = "/count",
    .method    = HTTP_GET,
    .handler   = count_handler,
    .user_ctx  = NULL
};

static httpd_handle_t start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = HTTP_PORT;

    httpd_handle_t server = NULL;
    ESP_LOGI(TAG, "Starting HTTP server on port %d", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        // Register the root URI handler
        httpd_register_uri_handler(server, &root_uri);
        ESP_LOGI(TAG, "/ endpoint registered");

        // Register the /stream URI handler
        httpd_register_uri_handler(server, &stream_uri);
        ESP_LOGI(TAG, "/stream endpoint registered");

        // Register the /count URI handler
        httpd_register_uri_handler(server, &count_uri);
        ESP_LOGI(TAG, "/count endpoint registered");

        return server;
    }

    ESP_LOGE(TAG, "Failed to start HTTP server");
    return NULL;
}

esp_err_t start_http_server(QueueHandle_t stream_queue)
{
    // Initialize mutex for frame access
    frame_mutex = xSemaphoreCreateMutex();
    if (frame_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create frame mutex");
        return ESP_FAIL;
    }

    // Initialize current frame to NULL
    current_frame.buffer = NULL;
    current_frame.len = 0;
    current_frame.detect_results.count = 0; // Initialize to 0

    // Use the provided stream queue
    frame_queue = stream_queue;

    // Start the HTTP server
    httpd_handle_t server = start_webserver();
    if (server == NULL) {
        ESP_LOGE(TAG, "HTTP server failed to start");
        return ESP_FAIL;
    }

    // Start the frame processing task
    xTaskCreate(frame_task, "frame_task", 8192, NULL, 5, NULL);

    return ESP_OK;
}
