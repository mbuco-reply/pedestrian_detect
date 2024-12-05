// File: components/app_http_server/http_server.cpp
#include "http_server.hpp"
#include "rgb888_to_bmp.hpp" // Include the BMP conversion header
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "esp_http_server.h"
#include "esp_log.h"

// ---------------------- Configuration ----------------------

#define HTTP_PORT 80

#define FRAME_QUEUE_SIZE 10

#define FRAME_WIDTH  640
#define FRAME_HEIGHT 480
#define FRAME_SIZE   (FRAME_WIDTH * FRAME_HEIGHT * 3) // RGB888: 3 bytes per pixel

// ---------------------- Global Definitions ----------------------

static const char *TAG = "HTTP_SERVER";

// Structure to hold frame buffer information
typedef struct {
    uint8_t *buffer; // Pointer to the frame data (BMP data after modification)
    size_t len;      // Length of the frame data
} frame_t;

// Global variables for frame management
static frame_t current_frame;
static SemaphoreHandle_t frame_mutex;
static QueueHandle_t frame_queue;

// ---------------------- Function Prototypes ----------------------

static esp_err_t stream_handler(httpd_req_t *req);
static esp_err_t root_handler(httpd_req_t *req);
static httpd_handle_t start_webserver(void);
static void frame_task(void *arg);
esp_err_t publish_frame(uint8_t *buffer, size_t len);

// ---------------------- Function Implementations ----------------------

/**
 * @brief Publish a new frame to the frame queue.
 *
 * This function should be called by other parts of your application
 * whenever a new frame is available for streaming.
 *
 * @param buffer Pointer to the frame data (must be dynamically allocated)
 * @param len    Length of the frame data
 * @return esp_err_t ESP_OK on success, ESP_FAIL on failure
 *
 * @note After calling this function, do not modify or free the buffer.
 *       The frame_task will handle freeing the buffer.
 */
esp_err_t publish_frame(uint8_t *buffer, size_t len)
{
    if (len != FRAME_SIZE) {
        ESP_LOGE(TAG, "Invalid frame size: expected %d, got %zu", FRAME_SIZE, len);
        return ESP_FAIL;
    }

    frame_t new_frame;
    new_frame.buffer = buffer;
    new_frame.len = len;

    if (xQueueSend(frame_queue, &new_frame, portMAX_DELAY) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to send frame to queue");
        free(buffer); // Free the buffer if it can't be sent
        return ESP_FAIL;
    }
    return ESP_OK;
}

/**
 * @brief FreeRTOS task to process incoming frames from the queue.
 *
 * This task continuously waits for new frames, converts them to BMP,
 * and updates the current frame buffer for streaming.
 *
 * @param arg Not used
 */
static void frame_task(void *arg)
{
    frame_t incoming_frame;
    while (1) {
        // Wait indefinitely for a new frame
        if (xQueueReceive(frame_queue, &incoming_frame, portMAX_DELAY) == pdTRUE) {
            // Convert RGB888 to BMP
            uint8_t* bmp_buffer = NULL;
            size_t bmp_size = 0;
            bool success = rgb888_to_bmp(incoming_frame.buffer, FRAME_WIDTH, FRAME_HEIGHT, &bmp_buffer, &bmp_size);
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
                xSemaphoreGive(frame_mutex);
            } else {
                // Could not take mutex, free the bmp_buffer
                ESP_LOGE(TAG, "Failed to take frame mutex");
                free(bmp_buffer);
            }
        }
    }
}

/**
 * @brief HTTP handler for the /stream endpoint.
 *
 * Serves the BMP image converted from the RGB888 frame.
 *
 * @param req Pointer to the HTTP request
 * @return esp_err_t ESP_OK on success
 */
static esp_err_t stream_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "Client connected to /stream");

    // Set response headers for BMP image
    httpd_resp_set_type(req, "image/bmp");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=\"image.bmp\"");
    httpd_resp_set_hdr(req, "Connection", "close");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    httpd_resp_set_hdr(req, "Pragma", "no-cache");

    uint8_t* bmp_buffer = NULL;
    size_t bmp_len = 0;

    // Acquire the mutex to safely access the current frame
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
        // Send the BMP data
        esp_err_t res = httpd_resp_send(req, (const char*)bmp_buffer, bmp_len);
        free(bmp_buffer); // Free the copied BMP buffer
        if (res != ESP_OK) {
            ESP_LOGE(TAG, "Failed to send BMP data");
            return res;
        }
    } else {
        // No frame available or failed to copy
        ESP_LOGE(TAG, "No BMP frame data available");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No BMP frame data available");
        return ESP_FAIL;
    }

    return ESP_OK;
}

/**
 * @brief HTTP handler for the root '/' endpoint.
 *
 * Serves an HTML page that displays the BMP image.
 *
 * @param req Pointer to the HTTP request
 * @return esp_err_t ESP_OK on success
 */
static esp_err_t root_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "Client connected to /");

    const char* resp_str =
        "<!DOCTYPE html>"
        "<html>"
        "<head>"
        "<title>ESP32 BMP Image</title>"
        "</head>"
        "<body>"
        "<h1>ESP32 BMP Image</h1>"
        "<img src=\"/stream\" alt=\"BMP Image\" />"
        "</body>"
        "</html>";

    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, resp_str, strlen(resp_str));

    return ESP_OK;
}

/**
 * @brief URI handler structure for /stream endpoint.
 */
static const httpd_uri_t stream_uri = {
    .uri       = "/stream",
    .method    = HTTP_GET,
    .handler   = stream_handler,
    .user_ctx  = NULL
};

/**
 * @brief URI handler structure for root '/' endpoint.
 */
static const httpd_uri_t root_uri = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = root_handler,
    .user_ctx  = NULL
};

/**
 * @brief Start the HTTP server and register URI handlers.
 *
 * @return httpd_handle_t Handle to the started HTTP server, or NULL on failure
 */
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
        return server;
    }

    ESP_LOGE(TAG, "Failed to start HTTP server");
    return NULL;
}

/**
 * @brief Application entry point.
 *
 * Initializes synchronization primitives, starts the HTTP server,
 * and creates the frame processing task.
 */
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
