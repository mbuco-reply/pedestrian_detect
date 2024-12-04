// File: components/app_http_server/http_server.cpp

#include "http_server.hpp"
#include "esp_log.h"
#include "esp_camera.h" // For camera_fb_t
#include <string.h>
#include <stdio.h>      // For snprintf
#include <stdlib.h>     // For malloc and free

// Define a tag for logging
static const char* TAG = "app_http_server";

// Variable to hold the stream queue handle
static QueueHandle_t s_stream_queue = NULL;

// Forward declarations
static esp_err_t index_handler(httpd_req_t *req);
static esp_err_t stream_handler(httpd_req_t *req);

// Function to initialize and start the HTTP server
extern "C" esp_err_t start_http_server(QueueHandle_t stream_queue)
{
    s_stream_queue = stream_queue;

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    // Start the HTTP server
    httpd_handle_t server = NULL;
    esp_err_t ret = httpd_start(&server, &config);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server");
        return ret;
    }

    // Register URI handler for the root path (serving the HTML page)
    httpd_uri_t index_uri = {
        .uri       = "/",
        .method    = HTTP_GET,
        .handler   = index_handler,
        .user_ctx  = NULL
    };

    ret = httpd_register_uri_handler(server, &index_uri);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register index URI handler");
        return ret;
    }

    // Register URI handler for the image data
    httpd_uri_t stream_uri = {
        .uri       = "/image_data",
        .method    = HTTP_GET,
        .handler   = stream_handler,
        .user_ctx  = NULL
    };

    ret = httpd_register_uri_handler(server, &stream_uri);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register image URI handler");
        return ret;
    }

    return ESP_OK;
}

// HTML page handler
static esp_err_t index_handler(httpd_req_t *req)
{
    const char* resp_str =
        "<!DOCTYPE html>"
        "<html>"
        "<head>"
        "<meta charset=\"UTF-8\">"
        "<title>ESP32 Image</title>"
        "</head>"
        "<body>"
        "<h1>ESP32 Image Rendering</h1>"
        "<canvas id=\"canvas\"></canvas>"
        "<script>"
        "fetch('/image_data')"
        ".then(response => response.json())"
        ".then(data => {"
            "const width = data.width;"
            "const height = data.height;"
            "const hexString = data.data;"
            "const canvas = document.getElementById('canvas');"
            "canvas.width = width;"
            "canvas.height = height;"
            "const ctx = canvas.getContext('2d');"
            "const imageData = ctx.createImageData(width, height);"
            "const pixelData = imageData.data;"
            "let j = 0;"
            "for (let i = 0; i < hexString.length; i += 6) {"
                "const r = parseInt(hexString.substr(i, 2), 16);"
                "const g = parseInt(hexString.substr(i + 2, 2), 16);"
                "const b = parseInt(hexString.substr(i + 4, 2), 16);"
                "pixelData[j++] = r;"
                "pixelData[j++] = g;"
                "pixelData[j++] = b;"
                "pixelData[j++] = 255;" // Alpha channel
            "}"
            "ctx.putImageData(imageData, 0, 0);"
        "})"
        ".catch(err => console.error(err));"
        "</script>"
        "</body>"
        "</html>";

    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, resp_str, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// Image data handler
static esp_err_t stream_handler(httpd_req_t *req)
{
    camera_fb_t *frame = NULL;

    // Wait indefinitely for a frame from the queue
    if (xQueueReceive(s_stream_queue, &frame, portMAX_DELAY) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to receive frame from queue");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Received frame from queue");

    // Set the content type to indicate JSON data
    httpd_resp_set_type(req, "application/json");

    // Prepare JSON response
    const size_t max_chunk_size = 1024; // Adjust as needed
    char *json_chunk = (char *)malloc(max_chunk_size);
    if (!json_chunk) {
        ESP_LOGE(TAG, "Failed to allocate memory for JSON chunk");
        httpd_resp_send_500(req);
        esp_camera_fb_return(frame);
        return ESP_FAIL;
    }

    // Start JSON object
    const char *json_start = "{ \"width\": %d, \"height\": %d, \"data\": \"";
    int len = snprintf(json_chunk, max_chunk_size, json_start, frame->width, frame->height);
    httpd_resp_send_chunk(req, json_chunk, len);

    // Convert binary data to hex and send in chunks
    size_t bytes_processed = 0;
    while (bytes_processed < frame->len) {
        size_t bytes_to_process = frame->len - bytes_processed;
        if (bytes_to_process > (max_chunk_size - 2) / 2) { // Each byte becomes two hex chars
            bytes_to_process = (max_chunk_size - 2) / 2;
        }

        size_t json_index = 0;
        for (size_t i = 0; i < bytes_to_process; i++) {
            snprintf(&json_chunk[json_index], 3, "%02X", frame->buf[bytes_processed + i]);
            json_index += 2;
        }

        bytes_processed += bytes_to_process;

        // Send the chunk
        httpd_resp_send_chunk(req, json_chunk, json_index);
    }

    // End JSON object
    const char *json_end = "\" }";
    httpd_resp_send_chunk(req, json_end, strlen(json_end));

    // Signal the end of the response
    httpd_resp_send_chunk(req, NULL, 0);

    free(json_chunk);
    esp_camera_fb_return(frame);

    return ESP_OK;
}
