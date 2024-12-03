// File: components/app_http_server/src/http_server.cpp

#include "http_server.hpp"
#include "esp_log.h"
#include "freertos/queue.h"
#include "esp_camera.h"
#include <string.h>
#include "driver/jpeg_encode.h"

static const char *TAG = "HTTP_SERVER";

// Define the boundary for multipart HTTP response
static const char *STREAM_BOUNDARY = "--frame\r\n";
// Define the content type for raw RGB888 stream
static const char *STREAM_CONTENT_TYPE = "multipart/x-mixed-replace; boundary=frame";

// HTTP handler for the RGB888 stream
static esp_err_t stream_handler(httpd_req_t *req)
{
    // Retrieve the frame queue from the user context
    QueueHandle_t stream_queue = (QueueHandle_t) req->user_ctx;
    camera_fb_t *fb = NULL;
    esp_err_t res = ESP_OK;

    // Set the content type to multipart/x-mixed-replace
    res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set Content-Type: %s", STREAM_CONTENT_TYPE);
        return res;
    }
    ESP_LOGI(TAG, "Content-Type set to %s", STREAM_CONTENT_TYPE);

    // Set headers to prevent caching and keep connection alive
    httpd_resp_set_hdr(req, "Connection", "keep-alive");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    httpd_resp_set_hdr(req, "Pragma", "no-cache");

    while (true) {
        // Wait for a frame from the queue with a 5-second timeout
        if (xQueueReceive(stream_queue, &fb, pdMS_TO_TICKS(5000))) {
            if (!fb) {
                ESP_LOGE(TAG, "Null frame received");
                continue;
            }

            // Ensure the frame is in RGB888 format
            if (fb->format != PIXFORMAT_RGB888) {
                ESP_LOGE(TAG, "Non-RGB888 frame format: %d", fb->format);
                esp_camera_fb_return(fb);
                continue;
            }

            // Log frame dimensions for debugging
            ESP_LOGI(TAG, "Frame received: %dx%d", fb->width, fb->height);

            // Send the boundary
            res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
            if (res != ESP_OK) {
                ESP_LOGE(TAG, "Failed to send boundary");
                esp_camera_fb_return(fb);
                break;
            }

            // Prepare and send Content-Type and Content-Length headers
            // Use a suitable Content-Type for raw RGB888
            // Note: 'image/raw' is not a standard MIME type. Consider using 'application/octet-stream' if necessary.
            const char *image_content_type = "image/raw"; // Custom MIME type
            char header[128];
            int header_length = snprintf(header, sizeof(header),
                                         "Content-Type: %s\r\nContent-Length: %u\r\n\r\n",
                                         image_content_type, fb->len);
            if (header_length < 0 || header_length >= sizeof(header)) {
                ESP_LOGE(TAG, "Header snprintf failed or truncated");
                esp_camera_fb_return(fb);
                break;
            }

            // Send the headers
            res = httpd_resp_send_chunk(req, header, header_length);
            if (res != ESP_OK) {
                ESP_LOGE(TAG, "Failed to send headers");
                esp_camera_fb_return(fb);
                break;
            }

            // Send the raw RGB888 frame data
            res = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
            if (res != ESP_OK) {
                ESP_LOGE(TAG, "Failed to send frame data");
                esp_camera_fb_return(fb);
                break;
            }
            ESP_LOGI(TAG, "Sent RGB888 frame: %u bytes", fb->len);

            // Return the frame buffer to free memory
            esp_camera_fb_return(fb);
        } else {
            // Timeout occurred; assume client disconnected
            ESP_LOGW(TAG, "Stream timeout or client disconnected");
            break;
        }
    }

    return res;
}

// HTTP handler to serve the main HTML page
static esp_err_t index_handler(httpd_req_t *req)
{
    const char *resp_str =
        "<!DOCTYPE html>"
        "<html>"
        "<head><title>ESP32-CAM RGB888 Stream</title></head>"
        "<body>"
        "<h1>ESP32-CAM Live Stream (RGB888)</h1>"
        "<canvas id=\"camera-stream\" width=\"640\" height=\"480\"></canvas>"
        "<script>"
        "var canvas = document.getElementById('camera-stream');"
        "var ctx = canvas.getContext('2d');"
        "var width = canvas.width;"
        "var height = canvas.height;"
        "var imgData = ctx.createImageData(width, height);"

        "// Function to fetch and render RGB888 frames"
        "function fetchStream() {"
        "    var xhr = new XMLHttpRequest();"
        "    xhr.open('GET', '/stream', true);"
        "    xhr.responseType = 'arraybuffer';"
        "    xhr.onload = function(e) {"
        "        if (this.status == 200) {"
        "            var array = new Uint8Array(this.response);"
        "            // Convert RGB888 to RGBA8888"
        "            for (var i = 0; i < array.length; i += 3) {"
        "                var j = (i / 3) * 4;"
        "                imgData.data[j]     = array[i];     // R"
        "                imgData.data[j + 1] = array[i + 1]; // G"
        "                imgData.data[j + 2] = array[i + 2]; // B"
        "                imgData.data[j + 3] = 255;          // A (opaque)"
        "            }"
        "            ctx.putImageData(imgData, 0, 0);"
        "        } else {"
        "            console.error('Failed to fetch stream: ' + this.status);"
        "        }"
        "    };"
        "    xhr.onerror = function() {"
        "        console.error('XHR error while fetching stream');"
        "    };"
        "    xhr.send();"
        "}"

        "// Poll the server for new frames every second"
        "setInterval(fetchStream, 1000); // Adjust the interval as needed"
        "</script>"
        "</body>"
        "</html>";

    // Set the Content-Type header
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, resp_str, strlen(resp_str));
}

// URI definitions
static const httpd_uri_t index_uri = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = index_handler,
    .user_ctx  = NULL
};

static httpd_uri_t stream_uri = {
    .uri       = "/stream",
    .method    = HTTP_GET,
    .handler   = stream_handler,
    .user_ctx  = NULL  // To be set when registering the handler
};

// Start the HTTP server
extern "C" esp_err_t start_http_server(QueueHandle_t stream_queue)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    // Start the HTTP server
    ESP_LOGI(TAG, "Starting HTTP Server");
    if (httpd_start(&server, &config) == ESP_OK) {
        // Assign the stream_queue to user_ctx for the stream_handler
        stream_uri.user_ctx = (void *)stream_queue;

        // Register URI handlers
        httpd_register_uri_handler(server, &index_uri);
        httpd_register_uri_handler(server, &stream_uri);

        ESP_LOGI(TAG, "HTTP Server started on port: '%d'", config.server_port);
        return ESP_OK;
    }

    ESP_LOGE(TAG, "Failed to start HTTP server");
    return ESP_FAIL;
}
