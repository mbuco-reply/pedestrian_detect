// File: components/app_http_server/src/http_server.cpp
#include "http_server.hpp"
#include "esp_log.h"
#include "freertos/queue.h"
#include "esp_camera.h"

static const char *TAG = "HTTP_SERVER";

// Define the boundary for multipart HTTP response
static const char *STREAM_BOUNDARY = "--frame\r\n";
static const char *STREAM_CONTENT_TYPE = "multipart/x-mixed-replace; boundary=frame";

// HTTP handler for the MJPEG stream
static esp_err_t stream_handler(httpd_req_t *req)
{
    QueueHandle_t stream_queue = (QueueHandle_t) req->user_ctx;
    camera_fb_t *fb = NULL;
    esp_err_t res = ESP_OK;

    // Set the content type without CRLF
    res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set Content-Type: %s", STREAM_CONTENT_TYPE);
        return res;
    }
    ESP_LOGI(TAG, "Content-Type set to %s", STREAM_CONTENT_TYPE);

    // Disable chunked encoding to keep the connection open
    httpd_resp_set_hdr(req, "Connection", "keep-alive");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    httpd_resp_set_hdr(req, "Pragma", "no-cache");

    while (true) {
        // Wait for a frame from the queue with a timeout to detect client disconnect
        if (xQueueReceive(stream_queue, &fb, pdMS_TO_TICKS(5000))) { // 5-second timeout
            if (!fb) {
                ESP_LOGE(TAG, "Null frame received");
                continue;
            }

            if (fb->format != PIXFORMAT_JPEG) {
                ESP_LOGE(TAG, "Non-JPEG frame format: %d", fb->format);
                esp_camera_fb_return(fb);
                continue;
            }

            // Send boundary
            res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
            if (res != ESP_OK) {
                ESP_LOGE(TAG, "Failed to send boundary");
                esp_camera_fb_return(fb);
                break;
            }
            // ESP_LOGI(TAG, "Boundary sent");

            // Prepare and send Content-Type and Content-Length
            char header[128];
            int header_length = snprintf(header, sizeof(header), "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n", fb->len);
            if (header_length < 0 || header_length >= sizeof(header)) {
                ESP_LOGE(TAG, "Header snprintf failed or truncated");
                esp_camera_fb_return(fb);
                break;
            }

            res = httpd_resp_send_chunk(req, header, header_length);
            if (res != ESP_OK) {
                ESP_LOGE(TAG, "Failed to send header");
                esp_camera_fb_return(fb);
                break;
            }
            // ESP_LOGI(TAG, "Header sent: Content-Length=%u", fb->len);

            // Send the JPEG frame
            res = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
            if (res != ESP_OK) {
                ESP_LOGE(TAG, "Failed to send frame");
                esp_camera_fb_return(fb);
                break;
            }
            ESP_LOGI(TAG, "Frame sent: %u bytes", fb->len);

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
        "<head><title>ESP32-CAM Stream</title></head>"
        "<body>"
        "<h1>ESP32-CAM Live Stream</h1>"
        "<img id=\"camera-stream\" src=\"/stream\" width=\"640\" height=\"480\" />"
        "<script>"
        "function refreshImage() {"
        "    var img = document.getElementById('camera-stream');"
        "    img.src = '/stream?t=' + new Date().getTime(); // Prevent caching"
        "}"
        "setInterval(refreshImage, 1000); // Refresh every second"
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
