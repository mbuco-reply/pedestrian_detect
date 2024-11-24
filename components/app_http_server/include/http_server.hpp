// File: components/app_http_server/include/http_server.hpp
#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_http_server.h"
#include "esp_err.h"

// Function to initialize and start the HTTP server
extern "C" esp_err_t start_http_server(QueueHandle_t stream_queue);
