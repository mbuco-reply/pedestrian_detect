// File: components/app_wifi/include/app_wifi.hpp
#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_log.h"

#include <string>

class AppWiFi {
public:
    AppWiFi(const std::string& ssid, const std::string& password);
    ~AppWiFi();
    void run();

private:
    std::string ssid_;
    std::string password_;
    static EventGroupHandle_t wifi_event_group;
    static const int WIFI_CONNECTED_BIT = BIT0;

    static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
};
