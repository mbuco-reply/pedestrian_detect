# File: components/app_wifi/CMakeLists.txt

idf_component_register(
    SRCS "src/app_wifi.cpp"
    INCLUDE_DIRS "include"
    REQUIRES esp_wifi esp_event esp_netif
)
