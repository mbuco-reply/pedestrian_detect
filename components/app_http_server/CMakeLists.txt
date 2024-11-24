# File: components/app_http_server/CMakeLists.txt

idf_component_register(
    SRCS "src/http_server.cpp"
    INCLUDE_DIRS "include"
    REQUIRES esp_http_server espressif__esp32-camera
)
