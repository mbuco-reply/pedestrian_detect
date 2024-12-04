set(include_dirs    include
                    ../app_camera/include
                    .)

set(requires    console
                pedestrian_detect
                esp_lcd
                esp_jpeg
                driver
                espressif__esp32-camera
                esp_timer
                esp_partition
                fb_gfx
                esp-dl
                )

idf_component_register(
    SRCS "src/app_detect.cpp" "src/who_ai_utils.cpp"
    INCLUDE_DIRS ${include_dirs}
    REQUIRES ${requires}
)
