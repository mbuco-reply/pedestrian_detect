set(include_dirs    include
                    ../app_camera/include
                    .)

set(requires    console
                esp_lcd
                driver
                espressif__esp32-camera
                esp_timer
                esp_partition
                )

idf_component_register(
    SRCS "src/app_lcd.cpp" "src/image_utils.cpp"
    INCLUDE_DIRS ${include_dirs}
    REQUIRES ${requires}
)
