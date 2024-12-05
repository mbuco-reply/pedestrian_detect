set(include_dirs    include
                    .)

set(requires    console
                driver
                espressif__esp32-camera
                esp_timer
                esp_partition
                esp_jpeg
                )

set(embed_files     "pedestrian.jpeg")

idf_component_register(
    SRCS "src/app_camera.cpp"
    INCLUDE_DIRS ${include_dirs}
    REQUIRES ${requires}
    EMBED_FILES ${embed_files}
)
