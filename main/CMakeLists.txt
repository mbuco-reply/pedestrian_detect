set(src_dirs        ./)

set(include_dirs    ./)

set(requires        nvs_flash
                    app_camera
                    app_lcd
                    app_detect
                    app_http_server
                    app_wifi)

idf_component_register(SRC_DIRS ${src_dirs} INCLUDE_DIRS ${include_dirs} REQUIRES ${requires} EMBED_FILES ${embed_files})
